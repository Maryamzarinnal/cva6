`timescale 1ns/1ps
import trace_cache_pkg::*;

// Builds a trace from fetch windows: start at the window where a branch is taken, then keep
// adding instructions from the next window(s) until we hit MAX_INSTR_PER_TRACE (one fetch
// window worth, e.g. 4). Tag = (base_pc, branch_flags). We fill at fetch; on write we overwrite
// branch_flags with resolved outcomes when we have them (Rotenberg-style). Stored as 16-bit
// chunks; valid_chunks marks instruction starts. MAX_INSTR_PER_TRACE must be <= TRACE_LEN.

module trace_builder #(
  parameter int unsigned MAX_INSTR_PER_TRACE = TRACE_LEN
) (
    input  logic clk_i,
    input  logic rst_ni,

    tracebuilder_instr_if.consumer instr_i,

    input  logic [GHR_WIDTH-1:0] ghr_i,
    input  logic                 flush_i,

    output logic                   trace_valid_o,
    output logic [TRACE_WIDTH-1:0] trace_data_o,

    output logic                   mem_req_o,
    output logic                   mem_we_o,
    output logic [TRACE_ADDRW-1:0] mem_addr_o,
    output logic [TRACE_WIDTH-1:0] mem_wdata_o,
    output logic [BE_WIDTH-1:0]    mem_be_o,
    output logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] mem_branch_pcs_o
);

  localparam int unsigned CHUNK_PTR_W = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned BR_CNT_W    = BR_CNT_WIDTH;
  localparam int unsigned START_CNT_W = $clog2(TRACE_LEN + 1);
  localparam int unsigned MAX_TAKEN   = 3;

  typedef enum logic [1:0] {
    IDLE,
    ACCUM
  } state_t;

  state_t state_q, state_d;
  trace_data_t trace_q, trace_d;
  logic [CHUNK_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;
  logic [BR_CNT_W-1:0]    br_cnt_q, br_cnt_d;
  logic [START_CNT_W-1:0] instr_start_cnt_q, instr_start_cnt_d;
  logic [PC_WIDTH-1:0]    last_branch_target_q, last_branch_target_d;
  logic [PC_WIDTH-1:0]    last_instr_pc_q, last_instr_pc_d;
  logic                   last_instr_compressed_q, last_instr_compressed_d;
  logic                   last_instr_was_taken_q, last_instr_was_taken_d;

  logic [TRACE_ADDRW-1:0] sram_wr_addr_q, sram_wr_addr_d;
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q,  commit_data_d;
  logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] branch_pcs_q, branch_pcs_d;
  logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] commit_branch_pcs_q, commit_branch_pcs_d;
  logic [GHR_WIDTH-1:0]   trace_start_ghr_q, trace_start_ghr_d;
  logic [CHUNK_PTR_W-1:0] commit_chunk_ptr_q, commit_chunk_ptr_d;
  logic [1:0]             taken_cnt_q, taken_cnt_d;

  // Per-set duplicate filter: remembers the last committed (pc, flags) per set.
  logic                        dup_valid [(1 << TRACE_ADDRW)];
  logic [PC_WIDTH-1:0]         dup_pc    [(1 << TRACE_ADDRW)];
  logic [CHUNKS_PER_TRACE-1:0] dup_flags [(1 << TRACE_ADDRW)];
  logic [BR_CNT_WIDTH-1:0]     dup_num_branches [(1 << TRACE_ADDRW)];

  assign instr_i.ready = 1'b1;
  assign mem_req_o     = commit_valid_q;
  assign mem_we_o      = commit_valid_q;
  assign mem_addr_o    = sram_wr_addr_q;
  assign mem_wdata_o       = commit_data_q;
  assign mem_be_o          = {BE_WIDTH{1'b1}};
  assign mem_branch_pcs_o  = commit_branch_pcs_q;
  assign trace_valid_o     = commit_valid_q;
  assign trace_data_o      = commit_data_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q                 <= IDLE;
      trace_q                 <= '0;
      chunk_ptr_q             <= '0;
      br_cnt_q                <= '0;
      instr_start_cnt_q       <= '0;
      last_branch_target_q    <= '0;
      last_instr_pc_q         <= '0;
      last_instr_compressed_q <= 1'b0;
      last_instr_was_taken_q  <= 1'b0;
      sram_wr_addr_q          <= '0;
      commit_valid_q          <= 1'b0;
      commit_data_q           <= '0;
      branch_pcs_q            <= '0;
      commit_branch_pcs_q     <= '0;
      trace_start_ghr_q       <= '0;
      commit_chunk_ptr_q      <= '0;
      taken_cnt_q             <= '0;
    end else begin
      state_q                 <= state_d;
      trace_q                 <= trace_d;
      chunk_ptr_q             <= chunk_ptr_d;
      br_cnt_q                <= br_cnt_d;
      instr_start_cnt_q       <= instr_start_cnt_d;
      last_branch_target_q    <= last_branch_target_d;
      last_instr_pc_q         <= last_instr_pc_d;
      last_instr_compressed_q <= last_instr_compressed_d;
      last_instr_was_taken_q  <= last_instr_was_taken_d;
      sram_wr_addr_q          <= sram_wr_addr_d;
      commit_valid_q          <= commit_valid_d;
      commit_data_q           <= commit_data_d;
      branch_pcs_q            <= branch_pcs_d;
      commit_branch_pcs_q     <= commit_branch_pcs_d;
      trace_start_ghr_q       <= trace_start_ghr_d;
      commit_chunk_ptr_q      <= commit_chunk_ptr_d;
      taken_cnt_q             <= taken_cnt_d;
    end
  end

  always_comb begin
    logic [CHUNK_PTR_W-1:0] temp_chunk_ptr;
    logic [BR_CNT_W-1:0]    temp_br_cnt;
    logic [START_CNT_W-1:0] temp_start_cnt;
    logic                   is_compressed;
    logic                   has_space;
    logic                   found_taken;
    logic [CHUNK_PTR_W-1:0] branch_slot;
    logic                   hit_taken;
    logic                   trace_full;

    state_d                 = state_q;
    trace_d                 = trace_q;
    chunk_ptr_d             = chunk_ptr_q;
    br_cnt_d                = br_cnt_q;
    instr_start_cnt_d       = instr_start_cnt_q;
    last_branch_target_d    = last_branch_target_q;
    last_instr_pc_d         = last_instr_pc_q;
    last_instr_compressed_d = last_instr_compressed_q;
    last_instr_was_taken_d  = last_instr_was_taken_q;
    sram_wr_addr_d          = sram_wr_addr_q;
    commit_valid_d          = 1'b0;
    commit_data_d           = commit_data_q;
    commit_branch_pcs_d     = commit_branch_pcs_q;
    branch_pcs_d            = branch_pcs_q;
    trace_start_ghr_d       = trace_start_ghr_q;
    commit_chunk_ptr_d      = commit_chunk_ptr_q;
    taken_cnt_d             = taken_cnt_q;

    temp_chunk_ptr          = '0;
    temp_br_cnt             = '0;
    temp_start_cnt          = '0;
    is_compressed           = 1'b0;
    has_space               = 1'b0;
    found_taken             = 1'b0;
    branch_slot             = '0;
    hit_taken               = 1'b0;
    trace_full              = 1'b0;

    if (flush_i) begin
      state_d                 = IDLE;
      chunk_ptr_d             = '0;
      br_cnt_d                = '0;
      instr_start_cnt_d       = '0;
      trace_d                 = '0;
      last_instr_pc_d         = '0;
      last_instr_compressed_d = 1'b0;
      last_instr_was_taken_d  = 1'b0;
      taken_cnt_d             = '0;
    end else begin
      case (state_q)

        IDLE: begin
          if (|instr_i.consumed && !instr_i.serving_unaligned) begin
            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i] && !found_taken) begin
                found_taken = 1'b1;
                branch_slot = CHUNK_PTR_W'(i);
              end
            end

            if (found_taken) begin
              trace_start_ghr_d       = ghr_i;
              temp_chunk_ptr          = '0;
              temp_br_cnt             = '0;
              temp_start_cnt          = '0;
              chunk_ptr_d             = '0;
              br_cnt_d                = '0;
              instr_start_cnt_d       = '0;
              trace_d                 = '0;
              last_instr_pc_d         = '0;
              last_instr_compressed_d = 1'b0;
              last_instr_was_taken_d  = 1'b0;

              trace_d.base_pc = pc_align_16(instr_i.pc[0]);

              for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
                if (instr_i.valid[i] && (CHUNK_PTR_W'(i) <= branch_slot) &&
                    (temp_start_cnt < START_CNT_W'(MAX_INSTR_PER_TRACE))) begin
                  is_compressed = (instr_i.inst[i][1:0] != 2'b11);

                  last_instr_pc_d         = instr_i.pc[i];
                  last_instr_compressed_d = is_compressed;
                  last_instr_was_taken_d  = instr_i.is_branch[i] && instr_i.taken[i];

                  if (!is_compressed) begin
                    trace_d.chunks[temp_chunk_ptr]         = instr_i.inst[i][15:0];
                    trace_d.chunks[temp_chunk_ptr+1]       = instr_i.inst[i][31:16];
                    trace_d.valid_chunks[temp_chunk_ptr]   = 1'b1;
                    trace_d.valid_chunks[temp_chunk_ptr+1] = 1'b0;
                    temp_chunk_ptr = temp_chunk_ptr + 2;
                  end else begin
                    trace_d.chunks[temp_chunk_ptr]       = instr_i.inst[i][15:0];
                    trace_d.valid_chunks[temp_chunk_ptr] = 1'b1;
                    temp_chunk_ptr = temp_chunk_ptr + 1;
                  end
                  temp_start_cnt = temp_start_cnt + START_CNT_W'(1);

                  if (instr_i.is_branch[i]) begin
                    trace_d.lookup_branch_flags[temp_br_cnt] = instr_i.taken[i];
                    trace_d.branch_flags[temp_br_cnt]        = instr_i.taken[i];
                    branch_pcs_d[temp_br_cnt]                = instr_i.pc[i];
                    if (instr_i.taken[i])
                      last_branch_target_d = instr_i.target[i];
                    temp_br_cnt = temp_br_cnt + 1;
                  end
                end
              end

              trace_d.lookup_num_branches = BR_CNT_W'(temp_br_cnt);
              trace_d.num_branches        = BR_CNT_W'(temp_br_cnt);
              chunk_ptr_d                 = temp_chunk_ptr;
              br_cnt_d                    = temp_br_cnt;
              instr_start_cnt_d           = temp_start_cnt;
              taken_cnt_d                 = 2'd1;
              state_d                     = ACCUM;
            end
          end
        end

        ACCUM: begin
          if (|instr_i.consumed) begin
            temp_chunk_ptr = chunk_ptr_q;
            temp_br_cnt    = br_cnt_q;
            temp_start_cnt = instr_start_cnt_q;
            hit_taken      = 1'b0;
            trace_full     = 1'b0;

            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              is_compressed = (instr_i.inst[i][1:0] != 2'b11);
              has_space = is_compressed ?
                          (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                          (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

              if (instr_i.valid[i] && has_space && !hit_taken &&
                  (temp_start_cnt < START_CNT_W'(MAX_INSTR_PER_TRACE))) begin
                last_instr_pc_d         = instr_i.pc[i];
                last_instr_compressed_d = is_compressed;
                last_instr_was_taken_d  = instr_i.is_branch[i] && instr_i.taken[i];

                if (!is_compressed) begin
                  trace_d.chunks[temp_chunk_ptr]         = instr_i.inst[i][15:0];
                  trace_d.chunks[temp_chunk_ptr+1]       = instr_i.inst[i][31:16];
                  trace_d.valid_chunks[temp_chunk_ptr]   = 1'b1;
                  trace_d.valid_chunks[temp_chunk_ptr+1] = 1'b0;
                  temp_chunk_ptr = temp_chunk_ptr + 2;
                end else begin
                  trace_d.chunks[temp_chunk_ptr]       = instr_i.inst[i][15:0];
                  trace_d.valid_chunks[temp_chunk_ptr] = 1'b1;
                  temp_chunk_ptr = temp_chunk_ptr + 1;
                end
                temp_start_cnt = temp_start_cnt + START_CNT_W'(1);

                if (instr_i.is_branch[i]) begin
                  trace_d.branch_flags[temp_br_cnt] = instr_i.taken[i];
                  branch_pcs_d[temp_br_cnt]         = instr_i.pc[i];
                  if (instr_i.taken[i]) begin
                    last_branch_target_d = instr_i.target[i];
                    taken_cnt_d = taken_cnt_q + 2'd1;
                    hit_taken = 1'b1;
                  end
                  temp_br_cnt = temp_br_cnt + 1;
                end
              end else if (instr_i.valid[i] &&
                           (!has_space || (temp_start_cnt >= START_CNT_W'(MAX_INSTR_PER_TRACE)))) begin
                trace_full = 1'b1;
              end
            end

            chunk_ptr_d       = temp_chunk_ptr;
            br_cnt_d          = temp_br_cnt;
            instr_start_cnt_d = temp_start_cnt;
            trace_d.num_branches = BR_CNT_W'(temp_br_cnt);

            if (trace_full ||
                (temp_start_cnt >= START_CNT_W'(MAX_INSTR_PER_TRACE) && temp_chunk_ptr > 0) ||
                (taken_cnt_d >= MAX_TAKEN[1:0] && temp_chunk_ptr > 0)) begin
              logic [TRACE_ADDRW-1:0] candidate_addr;
              logic                   is_duplicate;

              commit_chunk_ptr_d  = temp_chunk_ptr;
              trace_d.valid       = 1'b1;

              trace_d.target_addr = last_instr_was_taken_d
                                    ? last_branch_target_d
                                    : last_instr_pc_d + (last_instr_compressed_d ? 64'h2 : 64'h4);

              candidate_addr = tc_index(trace_d.base_pc, trace_d.lookup_branch_flags);

              is_duplicate = dup_valid[candidate_addr]
                           && (trace_d.base_pc      == dup_pc[candidate_addr])
                           && (trace_d.lookup_branch_flags == dup_flags[candidate_addr])
                           && (trace_d.lookup_num_branches == dup_num_branches[candidate_addr]);

              if (!is_duplicate) begin
                sram_wr_addr_d     = candidate_addr;
                commit_valid_d     = 1'b1;
                commit_data_d      = trace_d;
                commit_branch_pcs_d = branch_pcs_d;
              end

              state_d                 = IDLE;
              chunk_ptr_d             = '0;
              br_cnt_d                = '0;
              instr_start_cnt_d       = '0;
              taken_cnt_d             = '0;
              trace_d                 = '0;
            end
          end
        end

      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < (1 << TRACE_ADDRW); i++) begin
        dup_valid[i] <= 1'b0;
        dup_num_branches[i] <= '0;
      end
    end else if (commit_valid_d) begin
      trace_data_t dup_tmp;
      dup_tmp = trace_data_t'(commit_data_d);
      dup_valid[sram_wr_addr_d] <= 1'b1;
      dup_pc[sram_wr_addr_d]    <= dup_tmp.base_pc;
      dup_flags[sram_wr_addr_d] <= dup_tmp.lookup_branch_flags;
      dup_num_branches[sram_wr_addr_d] <= dup_tmp.lookup_num_branches;
    end
  end

`ifndef SYNTHESIS
  `ifdef TRACE_CACHE_DEBUG_VERBOSE
  always_ff @(posedge clk_i) begin
    if (commit_valid_q) begin
      trace_data_t dbg;
      dbg = trace_data_t'(commit_data_q);
      $display("[TC-BUILDER] ---- TRACE COMMITTED (non-duplicate) ----");
      $display("[TC-BUILDER]   SRAM addr  = %0d", sram_wr_addr_q);
      $display("[TC-BUILDER]   base_pc    = 0x%h", dbg.base_pc);
      $display("[TC-BUILDER]   HASH: pc[%0d:4]=0x%h ^ pc[%0d:%0d]=0x%h ^ pc[%0d:%0d]=0x%h ^ flags[1:0]=%b => set=%0d",
               TRACE_ADDRW+3,
               dbg.base_pc[TRACE_ADDRW+3:4],
               2*TRACE_ADDRW+3, TRACE_ADDRW+4,
               dbg.base_pc[2*TRACE_ADDRW+3:TRACE_ADDRW+4],
               3*TRACE_ADDRW+3, 2*TRACE_ADDRW+4,
               dbg.base_pc[3*TRACE_ADDRW+3:2*TRACE_ADDRW+4],
               dbg.branch_flags[TC_INDEX_FLAG_BITS-1:0],
               sram_wr_addr_q);
      $display("[TC-BUILDER]   target     = 0x%h", dbg.target_addr);
      $display("[TC-BUILDER]   #branches  = %0d", dbg.num_branches);
      $display("[TC-BUILDER]   br_flags   = %b", dbg.branch_flags);
      $display("[TC-BUILDER]   chunks used= %0d", commit_chunk_ptr_q);
      for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
        if (dbg.valid_chunks[i])
          $display("[TC-BUILDER]   chunk[%0d]  = 0x%h (instr start)", i, dbg.chunks[i]);
        else if (i > 0 && dbg.valid_chunks[i-1])
          $display("[TC-BUILDER]   chunk[%0d]  = 0x%h (instr high half)", i, dbg.chunks[i]);
      end
      $display("[TC-BUILDER] ---------------------------");
    end
  end
  `endif
`endif

endmodule
