`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_builder (
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

    input  logic [TRACE_WIDTH-1:0] mem_rdata_i
);

  localparam int unsigned CHUNK_PTR_W = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned BR_CNT_W    = $clog2(MAX_BRANCHES + 1);

  typedef enum logic [2:0] {
    IDLE, ACCUM, LOOKUP, COMPARE, COMMIT
  } state_t;

  state_t state_q, state_d;
  trace_data_t trace_q, trace_d;

  logic [CHUNK_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;
  logic [BR_CNT_W-1:0]    br_cnt_q, br_cnt_d;

  logic [PC_WIDTH-1:0]    last_branch_target_q, last_branch_target_d;
  logic [TRACE_ADDRW-1:0] sram_wr_ptr_q, sram_wr_ptr_d;
  logic [TRACE_ADDRW-1:0] unique_count_q, unique_count_d;

  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q, commit_data_d;

  logic [GHR_WIDTH-1:0]   trace_start_ghr_q, trace_start_ghr_d;
  
  logic [TRACE_ADDRW-1:0] lookup_idx_q, lookup_idx_d;
  trace_data_t            sram_trace;
  logic                   duplicate_found;

  assign instr_i.ready = 1'b1;

  assign sram_trace = mem_rdata_i;

  assign mem_req_o   = (state_q == LOOKUP) || commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = (state_q == LOOKUP) ? lookup_idx_q : sram_wr_ptr_q;
  assign mem_wdata_o = commit_data_q;
  assign mem_be_o    = {BE_WIDTH{1'b1}};

  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q              <= IDLE;
      trace_q              <= '0;
      chunk_ptr_q          <= '0;
      br_cnt_q             <= '0;
      last_branch_target_q <= '0;
      sram_wr_ptr_q        <= '0;
      unique_count_q       <= '0;
      commit_valid_q       <= 1'b0;
      commit_data_q        <= '0;
      trace_start_ghr_q    <= '0;
      lookup_idx_q         <= '0;
    end else begin
      state_q              <= state_d;
      trace_q              <= trace_d;
      chunk_ptr_q          <= chunk_ptr_d;
      br_cnt_q             <= br_cnt_d;
      last_branch_target_q <= last_branch_target_d;
      sram_wr_ptr_q        <= sram_wr_ptr_d;
      unique_count_q       <= unique_count_d;
      commit_valid_q       <= commit_valid_d;
      commit_data_q        <= commit_data_d;
      trace_start_ghr_q    <= trace_start_ghr_d;
      lookup_idx_q         <= lookup_idx_d;
    end
  end

  always_comb begin
    logic [CHUNK_PTR_W-1:0] temp_chunk_ptr;
    logic [BR_CNT_W-1:0]    temp_br_cnt;
    logic                   is_compressed;
    logic                   has_space;
    logic                   found_taken;
    logic                   branch_is_last;
    logic [CHUNK_PTR_W-1:0] branch_slot;

    state_d              = state_q;
    trace_d              = trace_q;
    chunk_ptr_d          = chunk_ptr_q;
    br_cnt_d             = br_cnt_q;
    last_branch_target_d = last_branch_target_q;
    sram_wr_ptr_d        = sram_wr_ptr_q;
    unique_count_d       = unique_count_q;
    commit_valid_d       = 1'b0;
    commit_data_d        = commit_data_q;
    trace_start_ghr_d    = trace_start_ghr_q;
    lookup_idx_d         = lookup_idx_q;
    duplicate_found      = 1'b0;

    temp_chunk_ptr = '0;
    temp_br_cnt    = '0;
    is_compressed  = 1'b0;
    has_space      = 1'b0;
    found_taken    = 1'b0;
    branch_is_last = 1'b0;
    branch_slot    = '0;

    if (flush_i) begin
      state_d     = IDLE;
      chunk_ptr_d = '0;
      br_cnt_d    = '0;
      trace_d     = '0;
    end else begin

      case (state_q)

        IDLE: begin
          chunk_ptr_d = '0;
          br_cnt_d    = '0;
          trace_d     = '0;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i] && !found_taken) begin
              found_taken = 1'b1;
              branch_slot = CHUNK_PTR_W'(i);
              
              branch_is_last = 1'b1;
              for (int j = i + 1; j < SLOTS_PER_CYCLE; j++) begin
                if (instr_i.valid[j]) branch_is_last = 1'b0;
              end
            end
          end

          if (found_taken && !branch_is_last) begin
            trace_start_ghr_d = ghr_i;
            temp_chunk_ptr    = '0;
            temp_br_cnt       = '0;

            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              if (instr_i.valid[i] && (CHUNK_PTR_W'(i) <= branch_slot)) begin
                is_compressed = (instr_i.inst[i][1:0] != 2'b11);

                if (i == 0)
                  trace_d.base_pc = instr_i.pc[i];

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

                if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                  temp_br_cnt          = temp_br_cnt + 1;
                  last_branch_target_d = instr_i.target[i];
                  if (temp_br_cnt > 1 && temp_br_cnt <= MAX_BRANCHES)
                    trace_d.branch_flags[temp_br_cnt-2] = 1'b1;
                end
              end
            end

            chunk_ptr_d = temp_chunk_ptr;
            br_cnt_d    = temp_br_cnt;
            state_d     = ACCUM;
          end
        end

        ACCUM: begin
          temp_chunk_ptr = chunk_ptr_q;
          temp_br_cnt    = br_cnt_q;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            is_compressed = (instr_i.inst[i][1:0] != 2'b11);
            has_space = is_compressed ?
                        (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                        (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

            if (instr_i.valid[i] && has_space) begin
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

              if (instr_i.is_branch[i] && instr_i.taken[i] && temp_br_cnt < MAX_BRANCHES) begin
                temp_br_cnt          = temp_br_cnt + 1;
                last_branch_target_d = instr_i.target[i];
                if (temp_br_cnt > 1)
                  trace_d.branch_flags[temp_br_cnt-2] = 1'b1;
              end
            end
          end

          chunk_ptr_d = temp_chunk_ptr;
          br_cnt_d    = temp_br_cnt;
          lookup_idx_d = '0;
          state_d     = LOOKUP;
        end

        LOOKUP: begin
          if (lookup_idx_q < unique_count_q) begin
            lookup_idx_d = lookup_idx_q + 1;
            state_d = COMPARE;
          end else begin
            state_d = COMMIT;
          end
        end

        COMPARE: begin
          if (sram_trace.valid && 
              sram_trace.base_pc == trace_q.base_pc &&
              sram_trace.branch_flags == trace_q.branch_flags) begin
            duplicate_found = 1'b1;
            state_d = IDLE;
            chunk_ptr_d = '0;
            br_cnt_d = '0;
            trace_d = '0;
          end else begin
            state_d = LOOKUP;
          end
        end

        COMMIT: begin
          commit_valid_d       = 1'b1;
          trace_d.valid        = 1'b1;
          trace_d.target_addr  = last_branch_target_q;
          trace_d.num_branches = br_cnt_q;
          commit_data_d        = trace_d;
          sram_wr_ptr_d        = sram_wr_ptr_q + 1;
          unique_count_d       = unique_count_q + 1;
          state_d              = IDLE;
          chunk_ptr_d          = '0;
          br_cnt_d             = '0;
          trace_d              = '0;
        end

      endcase
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (commit_valid_q) begin
      $display("[TC-UNIQUE] Trace #%0d: pc=%h target=%h branches=%0d", 
               unique_count_q, trace_q.base_pc, last_branch_target_q, br_cnt_q);
    end
  end
  
  always_ff @(posedge clk_i) begin
    if (state_q == COMPARE && duplicate_found) begin
      $display("[TC-DUP] Duplicate found: pc=%h", trace_q.base_pc);
    end
  end
`endif

endmodule