`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// Builds a trace from fetch windows: start at the window where a branch is taken, then keep
// adding instructions from the next window(s) until we hit MAX_INSTR_PER_TRACE (one fetch
// window worth, e.g. 4). Tag = (base_pc, branch_flags). We fill at fetch; on write we overwrite
// branch_flags with resolved outcomes when we have them (Rotenberg-style). Stored as 16-bit
// chunks; valid_chunks marks instruction starts. MAX_INSTR_PER_TRACE must be <= TRACE_LEN.
//
// IMPORTANT SAFETY RULE:
//   Any trace containing indirect control flow (jalr / c.jr / c.jalr) is dropped and never
//   committed. Those targets are dynamic and not safe to replay from historical trace data.

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
  localparam int unsigned TAKEN_CNT_W = TAKEN_CNT_WIDTH;

  typedef enum logic [1:0] {
    IDLE,
    ACCUM
  } state_t;

  function automatic logic is_indirect_cf_instr(input logic [INSTR_WIDTH-1:0] instr);
    logic is_rvc;
    begin
      is_rvc = (instr[1:0] != 2'b11);

      // 32-bit jalr
      if (!is_rvc && (instr[6:0] == OpcodeJalr)) begin
        is_indirect_cf_instr = 1'b1;
      end
      // compressed c.jr / c.jalr family (also catches the same encoding family used earlier)
      else if (is_rvc &&
               (instr[15:13] == OpcodeC2JalrMvAdd) &&
               (instr[6:2]   == 5'b00000) &&
               (instr[1:0]   == OpcodeC2)) begin
        is_indirect_cf_instr = 1'b1;
      end
      else begin
        is_indirect_cf_instr = 1'b0;
      end
    end
  endfunction

  state_t state_q, state_d;
  trace_data_t trace_q, trace_d;
  logic [CHUNK_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;
  logic [BR_CNT_W-1:0]    br_cnt_q, br_cnt_d;
  logic [START_CNT_W-1:0] instr_start_cnt_q, instr_start_cnt_d;
  logic [PC_WIDTH-1:0]    last_branch_target_q, last_branch_target_d;
  logic [PC_WIDTH-1:0]    last_instr_pc_q, last_instr_pc_d;
  logic                   last_instr_compressed_q, last_instr_compressed_d;
  logic                   last_instr_was_taken_q, last_instr_was_taken_d;
  logic                   has_indirect_cf_q, has_indirect_cf_d;

  logic [TRACE_ADDRW-1:0] sram_wr_addr_q, sram_wr_addr_d;
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q,  commit_data_d;
  logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] branch_pcs_q, branch_pcs_d;
  logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] commit_branch_pcs_q, commit_branch_pcs_d;
  logic [GHR_WIDTH-1:0]   trace_start_ghr_q, trace_start_ghr_d;
  logic [CHUNK_PTR_W-1:0] commit_chunk_ptr_q, commit_chunk_ptr_d;
  logic [TAKEN_CNT_W-1:0] taken_cnt_q, taken_cnt_d;

  // Per-set duplicate filter: remembers the last committed (pc, flags) per set.
  logic                        dup_valid [(1 << TRACE_ADDRW)];
  logic [PC_WIDTH-1:0]         dup_pc    [(1 << TRACE_ADDRW)];
  logic [CHUNKS_PER_TRACE-1:0] dup_flags [(1 << TRACE_ADDRW)];
  logic [BR_CNT_WIDTH-1:0]     dup_num_branches [(1 << TRACE_ADDRW)];

  logic                        stats_window_has_taken;
  logic                        stats_finalize_attempt_d;
  logic                        stats_finalize_duplicate_d;
  logic                        stats_finalize_indirect_d;
  logic                        stats_finalize_reason_full_d;
  logic                        stats_finalize_reason_max_instr_d;
  logic                        stats_finalize_reason_max_taken_d;
  logic [START_CNT_W-1:0]      stats_finalize_instrs_d;
  logic [BR_CNT_W-1:0]         stats_finalize_branches_d;
  logic [TAKEN_CNT_W-1:0]      stats_finalize_taken_d;

`ifndef SYNTHESIS
// pragma translate_off
  int unsigned tc_stat_windows_total;
  int unsigned tc_stat_windows_aligned;
  int unsigned tc_stat_windows_unaligned;
  int unsigned tc_stat_windows_unaligned_with_taken;
  int unsigned tc_stat_windows_with_taken;
  int unsigned tc_stat_trace_start_windows;
  int unsigned tc_stat_trace_finalize_attempts;
  int unsigned tc_stat_trace_commits;
  int unsigned tc_stat_trace_dup_drops;
  int unsigned tc_stat_trace_indirect_drops;
  int unsigned tc_stat_trace_end_full;
  int unsigned tc_stat_trace_end_max_instr;
  int unsigned tc_stat_trace_end_max_taken;
  int unsigned tc_stat_attempt_len_hist   [0:TRACE_LEN];
  int unsigned tc_stat_attempt_taken_hist [0:MAX_TAKEN];
  int unsigned tc_stat_commit_len_hist    [0:TRACE_LEN];
  int unsigned tc_stat_commit_taken_hist  [0:MAX_TAKEN];
  int unsigned tc_stat_commit_branch_hist [0:CHUNKS_PER_TRACE];
// pragma translate_on
`endif

  assign instr_i.ready      = 1'b1;
  assign mem_req_o          = commit_valid_q;
  assign mem_we_o           = commit_valid_q;
  assign mem_addr_o         = sram_wr_addr_q;
  assign mem_wdata_o        = commit_data_q;
  assign mem_be_o           = {BE_WIDTH{1'b1}};
  assign mem_branch_pcs_o   = commit_branch_pcs_q;
  assign trace_valid_o      = commit_valid_q;
  assign trace_data_o       = commit_data_q;

  always_comb begin
    stats_window_has_taken = 1'b0;
    for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
      if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i])
        stats_window_has_taken = 1'b1;
    end
  end

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
      has_indirect_cf_q       <= 1'b0;
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
      has_indirect_cf_q       <= has_indirect_cf_d;
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
    logic [TAKEN_CNT_W-1:0] temp_taken_cnt;
    logic                   temp_has_indirect_cf;
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
    has_indirect_cf_d       = has_indirect_cf_q;
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
    temp_taken_cnt          = '0;
    temp_has_indirect_cf    = has_indirect_cf_q;
    is_compressed           = 1'b0;
    has_space               = 1'b0;
    found_taken             = 1'b0;
    branch_slot             = '0;
    hit_taken               = 1'b0;
    trace_full              = 1'b0;
    stats_finalize_attempt_d          = 1'b0;
    stats_finalize_duplicate_d        = 1'b0;
    stats_finalize_indirect_d         = 1'b0;
    stats_finalize_reason_full_d      = 1'b0;
    stats_finalize_reason_max_instr_d = 1'b0;
    stats_finalize_reason_max_taken_d = 1'b0;
    stats_finalize_instrs_d           = '0;
    stats_finalize_branches_d         = '0;
    stats_finalize_taken_d            = '0;

    if (flush_i) begin
      state_d                 = IDLE;
      chunk_ptr_d             = '0;
      br_cnt_d                = '0;
      instr_start_cnt_d       = '0;
      trace_d                 = '0;
      last_instr_pc_d         = '0;
      last_instr_compressed_d = 1'b0;
      last_instr_was_taken_d  = 1'b0;
      has_indirect_cf_d       = 1'b0;
      taken_cnt_d             = '0;
    end else begin
      case (state_q)

        IDLE: begin
          // Unaligned windows are currently unsafe for TC tags/data because the 64-bit
          // instr_realign path still has known-bad address/instruction reconstruction.
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
              temp_taken_cnt          = '0;
              temp_has_indirect_cf    = 1'b0;
              chunk_ptr_d             = '0;
              br_cnt_d                = '0;
              instr_start_cnt_d       = '0;
              trace_d                 = '0;
              last_instr_pc_d         = '0;
              last_instr_compressed_d = 1'b0;
              last_instr_was_taken_d  = 1'b0;
              has_indirect_cf_d       = 1'b0;

              for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
                if (instr_i.valid[i] && (CHUNK_PTR_W'(i) <= branch_slot) &&
                    (temp_start_cnt < START_CNT_W'(MAX_INSTR_PER_TRACE))) begin

                  if (temp_start_cnt == 0)
                    trace_d.base_pc = instr_i.pc[i];

                  if (is_indirect_cf_instr(instr_i.inst[i]))
                    temp_has_indirect_cf = 1'b1;

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

                    if (instr_i.taken[i]) begin
                      last_branch_target_d = instr_i.target[i];
                      if (temp_taken_cnt < TAKEN_CNT_W'(MAX_TAKEN)) begin
                        trace_d.taken_targets[temp_taken_cnt] = instr_i.target[i];
                        temp_taken_cnt = temp_taken_cnt + TAKEN_CNT_W'(1);
                      end
                    end

                    temp_br_cnt = temp_br_cnt + 1;
                  end
                end
              end

              trace_d.lookup_num_branches = BR_CNT_W'(temp_br_cnt);
              trace_d.num_branches        = BR_CNT_W'(temp_br_cnt);
              trace_d.num_taken           = temp_taken_cnt;
              chunk_ptr_d                 = temp_chunk_ptr;
              br_cnt_d                    = temp_br_cnt;
              instr_start_cnt_d           = temp_start_cnt;
              taken_cnt_d                 = temp_taken_cnt;
              has_indirect_cf_d           = temp_has_indirect_cf;
              state_d                     = ACCUM;
            end
          end
        end

        ACCUM: begin
          if (|instr_i.consumed) begin
            temp_chunk_ptr       = chunk_ptr_q;
            temp_br_cnt          = br_cnt_q;
            temp_start_cnt       = instr_start_cnt_q;
            temp_taken_cnt       = taken_cnt_q;
            temp_has_indirect_cf = has_indirect_cf_q;
            hit_taken            = 1'b0;
            trace_full           = 1'b0;

            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              is_compressed = (instr_i.inst[i][1:0] != 2'b11);
              has_space = is_compressed ?
                          (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                          (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

              if (instr_i.valid[i] && has_space && !hit_taken &&
                  (temp_start_cnt < START_CNT_W'(MAX_INSTR_PER_TRACE))) begin

                if (is_indirect_cf_instr(instr_i.inst[i]))
                  temp_has_indirect_cf = 1'b1;

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

                    if (temp_taken_cnt < TAKEN_CNT_W'(MAX_TAKEN)) begin
                      trace_d.taken_targets[temp_taken_cnt] = instr_i.target[i];
                      temp_taken_cnt = temp_taken_cnt + TAKEN_CNT_W'(1);
                    end

                    hit_taken = 1'b1;
                  end

                  temp_br_cnt = temp_br_cnt + 1;
                end
              end else if (instr_i.valid[i] &&
                           (!has_space || (temp_start_cnt >= START_CNT_W'(MAX_INSTR_PER_TRACE)))) begin
                trace_full = 1'b1;
              end
            end

            chunk_ptr_d             = temp_chunk_ptr;
            br_cnt_d                = temp_br_cnt;
            instr_start_cnt_d       = temp_start_cnt;
            taken_cnt_d             = temp_taken_cnt;
            has_indirect_cf_d       = temp_has_indirect_cf;
            trace_d.num_branches    = BR_CNT_W'(temp_br_cnt);
            trace_d.num_taken       = temp_taken_cnt;

            if (trace_full ||
                (temp_start_cnt >= START_CNT_W'(MAX_INSTR_PER_TRACE) && temp_chunk_ptr > 0) ||
                (temp_taken_cnt >= TAKEN_CNT_W'(MAX_TAKEN) && temp_chunk_ptr > 0)) begin
              logic [TRACE_ADDRW-1:0] candidate_addr;
              logic                   is_duplicate;

              commit_chunk_ptr_d  = temp_chunk_ptr;
              trace_d.valid       = 1'b1;

              trace_d.target_addr = last_instr_was_taken_d
                                    ? last_branch_target_d
                                    : last_instr_pc_d + (last_instr_compressed_d ? 64'h2 : 64'h4);

              candidate_addr = tc_index(trace_d.base_pc, trace_d.lookup_branch_flags);

              is_duplicate = dup_valid[candidate_addr]
                           && (trace_d.base_pc             == dup_pc[candidate_addr])
                           && (trace_d.lookup_branch_flags == dup_flags[candidate_addr])
                           && (trace_d.lookup_num_branches == dup_num_branches[candidate_addr]);

              stats_finalize_attempt_d          = 1'b1;
              stats_finalize_duplicate_d        = is_duplicate;
              stats_finalize_indirect_d         = temp_has_indirect_cf;
              stats_finalize_reason_full_d      = trace_full;
              stats_finalize_reason_max_instr_d = (temp_start_cnt >= START_CNT_W'(MAX_INSTR_PER_TRACE)) && (temp_chunk_ptr > 0);
              stats_finalize_reason_max_taken_d = (temp_taken_cnt >= TAKEN_CNT_W'(MAX_TAKEN)) && (temp_chunk_ptr > 0);
              stats_finalize_instrs_d           = temp_start_cnt;
              stats_finalize_branches_d         = BR_CNT_W'(temp_br_cnt);
              stats_finalize_taken_d            = temp_taken_cnt;

              // Drop any trace containing indirect control flow.
              if (!is_duplicate && !temp_has_indirect_cf) begin
                sram_wr_addr_d      = candidate_addr;
                commit_valid_d      = 1'b1;
                commit_data_d       = trace_d;
                commit_branch_pcs_d = branch_pcs_d;
              end

              state_d                 = IDLE;
              chunk_ptr_d             = '0;
              br_cnt_d                = '0;
              instr_start_cnt_d       = '0;
              taken_cnt_d             = '0;
              has_indirect_cf_d       = 1'b0;
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
// pragma translate_off
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_stat_windows_total                <= 0;
      tc_stat_windows_aligned              <= 0;
      tc_stat_windows_unaligned            <= 0;
      tc_stat_windows_unaligned_with_taken <= 0;
      tc_stat_windows_with_taken           <= 0;
      tc_stat_trace_start_windows          <= 0;
      tc_stat_trace_finalize_attempts      <= 0;
      tc_stat_trace_commits                <= 0;
      tc_stat_trace_dup_drops              <= 0;
      tc_stat_trace_indirect_drops         <= 0;
      tc_stat_trace_end_full               <= 0;
      tc_stat_trace_end_max_instr          <= 0;
      tc_stat_trace_end_max_taken          <= 0;
      for (int i = 0; i <= TRACE_LEN; i++) begin
        tc_stat_attempt_len_hist[i] <= 0;
        tc_stat_commit_len_hist[i]  <= 0;
      end
      for (int i = 0; i <= MAX_TAKEN; i++) begin
        tc_stat_attempt_taken_hist[i] <= 0;
        tc_stat_commit_taken_hist[i]  <= 0;
      end
      for (int i = 0; i <= CHUNKS_PER_TRACE; i++)
        tc_stat_commit_branch_hist[i] <= 0;
    end else begin
      if (|instr_i.consumed) begin
        tc_stat_windows_total <= tc_stat_windows_total + 1;
        if (instr_i.serving_unaligned) begin
          tc_stat_windows_unaligned <= tc_stat_windows_unaligned + 1;
          if (stats_window_has_taken)
            tc_stat_windows_unaligned_with_taken <= tc_stat_windows_unaligned_with_taken + 1;
        end else begin
          tc_stat_windows_aligned <= tc_stat_windows_aligned + 1;
          if (stats_window_has_taken)
            tc_stat_windows_with_taken <= tc_stat_windows_with_taken + 1;
          if ((state_q == IDLE) && stats_window_has_taken)
            tc_stat_trace_start_windows <= tc_stat_trace_start_windows + 1;
        end
      end

      if (stats_finalize_attempt_d) begin
        tc_stat_trace_finalize_attempts <= tc_stat_trace_finalize_attempts + 1;
        tc_stat_attempt_len_hist[int'(stats_finalize_instrs_d)] <=
          tc_stat_attempt_len_hist[int'(stats_finalize_instrs_d)] + 1;
        tc_stat_attempt_taken_hist[int'(stats_finalize_taken_d)] <=
          tc_stat_attempt_taken_hist[int'(stats_finalize_taken_d)] + 1;

        if (stats_finalize_reason_full_d)
          tc_stat_trace_end_full <= tc_stat_trace_end_full + 1;
        if (stats_finalize_reason_max_instr_d)
          tc_stat_trace_end_max_instr <= tc_stat_trace_end_max_instr + 1;
        if (stats_finalize_reason_max_taken_d)
          tc_stat_trace_end_max_taken <= tc_stat_trace_end_max_taken + 1;

        if (commit_valid_d) begin
          tc_stat_trace_commits <= tc_stat_trace_commits + 1;
          tc_stat_commit_len_hist[int'(stats_finalize_instrs_d)] <=
            tc_stat_commit_len_hist[int'(stats_finalize_instrs_d)] + 1;
          tc_stat_commit_taken_hist[int'(stats_finalize_taken_d)] <=
            tc_stat_commit_taken_hist[int'(stats_finalize_taken_d)] + 1;
          tc_stat_commit_branch_hist[int'(stats_finalize_branches_d)] <=
            tc_stat_commit_branch_hist[int'(stats_finalize_branches_d)] + 1;
        end else if (stats_finalize_duplicate_d) begin
          tc_stat_trace_dup_drops <= tc_stat_trace_dup_drops + 1;
        end else if (stats_finalize_indirect_d) begin
          tc_stat_trace_indirect_drops <= tc_stat_trace_indirect_drops + 1;
        end
      end
    end
  end

  final begin
    int unsigned commit_br_ge4;
    commit_br_ge4 = 0;
    for (int b = 4; b <= CHUNKS_PER_TRACE; b++)
      commit_br_ge4 += tc_stat_commit_branch_hist[b];

    $display("[TC-BUILD] ===== Trace builder summary =====");
    $display("[TC-BUILD] windows total=%0d aligned=%0d unaligned=%0d unaligned_with_taken=%0d",
             tc_stat_windows_total, tc_stat_windows_aligned, tc_stat_windows_unaligned,
             tc_stat_windows_unaligned_with_taken);
    $display("[TC-BUILD] windows_with_taken=%0d trace_start_windows=%0d",
             tc_stat_windows_with_taken, tc_stat_trace_start_windows);
    $display("[TC-BUILD] finalize_attempts=%0d committed=%0d duplicate_dropped=%0d indirect_dropped=%0d",
             tc_stat_trace_finalize_attempts, tc_stat_trace_commits,
             tc_stat_trace_dup_drops, tc_stat_trace_indirect_drops);
    $display("[TC-BUILD] end_reasons: full=%0d max_instr=%0d max_taken=%0d",
             tc_stat_trace_end_full, tc_stat_trace_end_max_instr, tc_stat_trace_end_max_taken);
    $display("[TC-BUILD] attempted_len_hist: L1=%0d L2=%0d L3=%0d L4=%0d",
             tc_stat_attempt_len_hist[1], tc_stat_attempt_len_hist[2],
             tc_stat_attempt_len_hist[3], tc_stat_attempt_len_hist[4]);
    $display("[TC-BUILD] attempted_taken_hist: T0=%0d T1=%0d T2=%0d T3=%0d T4=%0d",
             tc_stat_attempt_taken_hist[0], tc_stat_attempt_taken_hist[1],
             tc_stat_attempt_taken_hist[2], tc_stat_attempt_taken_hist[3],
             tc_stat_attempt_taken_hist[4]);
    $display("[TC-BUILD] committed_len_hist: L1=%0d L2=%0d L3=%0d L4=%0d",
             tc_stat_commit_len_hist[1], tc_stat_commit_len_hist[2],
             tc_stat_commit_len_hist[3], tc_stat_commit_len_hist[4]);
    $display("[TC-BUILD] committed_taken_hist: T0=%0d T1=%0d T2=%0d T3=%0d T4=%0d",
             tc_stat_commit_taken_hist[0], tc_stat_commit_taken_hist[1],
             tc_stat_commit_taken_hist[2], tc_stat_commit_taken_hist[3],
             tc_stat_commit_taken_hist[4]);
    $display("[TC-BUILD] committed_branch_hist: B0=%0d B1=%0d B2=%0d B3=%0d B4plus=%0d",
             tc_stat_commit_branch_hist[0], tc_stat_commit_branch_hist[1],
             tc_stat_commit_branch_hist[2], tc_stat_commit_branch_hist[3], commit_br_ge4);
    $display("[TC-BUILD] =================================");
  end
// pragma translate_on

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
      $display("[TC-BUILDER]   #taken     = %0d", dbg.num_taken);
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