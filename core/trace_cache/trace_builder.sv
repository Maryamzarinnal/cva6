`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// Builds a suffix-only trace:
// - Tag = trigger fetch-window identity (base PC + branch pattern in that window)
// - Payload = straight-line instructions after the taken branch target
// - The trigger branch/window itself is never stored in the payload
// - Recording stops before the next control-flow, on unaligned trouble, or at MAX_INSTR_PER_TRACE

module trace_builder #(
  parameter int unsigned MAX_INSTR_PER_TRACE = MAX_TRACE_INSTR
) (
    input  logic clk_i,
    input  logic rst_ni,

    tracebuilder_instr_if.consumer instr_i,

    input  logic [GHR_WIDTH-1:0] ghr_i,
    input  logic                 flush_i,

    output logic                   trace_valid_o,
    output logic [TRACE_WIDTH-1:0] trace_data_o,
    output trace_tag_t             mem_tag_o,

    output logic                   mem_req_o,
    output logic                   mem_we_o,
    output logic [TRACE_ADDRW-1:0] mem_addr_o,
    output logic [TRACE_WIDTH-1:0] mem_wdata_o,
    output logic [BE_WIDTH-1:0]    mem_be_o,
    output logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] mem_branch_pcs_o
);

  localparam int unsigned CHUNK_PTR_W = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned START_CNT_W = $clog2(TRACE_LEN + 1);

  typedef enum logic {
    IDLE,
    ACCUM
  } state_t;

  function automatic logic is_compressed_instr(input logic [INSTR_WIDTH-1:0] instr);
    is_compressed_instr = (instr[1:0] != 2'b11);
  endfunction

  state_t state_q, state_d;

  trace_data_t active_payload_q, active_payload_d;
  trace_tag_t  active_tag_q, active_tag_d;
  logic [CHUNK_PTR_W-1:0] active_chunk_ptr_q, active_chunk_ptr_d;
  logic [START_CNT_W-1:0] active_instr_cnt_q, active_instr_cnt_d;
  logic [PC_WIDTH-1:0]    expected_pc_q, expected_pc_d;
  logic                   has_suffix_instr_q, has_suffix_instr_d;

  logic [TRACE_ADDRW-1:0] commit_addr_q, commit_addr_d;
  trace_tag_t             commit_tag_q, commit_tag_d;
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q, commit_data_d;

`ifndef SYNTHESIS
  logic dbg_evt_start_any;
  logic dbg_evt_start_single_taken;
  logic dbg_evt_start_multi_taken;
  logic dbg_evt_start_unaligned_taken;
  logic dbg_evt_accum_pc_gap;
  logic dbg_evt_finalize_any;
  logic dbg_evt_finalize_unaligned;
  logic dbg_evt_finalize_stop_cf;
  logic dbg_evt_finalize_trace_full;
  logic dbg_evt_finalize_len_limit;
  logic dbg_evt_finalize_taken_limit;
  logic dbg_evt_outcome_commit;
  logic dbg_evt_outcome_drop_duplicate;
  logic dbg_evt_outcome_drop_indirect;
  logic dbg_evt_outcome_drop_not_single;
  logic dbg_evt_outcome_drop_no_suffix;

  longint unsigned dbg_start_any_q;
  longint unsigned dbg_start_single_taken_q;
  longint unsigned dbg_start_multi_taken_q;
  longint unsigned dbg_start_unaligned_taken_q;
  longint unsigned dbg_accum_pc_gap_q;
  longint unsigned dbg_finalize_any_q;
  longint unsigned dbg_finalize_unaligned_q;
  longint unsigned dbg_finalize_stop_cf_q;
  longint unsigned dbg_finalize_trace_full_q;
  longint unsigned dbg_finalize_len_limit_q;
  longint unsigned dbg_finalize_taken_limit_q;
  longint unsigned dbg_outcome_commit_q;
  longint unsigned dbg_outcome_drop_duplicate_q;
  longint unsigned dbg_outcome_drop_indirect_q;
  longint unsigned dbg_outcome_drop_not_single_q;
  longint unsigned dbg_outcome_drop_no_suffix_q;
  longint unsigned dbg_payload_len0_q;
  longint unsigned dbg_payload_len1_q;
  longint unsigned dbg_payload_len2_q;
  longint unsigned dbg_payload_len3_q;
  longint unsigned dbg_payload_len4p_q;
`endif

  // Per-set duplicate filter: remembers the last committed trigger tag per set.
  logic                        dup_valid [(1 << TRACE_ADDRW)];
  logic [PC_WIDTH-1:0]         dup_pc    [(1 << TRACE_ADDRW)];
  logic [CHUNKS_PER_TRACE-1:0] dup_flags [(1 << TRACE_ADDRW)];
  logic [BR_CNT_WIDTH-1:0]     dup_num_branches [(1 << TRACE_ADDRW)];

  assign instr_i.ready    = 1'b1;
  assign mem_req_o        = commit_valid_q;
  assign mem_we_o         = commit_valid_q;
  assign mem_addr_o       = commit_addr_q;
  assign mem_tag_o        = commit_tag_q;
  assign mem_wdata_o      = commit_data_q;
  assign mem_be_o         = {BE_WIDTH{1'b1}};
  assign mem_branch_pcs_o = '0;
  assign trace_valid_o    = commit_valid_q;
  assign trace_data_o     = commit_data_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q             <= IDLE;
      active_payload_q    <= '0;
      active_tag_q        <= '0;
      active_chunk_ptr_q  <= '0;
      active_instr_cnt_q  <= '0;
      expected_pc_q       <= '0;
      has_suffix_instr_q  <= 1'b0;
      commit_addr_q       <= '0;
      commit_tag_q        <= '0;
      commit_valid_q      <= 1'b0;
      commit_data_q       <= '0;
    end else begin
      state_q             <= state_d;
      active_payload_q    <= active_payload_d;
      active_tag_q        <= active_tag_d;
      active_chunk_ptr_q  <= active_chunk_ptr_d;
      active_instr_cnt_q  <= active_instr_cnt_d;
      expected_pc_q       <= expected_pc_d;
      has_suffix_instr_q  <= has_suffix_instr_d;
      commit_addr_q       <= commit_addr_d;
      commit_tag_q        <= commit_tag_d;
      commit_valid_q      <= commit_valid_d;
      commit_data_q       <= commit_data_d;
    end
  end

  always_comb begin
    logic [PC_WIDTH-1:0] trigger_base_pc;
    logic [PC_WIDTH-1:0] trigger_target_pc;
    logic [TRIGGER_BRANCH_BITS-1:0] trigger_flags;
    logic trigger_base_pc_valid;
    int unsigned trigger_cf_cnt;
    int unsigned trigger_taken_cnt;
    logic [CHUNKS_PER_TRACE-1:0] candidate_flags;
    logic [TRACE_ADDRW-1:0] candidate_addr;
    logic is_duplicate;
    logic stop_suffix_now;
    logic finalize_now;
    logic finalize_len_limit;
    logic any_consumed_now;

    state_d            = state_q;
    active_payload_d   = active_payload_q;
    active_tag_d       = active_tag_q;
    active_chunk_ptr_d = active_chunk_ptr_q;
    active_instr_cnt_d = active_instr_cnt_q;
    expected_pc_d      = expected_pc_q;
    has_suffix_instr_d = has_suffix_instr_q;

    commit_addr_d      = commit_addr_q;
    commit_tag_d       = commit_tag_q;
    commit_valid_d     = 1'b0;
    commit_data_d      = commit_data_q;

    trigger_base_pc       = '0;
    trigger_target_pc     = '0;
    trigger_flags         = '0;
    trigger_base_pc_valid = 1'b0;
    trigger_cf_cnt        = 0;
    trigger_taken_cnt     = 0;
    candidate_flags       = '0;
    candidate_addr        = '0;
    is_duplicate          = 1'b0;
    stop_suffix_now       = 1'b0;
    finalize_now          = 1'b0;
    finalize_len_limit    = 1'b0;
    any_consumed_now      = 1'b0;

`ifndef SYNTHESIS
    dbg_evt_start_any               = 1'b0;
    dbg_evt_start_single_taken      = 1'b0;
    dbg_evt_start_multi_taken       = 1'b0;
    dbg_evt_start_unaligned_taken   = 1'b0;
    dbg_evt_accum_pc_gap            = 1'b0;
    dbg_evt_finalize_any            = 1'b0;
    dbg_evt_finalize_unaligned      = 1'b0;
    dbg_evt_finalize_stop_cf        = 1'b0;
    dbg_evt_finalize_trace_full     = 1'b0;
    dbg_evt_finalize_len_limit      = 1'b0;
    dbg_evt_finalize_taken_limit    = 1'b0;
    dbg_evt_outcome_commit          = 1'b0;
    dbg_evt_outcome_drop_duplicate  = 1'b0;
    dbg_evt_outcome_drop_indirect   = 1'b0;
    dbg_evt_outcome_drop_not_single = 1'b0;
    dbg_evt_outcome_drop_no_suffix  = 1'b0;
`endif

    if (flush_i) begin
      state_d            = IDLE;
      active_payload_d   = '0;
      active_tag_d       = '0;
      active_chunk_ptr_d = '0;
      active_instr_cnt_d = '0;
      expected_pc_d      = '0;
      has_suffix_instr_d = 1'b0;
    end else begin
      case (state_q)
        IDLE: begin
          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.consumed[i] && instr_i.valid[i]) begin
              any_consumed_now = 1'b1;
              if (!trigger_base_pc_valid) begin
                trigger_base_pc = instr_i.pc[i];
                trigger_base_pc_valid = 1'b1;
              end

              if (instr_i.is_branch[i]) begin
                if (trigger_cf_cnt < TRIGGER_BRANCH_BITS)
                  trigger_flags[trigger_cf_cnt] = instr_i.taken[i];
                trigger_cf_cnt++;
              end

              if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                trigger_taken_cnt++;
                if (trigger_taken_cnt == 1)
                  trigger_target_pc = instr_i.target[i];
              end
            end
          end

          if (any_consumed_now && instr_i.serving_unaligned) begin
`ifndef SYNTHESIS
            if (trigger_taken_cnt != 0)
              dbg_evt_start_unaligned_taken = 1'b1;
`endif
          end else if (trigger_taken_cnt != 0) begin
`ifndef SYNTHESIS
            dbg_evt_start_any = 1'b1;
            if (trigger_taken_cnt == 1)
              dbg_evt_start_single_taken = 1'b1;
            else
              dbg_evt_start_multi_taken = 1'b1;
`endif
            if (trigger_taken_cnt == 1 && trigger_base_pc_valid) begin
              active_payload_d                 = '0;
              active_payload_d.base_pc         = trigger_target_pc;
              active_payload_d.target_addr     = trigger_target_pc;
              active_tag_d                     = make_trace_tag(
                                                  trigger_base_pc,
                                                  TRIGGER_BRANCH_CNT_WIDTH'(trigger_cf_cnt),
                                                  trigger_flags
                                                );
              active_chunk_ptr_d               = '0;
              active_instr_cnt_d               = '0;
              expected_pc_d                    = trigger_target_pc;
              has_suffix_instr_d               = 1'b0;
              state_d                          = ACCUM;
            end
          end
        end

        ACCUM: begin
          if (|instr_i.consumed && instr_i.serving_unaligned) begin
            finalize_now = 1'b1;
`ifndef SYNTHESIS
            dbg_evt_finalize_any        = 1'b1;
            dbg_evt_finalize_unaligned  = 1'b1;
            dbg_evt_finalize_trace_full = 1'b1;
`endif
          end else if (|instr_i.consumed) begin
            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              logic is_rvc;
              logic [PC_WIDTH-1:0] next_pc_calc;

              if (!(instr_i.consumed[i] && instr_i.valid[i]))
                continue;

              if (instr_i.pc[i] != expected_pc_q) begin
`ifndef SYNTHESIS
                dbg_evt_accum_pc_gap = 1'b1;
`endif
                continue;
              end

              if (instr_i.is_branch[i]) begin
                stop_suffix_now = 1'b1;
                finalize_now    = 1'b1;
`ifndef SYNTHESIS
                dbg_evt_finalize_any        = 1'b1;
                dbg_evt_finalize_stop_cf    = 1'b1;
                dbg_evt_finalize_trace_full = 1'b1;
`endif
                break;
              end

              is_rvc = is_compressed_instr(instr_i.inst[i]);
              if ((!is_rvc && (active_chunk_ptr_q + 2 > CHUNKS_PER_TRACE)) ||
                  ( is_rvc && (active_chunk_ptr_q + 1 > CHUNKS_PER_TRACE))) begin
                finalize_now = 1'b1;
`ifndef SYNTHESIS
                dbg_evt_finalize_any        = 1'b1;
                dbg_evt_finalize_trace_full = 1'b1;
`endif
                break;
              end

              if (active_instr_cnt_q >= START_CNT_W'(MAX_INSTR_PER_TRACE)) begin
                finalize_now = 1'b1;
                finalize_len_limit = 1'b1;
`ifndef SYNTHESIS
                dbg_evt_finalize_any       = 1'b1;
                dbg_evt_finalize_len_limit = 1'b1;
`endif
                break;
              end

              active_payload_d.valid_chunks[active_chunk_ptr_q] = 1'b1;
              active_payload_d.chunks[active_chunk_ptr_q]       = instr_i.inst[i][15:0];
              if (is_rvc) begin
                active_chunk_ptr_d = active_chunk_ptr_q + CHUNK_PTR_W'(1);
              end else begin
                active_payload_d.valid_chunks[active_chunk_ptr_q + 1] = 1'b0;
                active_payload_d.chunks[active_chunk_ptr_q + 1]       = instr_i.inst[i][31:16];
                active_chunk_ptr_d = active_chunk_ptr_q + CHUNK_PTR_W'(2);
              end

              active_instr_cnt_d  = active_instr_cnt_q + START_CNT_W'(1);
              next_pc_calc        = instr_i.pc[i] + (is_rvc ? PC_WIDTH'(64'd2) : PC_WIDTH'(64'd4));
              expected_pc_d       = next_pc_calc;
              active_payload_d.target_addr = next_pc_calc;
              has_suffix_instr_d  = 1'b1;

              if (active_instr_cnt_d >= START_CNT_W'(MAX_INSTR_PER_TRACE)) begin
                finalize_now = 1'b1;
                finalize_len_limit = 1'b1;
`ifndef SYNTHESIS
                dbg_evt_finalize_any       = 1'b1;
                dbg_evt_finalize_len_limit = 1'b1;
`endif
                break;
              end
            end
          end

          if (finalize_now) begin
            candidate_flags[TRIGGER_BRANCH_BITS-1:0] = active_tag_q.branch_flags;
            candidate_addr = tc_index(active_tag_q.base_pc, candidate_flags);
            is_duplicate = dup_valid[candidate_addr]
                         && (active_tag_q.base_pc      == dup_pc[candidate_addr])
                         && (candidate_flags           == dup_flags[candidate_addr])
                         && (BR_CNT_WIDTH'(active_tag_q.num_branches) == dup_num_branches[candidate_addr]);

            if (has_suffix_instr_q || has_suffix_instr_d) begin
              active_payload_d.valid        = 1'b1;
              active_payload_d.lookup_branch_flags = '0;
              active_payload_d.lookup_num_branches = '0;
              active_payload_d.branch_flags = '0;
              active_payload_d.num_branches = '0;
              active_payload_d.num_taken    = '0;
              active_payload_d.taken_targets = '0;

              if (!is_duplicate) begin
                commit_addr_d  = candidate_addr;
                commit_tag_d   = active_tag_q;
                commit_valid_d = 1'b1;
                commit_data_d  = active_payload_d;
`ifndef SYNTHESIS
                dbg_evt_outcome_commit = 1'b1;
`endif
              end else begin
`ifndef SYNTHESIS
                dbg_evt_outcome_drop_duplicate = 1'b1;
`endif
              end
            end else begin
`ifndef SYNTHESIS
              dbg_evt_outcome_drop_no_suffix = 1'b1;
`endif
            end

            state_d            = IDLE;
            active_payload_d   = '0;
            active_tag_d       = '0;
            active_chunk_ptr_d = '0;
            active_instr_cnt_d = '0;
            expected_pc_d      = '0;
            has_suffix_instr_d = 1'b0;
          end
        end
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < (1 << TRACE_ADDRW); i++) begin
        dup_valid[i] <= 1'b0;
        dup_pc[i] <= '0;
        dup_flags[i] <= '0;
        dup_num_branches[i] <= '0;
      end
    end else if (commit_valid_d) begin
      logic [CHUNKS_PER_TRACE-1:0] dup_flags_tmp;
      dup_flags_tmp = '0;
      dup_flags_tmp[TRIGGER_BRANCH_BITS-1:0] = commit_tag_d.branch_flags;
      dup_valid[commit_addr_d] <= 1'b1;
      dup_pc[commit_addr_d] <= commit_tag_d.base_pc;
      dup_flags[commit_addr_d] <= dup_flags_tmp;
      dup_num_branches[commit_addr_d] <= BR_CNT_WIDTH'(commit_tag_d.num_branches);
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dbg_start_any_q               <= 0;
      dbg_start_single_taken_q      <= 0;
      dbg_start_multi_taken_q       <= 0;
      dbg_start_unaligned_taken_q   <= 0;
      dbg_accum_pc_gap_q            <= 0;
      dbg_finalize_any_q            <= 0;
      dbg_finalize_unaligned_q      <= 0;
      dbg_finalize_stop_cf_q        <= 0;
      dbg_finalize_trace_full_q     <= 0;
      dbg_finalize_len_limit_q      <= 0;
      dbg_finalize_taken_limit_q    <= 0;
      dbg_outcome_commit_q          <= 0;
      dbg_outcome_drop_duplicate_q  <= 0;
      dbg_outcome_drop_indirect_q   <= 0;
      dbg_outcome_drop_not_single_q <= 0;
      dbg_outcome_drop_no_suffix_q  <= 0;
      dbg_payload_len0_q            <= 0;
      dbg_payload_len1_q            <= 0;
      dbg_payload_len2_q            <= 0;
      dbg_payload_len3_q            <= 0;
      dbg_payload_len4p_q           <= 0;
    end else begin
      if (dbg_evt_start_any)
        dbg_start_any_q <= dbg_start_any_q + 1;
      if (dbg_evt_start_single_taken)
        dbg_start_single_taken_q <= dbg_start_single_taken_q + 1;
      if (dbg_evt_start_multi_taken)
        dbg_start_multi_taken_q <= dbg_start_multi_taken_q + 1;
      if (dbg_evt_start_unaligned_taken)
        dbg_start_unaligned_taken_q <= dbg_start_unaligned_taken_q + 1;
      if (dbg_evt_accum_pc_gap)
        dbg_accum_pc_gap_q <= dbg_accum_pc_gap_q + 1;
      if (dbg_evt_finalize_any)
        dbg_finalize_any_q <= dbg_finalize_any_q + 1;
      if (dbg_evt_finalize_unaligned)
        dbg_finalize_unaligned_q <= dbg_finalize_unaligned_q + 1;
      if (dbg_evt_finalize_stop_cf)
        dbg_finalize_stop_cf_q <= dbg_finalize_stop_cf_q + 1;
      if (dbg_evt_finalize_trace_full)
        dbg_finalize_trace_full_q <= dbg_finalize_trace_full_q + 1;
      if (dbg_evt_finalize_len_limit)
        dbg_finalize_len_limit_q <= dbg_finalize_len_limit_q + 1;
      if (dbg_evt_finalize_taken_limit)
        dbg_finalize_taken_limit_q <= dbg_finalize_taken_limit_q + 1;
      if (dbg_evt_outcome_commit)
        dbg_outcome_commit_q <= dbg_outcome_commit_q + 1;
      if (dbg_evt_outcome_drop_duplicate)
        dbg_outcome_drop_duplicate_q <= dbg_outcome_drop_duplicate_q + 1;
      if (dbg_evt_outcome_drop_indirect)
        dbg_outcome_drop_indirect_q <= dbg_outcome_drop_indirect_q + 1;
      if (dbg_evt_outcome_drop_not_single)
        dbg_outcome_drop_not_single_q <= dbg_outcome_drop_not_single_q + 1;
      if (dbg_evt_outcome_drop_no_suffix)
        dbg_outcome_drop_no_suffix_q <= dbg_outcome_drop_no_suffix_q + 1;

      if (dbg_evt_finalize_any) begin
        unique case (int'(active_instr_cnt_d))
          0:       dbg_payload_len0_q  <= dbg_payload_len0_q + 1;
          1:       dbg_payload_len1_q  <= dbg_payload_len1_q + 1;
          2:       dbg_payload_len2_q  <= dbg_payload_len2_q + 1;
          3:       dbg_payload_len3_q  <= dbg_payload_len3_q + 1;
          default: dbg_payload_len4p_q <= dbg_payload_len4p_q + 1;
        endcase
      end
    end
  end

  final begin
    $display("[TC-BUILDER-DBG] starts: any=%0d single_taken=%0d multi_taken=%0d unaligned_taken=%0d",
             dbg_start_any_q, dbg_start_single_taken_q, dbg_start_multi_taken_q,
             dbg_start_unaligned_taken_q);
    $display("[TC-BUILDER-DBG] accum: pc_gap_windows=%0d finalizations=%0d stop_unaligned=%0d stop_cf=%0d cause_trace_full=%0d cause_len_limit=%0d cause_taken_limit=%0d",
             dbg_accum_pc_gap_q, dbg_finalize_any_q, dbg_finalize_unaligned_q,
             dbg_finalize_stop_cf_q, dbg_finalize_trace_full_q,
             dbg_finalize_len_limit_q, dbg_finalize_taken_limit_q);
    $display("[TC-BUILDER-DBG] outcomes: commit=%0d drop_duplicate=%0d drop_indirect=%0d drop_not_single=%0d drop_no_suffix=%0d",
             dbg_outcome_commit_q, dbg_outcome_drop_duplicate_q,
             dbg_outcome_drop_indirect_q, dbg_outcome_drop_not_single_q,
             dbg_outcome_drop_no_suffix_q);
    $display("[TC-BUILDER-DBG] payload_len_hist: len0=%0d len1=%0d len2=%0d len3=%0d len4p=%0d",
             dbg_payload_len0_q, dbg_payload_len1_q, dbg_payload_len2_q,
             dbg_payload_len3_q, dbg_payload_len4p_q);
  end
`endif

endmodule
