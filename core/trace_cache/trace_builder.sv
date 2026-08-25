`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// Multi-window trace builder.  Collects up to TRACE_LEN (4) instructions
// across up to MAX_TAKEN (3) taken branches, so one replay can cover several
// taken-branch redirects the frontend would otherwise pay for one at a time.
//
//   IDLE  taken direct branch in the window -> FILL, if there is room left
//   FILL  target window has not arrived yet    -> wait
//         another taken branch, still room     -> stay in FILL
//         otherwise                            -> commit
//
// Returns and direct calls are recorded.  The PC guard waits for the target
// window rather than discarding the trace; discarding lost 98.8% of them.

module trace_builder #(
  parameter int unsigned MAX_INSTR_PER_TRACE = MAX_TRACE_INSTR
) (
    input  logic clk_i,
    input  logic rst_ni,

    tracebuilder_instr_if.consumer instr_i,

    input  logic [GHR_WIDTH-1:0] ghr_i,
    input  logic                 flush_i,
    // (Option B): high the cycle a TC replay fires.  Forces the FSM
    // out of TB_FILL so the next icache window (which will be exit_pc,
    // not the previously expected branch target) starts a fresh trace
    // instead of being rejected forever by the PC guard.
    input  logic                 tc_replay_fired_i,

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
  localparam int unsigned INSTR_CNT_W = $clog2(MAX_INSTR_PER_TRACE + 1);
  // Minimum instruction count for a committed trace to survive replay-time
  // policy (tc_trace_policy_ok in frontend.sv).  Must equal TC_MIN_ACCEPT_LEN.
  localparam int unsigned TB_MIN_ACCEPT_LEN = 3;

  // Shortest single-taken trace the builder will store.  Multi-taken traces
  // ignore this and are always admitted.
  //
  // This was 4, on the reasoning that a length-3 single-taken replay breaks
  // even and so should not occupy a way.  That only holds if the ways are
  // contended, and they are not: 161 traces in 2048 entries, 9 evictions in
  // 13.2M cycles, and 99.84% of built traces thrown away by this filter.
  // With the ways empty, a break-even trace still beats nothing.  Matching
  // TB_MIN_ACCEPT_LEN also closes a mismatch where the frontend would happily
  // replay length-3 traces the builder refused to store.  Length 2 stays out.
  localparam int unsigned TB_VALUE_MIN_LEN = TB_MIN_ACCEPT_LEN;

  // -----------------------------------------------------------------------
  // State machine
  // -----------------------------------------------------------------------
  typedef enum logic {
    TB_IDLE,
    TB_FILL
  } tb_state_t;

  tb_state_t state_q, state_d;

  function automatic logic is_compressed(input logic [INSTR_WIDTH-1:0] inst);
    is_compressed = (inst[1:0] != 2'b11);
  endfunction

  // Direct CF: conditional branches + non-call unconditional JAL.
  function automatic logic is_direct_cf(input logic [INSTR_WIDTH-1:0] inst);
    logic rvc;
    begin
      rvc = is_compressed(inst);
      if (!rvc)
        is_direct_cf = (inst[6:0] == riscv::OpcodeBranch) ||
                       ((inst[6:0] == riscv::OpcodeJal) &&
                        (inst[11:7] != 5'd1) && (inst[11:7] != 5'd5));
      else
        is_direct_cf =
            ((inst[1:0] == riscv::OpcodeC1) &&
             ((inst[15:13] == riscv::OpcodeC1Beqz) ||
              (inst[15:13] == riscv::OpcodeC1Bnez) ||
              (inst[15:13] == riscv::OpcodeC1J)));
    end
  endfunction

  // Return CF ? jalr rd, rs1 where rs1 is a link register (x1/x5)
  // and rs1 != rd (standard RISC-V return convention).
  // Target comes from the frontend's RAS prediction.
  function automatic logic is_return_cf(input logic [INSTR_WIDTH-1:0] inst);
    logic rvc;
    begin
      rvc = is_compressed(inst);
      if (!rvc)
        // jalr rd, rs1, imm ? return when rs1 is link register and rs1 != rd
        is_return_cf = (inst[6:0] == riscv::OpcodeJalr) &&
                       ((inst[19:15] == 5'd1) || (inst[19:15] == 5'd5)) &&
                       (inst[19:15] != inst[11:7]);
      else
        // c.jr rs1 where rs1 = x1 or x5 (funct4=1000, rs2=0, [1:0]=10)
        // c.jr has rd=x0 implicitly, so rs1 != rd is always true for x1/x5
        is_return_cf = (inst[1:0] == riscv::OpcodeC2) &&
                       (inst[15:12] == 4'b1000) &&
                       (inst[6:2] == 5'b00000) &&
                       ((inst[11:7] == 5'd1) || (inst[11:7] == 5'd5));
    end
  endfunction

  // Direct call CF ? jal rd, imm where rd = x1(ra) or x5(t0).
  // Target is PC-relative (deterministic). RAS push needed during replay.
  function automatic logic is_call_cf(input logic [INSTR_WIDTH-1:0] inst);
    logic rvc;
    begin
      rvc = is_compressed(inst);
      if (!rvc)
        is_call_cf = (inst[6:0] == riscv::OpcodeJal) &&
                     ((inst[11:7] == 5'd1) || (inst[11:7] == 5'd5));
      else
        is_call_cf = 1'b0;  // No compressed direct call in RV64
    end
  endfunction

  // -----------------------------------------------------------------------
  // Accumulation registers (partial trace latched from the IDLE window)
  // -----------------------------------------------------------------------
  logic [PC_WIDTH-1:0]                     accum_base_pc_q;
  logic [TRIGGER_BRANCH_BITS-1:0]          accum_trig_flags_q;
  logic [TRIGGER_BRANCH_CNT_WIDTH-1:0]     accum_trig_cnt_q;
  logic [CHUNKS_PER_TRACE-1:0][15:0]       accum_chunks_q;
  logic [CHUNKS_PER_TRACE-1:0]             accum_valid_chunks_q;
  logic [CHUNKS_PER_TRACE-1:0]             accum_br_flags_q;
  logic [BR_CNT_WIDTH-1:0]                 accum_num_br_q;
  logic [PC_WIDTH-1:0]                     accum_taken_target_q;
  logic [CHUNK_PTR_W-1:0]                  accum_chunk_ptr_q;
  logic [INSTR_CNT_W-1:0]                  accum_instr_cnt_q;
  logic [GHR_WIDTH-1:0]                    accum_ghr_q;  // GHR snapshot at trace start
  // multi-taken accumulators
  logic [TAKEN_CNT_WIDTH-1:0]              accum_taken_cnt_q;
  logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]      accum_taken_targets_q;

  // Next-state wires (set by always_comb, consumed by always_ff).
  logic [PC_WIDTH-1:0]                     accum_base_pc_nxt;
  logic [TRIGGER_BRANCH_BITS-1:0]          accum_trig_flags_nxt;
  logic [TRIGGER_BRANCH_CNT_WIDTH-1:0]     accum_trig_cnt_nxt;
  logic [CHUNKS_PER_TRACE-1:0][15:0]       accum_chunks_nxt;
  logic [CHUNKS_PER_TRACE-1:0]             accum_valid_chunks_nxt;
  logic [CHUNKS_PER_TRACE-1:0]             accum_br_flags_nxt;
  logic [BR_CNT_WIDTH-1:0]                 accum_num_br_nxt;
  logic [PC_WIDTH-1:0]                     accum_taken_target_nxt;
  logic [CHUNK_PTR_W-1:0]                  accum_chunk_ptr_nxt;
  logic [INSTR_CNT_W-1:0]                  accum_instr_cnt_nxt;
  // multi-taken next-state wires
  logic [TAKEN_CNT_WIDTH-1:0]              accum_taken_cnt_nxt;
  logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]      accum_taken_targets_nxt;

  // FILL exit reason (set in always_comb, read in always_ff)
  logic [2:0] fill_exit_reason;
  // set in always_comb when a return stops the scan early,
  // read in always_ff for counters ? must be module-scope for cross-block access.
  logic        w_truncate_at_return;

  // latch-enable for accum registers ? only asserted when the
  // combinational block actually computes new accumulator values.
  // Without this, wait cycles in TB_FILL overwrite accum_*_q with
  // the default '0 (the original bug that zeroed accum_taken_target_q).
  logic accum_latch_en;

  // Set at every commit site: is this trace worth a way?  Multi-taken always
  // is -- it saves redirects the icache cannot.  Single-taken only if it is
  // at least TB_VALUE_MIN_LEN long, otherwise the replay does not cover its
  // own bubble.
  logic tb_commit_value_ok_d;

`ifndef SYNTHESIS
  // PC guard diagnostics (declared early for use in always_comb)
  longint unsigned dbg_pcguard_miss_total;
  longint unsigned dbg_pcguard_hit_total;
`endif

  // Commit output registers (1-cycle write to SRAM).
  logic                    commit_valid_q;
  logic [TRACE_ADDRW-1:0]  commit_addr_q;
  trace_tag_t              commit_tag_q;
  logic [TRACE_WIDTH-1:0]  commit_data_q;

  logic                    commit_valid_d;
  logic [TRACE_ADDRW-1:0]  commit_addr_d;
  trace_tag_t              commit_tag_d;
  logic [TRACE_WIDTH-1:0]  commit_data_d;

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

  // State + commit pipeline registers.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q        <= TB_IDLE;
      commit_valid_q <= 1'b0;
      commit_addr_q  <= '0;
      commit_tag_q   <= '0;
      commit_data_q  <= '0;
    end else begin
      state_q        <= state_d;
      // gate the commit on the value filter so dropped traces never
      // touch SRAM. commit_addr/tag/data still latch (cheap, single payload)
      // but with commit_valid_q=0 they are inert downstream.
      commit_valid_q <= commit_valid_d && tb_commit_value_ok_d;
      commit_addr_q  <= commit_addr_d;
      commit_tag_q   <= commit_tag_d;
      commit_data_q  <= commit_data_d;
    end
  end

  // Accumulation registers: latch on IDLE?FILL or FILL?FILL, clear on flush or return to IDLE.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      accum_base_pc_q      <= '0;
      accum_trig_flags_q   <= '0;
      accum_trig_cnt_q     <= '0;
      accum_chunks_q       <= '0;
      accum_valid_chunks_q <= '0;
      accum_br_flags_q     <= '0;
      accum_num_br_q       <= '0;
      accum_taken_target_q <= '0;
      accum_chunk_ptr_q    <= '0;
      accum_instr_cnt_q    <= '0;
      accum_ghr_q          <= '0;
      accum_taken_cnt_q    <= '0;
      accum_taken_targets_q <= '0;
    end else if (flush_i || (state_d == TB_IDLE && state_q != TB_IDLE)) begin
      accum_base_pc_q      <= '0;
      accum_trig_flags_q   <= '0;
      accum_trig_cnt_q     <= '0;
      accum_chunks_q       <= '0;
      accum_valid_chunks_q <= '0;
      accum_br_flags_q     <= '0;
      accum_num_br_q       <= '0;
      accum_taken_target_q <= '0;
      accum_chunk_ptr_q    <= '0;
      accum_instr_cnt_q    <= '0;
      accum_ghr_q          <= '0;
      accum_taken_cnt_q    <= '0;
      accum_taken_targets_q <= '0;
    end else if (state_d == TB_FILL && accum_latch_en) begin
      // Latch only when combo block produced new data
      // (IDLE?FILL or FILL?FILL loop), NOT during wait cycles.
      accum_base_pc_q      <= accum_base_pc_nxt;
      accum_trig_flags_q   <= accum_trig_flags_nxt;
      accum_trig_cnt_q     <= accum_trig_cnt_nxt;
      accum_chunks_q       <= accum_chunks_nxt;
      accum_valid_chunks_q <= accum_valid_chunks_nxt;
      accum_br_flags_q     <= accum_br_flags_nxt;
      accum_num_br_q       <= accum_num_br_nxt;
      accum_taken_target_q <= accum_taken_target_nxt;
      accum_chunk_ptr_q    <= accum_chunk_ptr_nxt;
      accum_instr_cnt_q    <= accum_instr_cnt_nxt;
      accum_taken_cnt_q    <= accum_taken_cnt_nxt;
      accum_taken_targets_q <= accum_taken_targets_nxt;
      if (state_q == TB_IDLE)
        accum_ghr_q <= ghr_i;  // capture GHR only at trace start
    end
  end

  // -----------------------------------------------------------------------
  // combinational scan: IDLE finds a taken branch ? FILL.
  // FILL scans target window: if another taken branch ? loop (FILL?FILL);
  // otherwise ? commit.  Up to MAX_TAKEN taken branches per trace.
  // -----------------------------------------------------------------------

  always_comb begin
    // Window scan variables
    logic [PC_WIDTH-1:0]                     w_base_pc;
    logic                                    w_has_base;
    logic [TRIGGER_BRANCH_BITS-1:0]          w_trig_flags;
    int unsigned                             w_trig_cnt;
    logic [CHUNKS_PER_TRACE-1:0][15:0]       w_chunks;
    logic [CHUNKS_PER_TRACE-1:0]             w_valid_chunks;
    logic [CHUNKS_PER_TRACE-1:0]             w_br_flags;
    logic [BR_CNT_WIDTH-1:0]                 w_num_br;
    logic [PC_WIDTH-1:0]                     w_taken_target;
    logic [PC_WIDTH-1:0]                     w_exit_pc;
    logic [CHUNK_PTR_W-1:0]                  w_cptr;
    logic [INSTR_CNT_W-1:0]                  w_icnt;
    logic                                    w_has_taken;
    logic                                    w_reject;
    logic                                    w_overflow;
    logic                                    w_had_input;
    // multi-taken accumulators carried through scan
    logic [TAKEN_CNT_WIDTH-1:0]              w_taken_cnt;
    logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]      w_taken_targets;

    trace_data_t                             payload;
    logic [TRACE_ADDRW-1:0]                  cand_addr;

    // FILL exit reason tracking (0=none, 1=flush, 2=pcguard,
    //           3=commit, 4=discard/accum, 5=loop-mt, 6=wait)
    fill_exit_reason = 3'd0;
    accum_latch_en  = 1'b0;

    // Defaults
    state_d        = state_q;
    commit_valid_d = 1'b0;
    commit_addr_d  = commit_addr_q;
    commit_tag_d   = commit_tag_q;
    commit_data_d  = commit_data_q;
    // default 0; each commit branch sets it explicitly. With
    // commit_valid_d also defaulted to 0 the FF gate is a no-op until a
    // branch sets both.
    tb_commit_value_ok_d = 1'b0;

    accum_base_pc_nxt      = '0;
    accum_trig_flags_nxt   = '0;
    accum_trig_cnt_nxt     = '0;
    accum_chunks_nxt       = '0;
    accum_valid_chunks_nxt = '0;
    accum_br_flags_nxt     = '0;
    accum_num_br_nxt       = '0;
    accum_taken_target_nxt = '0;
    accum_chunk_ptr_nxt    = '0;
    accum_instr_cnt_nxt    = '0;
    accum_taken_cnt_nxt    = '0;
    accum_taken_targets_nxt = '0;

    w_base_pc      = '0;
    w_has_base     = 1'b0;
    w_trig_flags   = '0;
    w_trig_cnt     = 0;
    w_has_taken    = 1'b0;
    w_reject       = 1'b0;
    w_overflow     = 1'b0;
    w_had_input    = 1'b0;
    w_taken_target = '0;
    w_exit_pc      = '0;
    w_taken_cnt    = '0;
    w_taken_targets = '0;
    w_truncate_at_return = 1'b0;
    payload        = '0;
    cand_addr      = '0;

    // In FILL, start scan from accumulated state
    if (state_q == TB_FILL) begin
      w_chunks       = accum_chunks_q;
      w_valid_chunks = accum_valid_chunks_q;
      w_br_flags     = accum_br_flags_q;
      w_num_br       = accum_num_br_q;
      w_cptr         = accum_chunk_ptr_q;
      w_icnt         = accum_instr_cnt_q;
      w_exit_pc      = accum_taken_target_q;  // default exit: branch target
      w_taken_cnt    = accum_taken_cnt_q;
      w_taken_targets = accum_taken_targets_q;
    end else begin
      w_chunks       = '0;
      w_valid_chunks = '0;
      w_br_flags     = '0;
      w_num_br       = '0;
      w_cptr         = '0;
      w_icnt         = '0;
    end

    if (flush_i) begin
      state_d = TB_IDLE;
      if (state_q == TB_FILL) fill_exit_reason = 3'd1;  // flush
    end else begin
      // ---- Scan current fetch window ----
      for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
        logic rvc, direct, taken_cf;
        logic ret_cf, call_cf;  // return / direct-call classification

        if (!(instr_i.consumed[i] && instr_i.valid[i]))
          continue;

        // FILL-PC guard: the first consumed instruction of the fill
        // window MUST start at the taken-branch target.  If the PC does
        // not match, the pipeline hasn't redirected yet ? skip without
        // setting w_had_input so the builder waits another cycle in FILL.
        if (state_q == TB_FILL && !w_had_input) begin
          if (instr_i.pc[i] != accum_taken_target_q) begin
`ifndef SYNTHESIS
            if (dbg_pcguard_miss_total < 20)
              $display("[TC-PCGUARD] t=%0t slot=%0d got_pc=0x%h expect_pc=0x%h delta=%0d",
                       $time, i, instr_i.pc[i], accum_taken_target_q,
                       instr_i.pc[i] - accum_taken_target_q);
`endif
            break;  // not the target window yet ? wait
          end else begin
`ifndef SYNTHESIS
            if (dbg_pcguard_hit_total < 20)
              $display("[TC-PCGUARD-HIT] t=%0t slot=%0d pc=0x%h TARGET MATCHED",
                       $time, i, instr_i.pc[i]);
`endif
          end
        end

        if (w_icnt >= INSTR_CNT_W'(MAX_INSTR_PER_TRACE))
          break;

        w_had_input = 1'b1;

        rvc      = is_compressed(instr_i.inst[i]);
        direct   = instr_i.is_branch[i] && is_direct_cf(instr_i.inst[i]);
        ret_cf   = instr_i.is_branch[i] && is_return_cf(instr_i.inst[i]);
        call_cf  = instr_i.is_branch[i] && is_call_cf(instr_i.inst[i]);
        taken_cf = instr_i.is_branch[i] && instr_i.taken[i];

        // IDLE: set base PC from first valid slot
        if (!w_has_base && state_q == TB_IDLE) begin
          w_base_pc  = instr_i.pc[i];
          w_has_base = 1'b1;
        end

        // FILL with taken branch ? if we've already reached MAX_TAKEN,
        // stop BEFORE this instruction (trace is full of taken branches).
        if (state_q == TB_FILL && taken_cf &&
            (w_taken_cnt >= TAKEN_CNT_WIDTH'(MAX_TAKEN))) begin
          // Don't add this instruction ? commit what we have.
          break;
        end

        // Accept direct CFs, returns (RAS-predicted target), and
        // direct calls.  Reject other indirect CFs (computed jumps).
        if (instr_i.is_branch[i] && !direct && !ret_cf && !call_cf) begin
          w_reject = 1'b1;
          break;
        end

        // Truncate trace at return instruction boundary.
        // Do not store the return or any instruction at/after it ? the
        // return target comes from the RAS and is runtime-dynamic, so
        // embedding it in the trace would produce stale wrong-path replays.
        if (ret_cf) begin
          w_truncate_at_return = 1'b1;
          break;
        end

        // Check chunk capacity
        if ((!rvc && (w_cptr + CHUNK_PTR_W'(2) > CHUNK_PTR_W'(CHUNKS_PER_TRACE))) ||
            ( rvc && (w_cptr + CHUNK_PTR_W'(1) > CHUNK_PTR_W'(CHUNKS_PER_TRACE)))) begin
          w_overflow = 1'b1;
          break;
        end

        // Store instruction as 16-bit chunks
        w_valid_chunks[w_cptr] = 1'b1;
        w_chunks[w_cptr]       = instr_i.inst[i][15:0];
        if (rvc) begin
          w_cptr = w_cptr + CHUNK_PTR_W'(1);
        end else begin
          w_valid_chunks[w_cptr + CHUNK_PTR_W'(1)] = 1'b0;
          w_chunks[w_cptr + CHUNK_PTR_W'(1)]       = instr_i.inst[i][31:16];
          w_cptr = w_cptr + CHUNK_PTR_W'(2);
        end
        w_icnt = w_icnt + INSTR_CNT_W'(1);

        // Exit PC: address following this instruction
        w_exit_pc = instr_i.pc[i] + (rvc ? PC_WIDTH'(2) : PC_WIDTH'(4));

        // Branch flag recording
        if (instr_i.is_branch[i]) begin
          if (state_q == TB_IDLE) begin
            if (w_trig_cnt < TRIGGER_BRANCH_BITS)
              w_trig_flags[w_trig_cnt] = instr_i.taken[i];
            w_trig_cnt++;
          end

          w_br_flags[w_num_br] = instr_i.taken[i];
          w_num_br = w_num_br + BR_CNT_WIDTH'(1);

          if (taken_cf) begin
            // Record this taken target in the targets array
            if (w_taken_cnt < TAKEN_CNT_WIDTH'(MAX_TAKEN))
              w_taken_targets[w_taken_cnt] = instr_i.target[i];
            w_taken_cnt = w_taken_cnt + TAKEN_CNT_WIDTH'(1);

            w_taken_target = instr_i.target[i];
            w_exit_pc      = instr_i.target[i];  // override: exit ? target
            w_has_taken    = 1'b1;
            break;  // Stop scanning this window ? next window starts at target
          end
        end
      end // for each slot

      // ---- State transitions ----
      case (state_q)
        TB_IDLE: begin
          if (w_has_taken && w_has_base && !w_reject && !w_overflow) begin
            if (w_icnt >= INSTR_CNT_W'(MAX_INSTR_PER_TRACE)) begin
              // No room for target instructions ? skip
              state_d = TB_IDLE;
            end else begin
              // Partial trace ? go to FILL for target instructions
              accum_base_pc_nxt      = w_base_pc;
              accum_trig_flags_nxt   = w_trig_flags;
              accum_trig_cnt_nxt     = TRIGGER_BRANCH_CNT_WIDTH'(w_trig_cnt);
              accum_chunks_nxt       = w_chunks;
              accum_valid_chunks_nxt = w_valid_chunks;
              accum_br_flags_nxt     = w_br_flags;
              accum_num_br_nxt       = w_num_br;
              accum_taken_target_nxt = w_taken_target;
              accum_chunk_ptr_nxt    = w_cptr;
              accum_instr_cnt_nxt    = w_icnt;
              accum_taken_cnt_nxt    = w_taken_cnt;
              accum_taken_targets_nxt = w_taken_targets;
              accum_latch_en         = 1'b1;
              state_d = TB_FILL;
            end
          end
        end

        TB_FILL: begin
          // If trace is already full entering this cycle, commit immediately.
          if (w_icnt >= INSTR_CNT_W'(MAX_INSTR_PER_TRACE) && !w_had_input && !w_reject) begin
            payload               = '0;
            payload.valid         = 1'b1;
            payload.base_pc       = accum_base_pc_q;
            payload.chunks        = accum_chunks_q;
            payload.valid_chunks  = accum_valid_chunks_q;
            payload.branch_flags  = accum_br_flags_q;
            payload.num_branches  = accum_num_br_q;
            payload.num_taken     = accum_taken_cnt_q;
            payload.taken_targets = accum_taken_targets_q;
            payload.target_addr   = accum_taken_target_q;
            payload.lookup_branch_flags = accum_br_flags_q;
            payload.lookup_num_branches = accum_num_br_q;

            cand_addr = tc_index(accum_base_pc_q, accum_ghr_q);
            commit_addr_d  = cand_addr;
            commit_tag_d   = make_trace_tag(accum_base_pc_q,
                                            accum_trig_cnt_q,
                                            accum_trig_flags_q,
                                            accum_ghr_q);
            commit_valid_d = 1'b1;
            commit_data_d  = payload;
            // at-entry-full path ? instr count == MAX_INSTR_PER_TRACE,
            // which is >= TB_VALUE_MIN_LEN, so always passes the value filter.
            tb_commit_value_ok_d = (accum_taken_cnt_q >= TAKEN_CNT_WIDTH'(2)) ||
                                   (INSTR_CNT_W'(MAX_INSTR_PER_TRACE) >= INSTR_CNT_W'(TB_VALUE_MIN_LEN));
            state_d = TB_IDLE;
            fill_exit_reason = 3'd3;  // early-commit (full at entry)
          end else if (w_had_input || w_reject || w_overflow) begin
            logic fill_added;
            fill_added = (w_icnt > accum_instr_cnt_q);

            if (fill_added) begin
              if (w_has_taken &&
                  (w_icnt < INSTR_CNT_W'(MAX_INSTR_PER_TRACE)) &&
                  !w_reject && !w_overflow) begin
                // Another taken branch found AND room for more ?
                // loop back to FILL for the next target window.
                accum_base_pc_nxt      = accum_base_pc_q;  // keep original base
                accum_trig_flags_nxt   = accum_trig_flags_q;
                accum_trig_cnt_nxt     = accum_trig_cnt_q;
                accum_chunks_nxt       = w_chunks;
                accum_valid_chunks_nxt = w_valid_chunks;
                accum_br_flags_nxt     = w_br_flags;
                accum_num_br_nxt       = w_num_br;
                accum_taken_target_nxt = w_taken_target;
                accum_chunk_ptr_nxt    = w_cptr;
                accum_instr_cnt_nxt    = w_icnt;
                accum_taken_cnt_nxt    = w_taken_cnt;
                accum_taken_targets_nxt = w_taken_targets;
                accum_latch_en         = 1'b1;
                state_d = TB_FILL;  // loop
                fill_exit_reason = 3'd5;  // loop-mt (multi-taken)
              end else begin
                // No more taken branches, or trace full ? commit
                payload               = '0;
                payload.valid         = 1'b1;
                payload.base_pc       = accum_base_pc_q;
                payload.chunks        = w_chunks;
                payload.valid_chunks  = w_valid_chunks;
                payload.branch_flags  = w_br_flags;
                payload.num_branches  = w_num_br;
                payload.num_taken     = w_taken_cnt;
                payload.taken_targets = w_taken_targets;
                payload.target_addr   = w_exit_pc;
                payload.lookup_branch_flags = w_br_flags;
                payload.lookup_num_branches = w_num_br;

                cand_addr = tc_index(accum_base_pc_q, accum_ghr_q);

                commit_addr_d  = cand_addr;
                commit_tag_d   = make_trace_tag(accum_base_pc_q,
                                                accum_trig_cnt_q,
                                                accum_trig_flags_q,
                                                accum_ghr_q);
                commit_valid_d = 1'b1;
                commit_data_d  = payload;
                // value filter ? multi-taken, or length >= TB_VALUE_MIN_LEN.
                tb_commit_value_ok_d = (w_taken_cnt >= TAKEN_CNT_WIDTH'(2)) ||
                                       (w_icnt      >= INSTR_CNT_W'(TB_VALUE_MIN_LEN));
                state_d = TB_IDLE;
                fill_exit_reason = 3'd3;  // commit (fill_added, no-taken/full)
              end
            end else begin
              // No new instructions added this cycle, but we may have a
              // multi-block trace from previous FILL loops.  Commit if
              // accumulated count exceeds 1 instruction (the IDLE window).
              if (accum_instr_cnt_q > INSTR_CNT_W'(1)) begin
                payload               = '0;
                payload.valid         = 1'b1;
                payload.base_pc       = accum_base_pc_q;
                payload.chunks        = accum_chunks_q;
                payload.valid_chunks  = accum_valid_chunks_q;
                payload.branch_flags  = accum_br_flags_q;
                payload.num_branches  = accum_num_br_q;
                payload.num_taken     = accum_taken_cnt_q;
                payload.taken_targets = accum_taken_targets_q;
                payload.target_addr   = accum_taken_target_q;
                payload.lookup_branch_flags = accum_br_flags_q;
                payload.lookup_num_branches = accum_num_br_q;

                cand_addr = tc_index(accum_base_pc_q, accum_ghr_q);
                commit_addr_d  = cand_addr;
                commit_tag_d   = make_trace_tag(accum_base_pc_q,
                                                accum_trig_cnt_q,
                                                accum_trig_flags_q,
                                                accum_ghr_q);
                commit_valid_d = 1'b1;
                commit_data_d  = payload;
                // value filter ? multi-taken, or length >= TB_VALUE_MIN_LEN.
                tb_commit_value_ok_d = (accum_taken_cnt_q >= TAKEN_CNT_WIDTH'(2)) ||
                                       (accum_instr_cnt_q >= INSTR_CNT_W'(TB_VALUE_MIN_LEN));
              end
              state_d = TB_IDLE;
              fill_exit_reason = (commit_valid_d) ? 3'd4 : 3'd2;  // 4=accum-commit, 2=discard(pcguard/etc)
            end
          end
          // else: no input yet ? stay in FILL
          if (state_q == TB_FILL && state_d == TB_FILL && fill_exit_reason == 3'd0)
            fill_exit_reason = 3'd6;  // wait (no input)
        end
      endcase

      // suppress the write if the truncated trace is shorter
      // than the replay-time minimum.  A too-short trace written to SRAM
      // would be rejected on every replay anyway, wasting a slot.
      // Also fix fill_exit_reason so dbg_fill_commit_q does not count this
      // as a successful commit (reclassify to 2 = discard).
      if (w_truncate_at_return && commit_valid_d &&
          (w_icnt < INSTR_CNT_W'(TB_MIN_ACCEPT_LEN))) begin
        commit_valid_d   = 1'b0;
        fill_exit_reason = 3'd2;  // discard ? return made trace too short
      end

      // A replay fired, so fetch jumped to exit_pc and the target window we
      // were sitting in FILL waiting for is never coming.  Drop back to IDLE
      // and let the accumulators clear.  No commit can be lost here: the
      // frontend zeroed instr valid this cycle, so the scan made no progress.
      if (tc_replay_fired_i && state_d == TB_FILL) begin
        state_d          = TB_IDLE;
        accum_latch_en   = 1'b0;
        fill_exit_reason = 3'd7;  // tc-replay-reset
      end
    end
  end

`ifndef SYNTHESIS
  longint unsigned dbg_commit_q;
  longint unsigned dbg_fill_start_q;
  longint unsigned dbg_fill_loop_q;  // FILL?FILL loops (multi-taken)

  // FILL exit reason breakdown
  longint unsigned dbg_fill_flush_q;      // flush killed FILL
  longint unsigned dbg_fill_pcguard_q;    // PC guard rejected (w_reject from guard)
  longint unsigned dbg_fill_commit_q;     // fill_added ? commit (no taken / full)
  longint unsigned dbg_fill_discard_q;    // !fill_added ? discard/accum-commit
  longint unsigned dbg_fill_loop_mt_q;    // fill_added + taken + room ? FILL loop
  longint unsigned dbg_fill_wait_q;       // no input, stayed in FILL
  longint unsigned dbg_fill_tc_reset_q;   // TB_FILL aborted by tc_replay_fired_i

  // counters
  longint unsigned dbg_ret_truncated_q;   // trace shortened before return, still accepted
  longint unsigned dbg_ret_rejected_q;    // trace dropped because return made it too short

  // hot-PC tracker: monitor builder activity for the top mismatch PCs
  localparam logic [PC_WIDTH-1:0] HOT_PC_0 = 64'h80002880;
  localparam logic [PC_WIDTH-1:0] HOT_PC_1 = 64'h8000287a;
  localparam logic [PC_WIDTH-1:0] HOT_PC_2 = 64'h80002888;  // stored neighbour
  localparam logic [PC_WIDTH-1:0] HOT_PC_3 = 64'h8000287e;  // stored neighbour

  int unsigned dbg_fill_hot0, dbg_fill_hot1, dbg_fill_hot2, dbg_fill_hot3;
  int unsigned dbg_commit_hot0, dbg_commit_hot1, dbg_commit_hot2, dbg_commit_hot3;
  // how many builder commits were dropped by the value filter.
  longint unsigned dbg_value_filter_drops_q;
  longint unsigned dbg_value_filter_pass_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dbg_commit_q     <= 0;
      dbg_fill_start_q <= 0;
      dbg_fill_loop_q  <= 0;
      dbg_fill_flush_q    <= 0;
      dbg_fill_pcguard_q  <= 0;
      dbg_fill_commit_q   <= 0;
      dbg_fill_discard_q  <= 0;
      dbg_fill_loop_mt_q  <= 0;
      dbg_fill_wait_q     <= 0;
      dbg_fill_tc_reset_q <= 0;
      dbg_ret_truncated_q <= 0;
      dbg_ret_rejected_q  <= 0;
      dbg_pcguard_miss_total <= 0;
      dbg_pcguard_hit_total  <= 0;
      dbg_fill_hot0 <= 0; dbg_fill_hot1 <= 0; dbg_fill_hot2 <= 0; dbg_fill_hot3 <= 0;
      dbg_commit_hot0 <= 0; dbg_commit_hot1 <= 0; dbg_commit_hot2 <= 0; dbg_commit_hot3 <= 0;
      dbg_value_filter_drops_q <= 0;
      dbg_value_filter_pass_q  <= 0;
    end else begin
      if (commit_valid_d)                              dbg_commit_q     <= dbg_commit_q + 1;
      // split commits by value-filter outcome.
      if (commit_valid_d &&  tb_commit_value_ok_d)     dbg_value_filter_pass_q  <= dbg_value_filter_pass_q  + 1;
      if (commit_valid_d && !tb_commit_value_ok_d)     dbg_value_filter_drops_q <= dbg_value_filter_drops_q + 1;
      // count FILL?FILL loops (multi-taken continuations)
      if (state_q == TB_FILL && state_d == TB_FILL)
        dbg_fill_loop_q <= dbg_fill_loop_q + 1;
      // FILL exit reason counters
      if (state_q == TB_FILL) begin
        case (fill_exit_reason)
          3'd1: dbg_fill_flush_q    <= dbg_fill_flush_q + 1;
          3'd2: dbg_fill_pcguard_q  <= dbg_fill_pcguard_q + 1;  // discard (pcguard or other)
          3'd3: dbg_fill_commit_q   <= dbg_fill_commit_q + 1;
          3'd4: dbg_fill_discard_q  <= dbg_fill_discard_q + 1;  // accum-commit (!fill_added)
          3'd5: dbg_fill_loop_mt_q  <= dbg_fill_loop_mt_q + 1;
          3'd6: dbg_fill_wait_q     <= dbg_fill_wait_q + 1;
          3'd7: dbg_fill_tc_reset_q <= dbg_fill_tc_reset_q + 1;
          default: ;
        endcase
        // Count cycles where PC guard gets a valid but wrong-PC window
        if (fill_exit_reason == 3'd6) begin
          // Check if any slots were valid (wrong-PC wait) vs no slots valid (empty wait)
          if (|instr_i.valid)
            dbg_pcguard_miss_total <= dbg_pcguard_miss_total + 1;
        end
        // Count successful PC guard matches (w_had_input means guard passed)
        if (fill_exit_reason != 3'd1 && fill_exit_reason != 3'd6 &&
            fill_exit_reason != 3'd0 && fill_exit_reason != 3'd7)
          dbg_pcguard_hit_total <= dbg_pcguard_hit_total + 1;
      end
      // count all return encounters ? TB_IDLE (trace never started)
      // and TB_FILL (truncated-and-accepted vs. too-short-or-no-taken rejected).
      if (w_truncate_at_return) begin
        if (commit_valid_d)
          dbg_ret_truncated_q <= dbg_ret_truncated_q + 1;
        else
          dbg_ret_rejected_q  <= dbg_ret_rejected_q  + 1;
      end

      if (state_d == TB_FILL && state_q == TB_IDLE) begin
        dbg_fill_start_q <= dbg_fill_start_q + 1;
        if (accum_base_pc_nxt == HOT_PC_0) begin dbg_fill_hot0 <= dbg_fill_hot0 + 1;
          $display("[TC-HOTPC] t=%0t FILL_START base_pc=0x%h (HOT0)", $time, accum_base_pc_nxt[31:0]); end
        if (accum_base_pc_nxt == HOT_PC_1) begin dbg_fill_hot1 <= dbg_fill_hot1 + 1;
          $display("[TC-HOTPC] t=%0t FILL_START base_pc=0x%h (HOT1)", $time, accum_base_pc_nxt[31:0]); end
        if (accum_base_pc_nxt == HOT_PC_2) begin dbg_fill_hot2 <= dbg_fill_hot2 + 1;
          $display("[TC-HOTPC] t=%0t FILL_START base_pc=0x%h (HOT2)", $time, accum_base_pc_nxt[31:0]); end
        if (accum_base_pc_nxt == HOT_PC_3) begin dbg_fill_hot3 <= dbg_fill_hot3 + 1;
          $display("[TC-HOTPC] t=%0t FILL_START base_pc=0x%h (HOT3)", $time, accum_base_pc_nxt[31:0]); end
      end
      if (commit_valid_d) begin
        if (accum_base_pc_q == HOT_PC_0) begin dbg_commit_hot0 <= dbg_commit_hot0 + 1;
          $display("[TC-HOTPC] t=%0t COMMIT base_pc=0x%h nbr=%0d bflags=%b (HOT0)", $time, accum_base_pc_q[31:0], accum_trig_cnt_q, accum_trig_flags_q); end
        if (accum_base_pc_q == HOT_PC_1) begin dbg_commit_hot1 <= dbg_commit_hot1 + 1;
          $display("[TC-HOTPC] t=%0t COMMIT base_pc=0x%h nbr=%0d bflags=%b (HOT1)", $time, accum_base_pc_q[31:0], accum_trig_cnt_q, accum_trig_flags_q); end
        if (accum_base_pc_q == HOT_PC_2) begin dbg_commit_hot2 <= dbg_commit_hot2 + 1;
          $display("[TC-HOTPC] t=%0t COMMIT base_pc=0x%h nbr=%0d bflags=%b (HOT2)", $time, accum_base_pc_q[31:0], accum_trig_cnt_q, accum_trig_flags_q); end
        if (accum_base_pc_q == HOT_PC_3) begin dbg_commit_hot3 <= dbg_commit_hot3 + 1;
          $display("[TC-HOTPC] t=%0t COMMIT base_pc=0x%h nbr=%0d bflags=%b (HOT3)", $time, accum_base_pc_q[31:0], accum_trig_cnt_q, accum_trig_flags_q); end
      end
    end
  end

  final begin
    $display("[TC-BUILDER] V90 multi-taken: commits=%0d fill_starts=%0d fill_loops=%0d",
             dbg_commit_q, dbg_fill_start_q, dbg_fill_loop_q);
    $display("[TC-BUILDER-DIAG] flush=%0d pcguard_discard=%0d commit=%0d accum_commit=%0d loop_mt=%0d wait=%0d tc_reset=%0d",
             dbg_fill_flush_q, dbg_fill_pcguard_q, dbg_fill_commit_q,
             dbg_fill_discard_q, dbg_fill_loop_mt_q, dbg_fill_wait_q,
             dbg_fill_tc_reset_q);
    $display("[TC-BUILDER-DIAG2] pcguard_hit=%0d pcguard_miss_with_valid=%0d wait_no_valid=%0d",
             dbg_pcguard_hit_total, dbg_pcguard_miss_total,
             dbg_fill_wait_q - dbg_pcguard_miss_total);
    $display("[TC-FINAL] tc_truncated_return=%0d tc_rejected_return=%0d",
             dbg_ret_truncated_q, dbg_ret_rejected_q);
    $display("[TC-V95] builder_value_filter: pass=%0d drop=%0d (drop = single-taken trace with len<%0d)",
             dbg_value_filter_pass_q, dbg_value_filter_drops_q, TB_VALUE_MIN_LEN);
    $display("[TC-HOTPC-BUILDER] fill_starts: 0x%h=%0d  0x%h=%0d  0x%h=%0d  0x%h=%0d",
             HOT_PC_0[31:0], dbg_fill_hot0, HOT_PC_1[31:0], dbg_fill_hot1,
             HOT_PC_2[31:0], dbg_fill_hot2, HOT_PC_3[31:0], dbg_fill_hot3);
    $display("[TC-HOTPC-BUILDER] commits:     0x%h=%0d  0x%h=%0d  0x%h=%0d  0x%h=%0d",
             HOT_PC_0[31:0], dbg_commit_hot0, HOT_PC_1[31:0], dbg_commit_hot1,
             HOT_PC_2[31:0], dbg_commit_hot2, HOT_PC_3[31:0], dbg_commit_hot3);
  end
`endif

endmodule
