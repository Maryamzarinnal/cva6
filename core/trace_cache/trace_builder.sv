`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// V70 single-window trace builder.
// Records up to TRACE_LEN (4) instructions spanning one taken branch.
// The trace stitches the tail of one basic block with the head of the next,
// eliminating the taken-branch redirect bubble for the frontend.
//
// State machine:
//   IDLE ? taken direct branch found ? FILL (if room for target instructions)
//   FILL ? fill remaining slots from target window ? commit
//
// If the taken branch fills the trace's last slot, there is no room for
// target instructions ? the trace would be identical to the I-cache output,
// so it is not recorded.
//
// Indirect/call branches are rejected (target unpredictable / RAS update
// not handled during trace replay).

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
  localparam int unsigned INSTR_CNT_W = $clog2(MAX_INSTR_PER_TRACE + 1);

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
      commit_valid_q <= commit_valid_d;
      commit_addr_q  <= commit_addr_d;
      commit_tag_q   <= commit_tag_d;
      commit_data_q  <= commit_data_d;
    end
  end

  // Accumulation registers: latch on IDLE?FILL, clear on flush or return to IDLE.
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
    end else if (state_d == TB_FILL && state_q == TB_IDLE) begin
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
    end
  end

  // -----------------------------------------------------------------------
  // V70 combinational scan: IDLE finds a taken branch, FILL gets target
  // instructions, commit writes trace to SRAM.
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

    trace_data_t                             payload;
    logic [TRACE_ADDRW-1:0]                  cand_addr;

    // Defaults
    state_d        = state_q;
    commit_valid_d = 1'b0;
    commit_addr_d  = commit_addr_q;
    commit_tag_d   = commit_tag_q;
    commit_data_d  = commit_data_q;

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
    end else begin
      // ---- Scan current fetch window ----
      for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
        logic rvc, direct, taken_cf;

        if (!(instr_i.consumed[i] && instr_i.valid[i]))
          continue;

        // V71 FILL-PC guard: the first instruction of the fill window MUST
        // start at the taken-branch target.  If the target window was
        // invisible (IQ full / tc_hit gating), the builder stays in FILL
        // and the NEXT visible window has the wrong instructions.  Reject
        // that case by verifying the PC of the first fill-phase instruction.
        if (state_q == TB_FILL && !w_had_input) begin
          if (instr_i.pc[i] != accum_taken_target_q) begin
            w_reject = 1'b1;
            break;
          end
        end

        if (w_icnt >= INSTR_CNT_W'(MAX_INSTR_PER_TRACE))
          break;

        w_had_input = 1'b1;

        rvc      = is_compressed(instr_i.inst[i]);
        direct   = instr_i.is_branch[i] && is_direct_cf(instr_i.inst[i]);
        taken_cf = instr_i.is_branch[i] && instr_i.taken[i];

        // IDLE: set base PC from first valid slot
        if (!w_has_base && state_q == TB_IDLE) begin
          w_base_pc  = instr_i.pc[i];
          w_has_base = 1'b1;
        end

        // FILL: stop BEFORE any taken CF (trace already has MAX_TAKEN=1)
        if (state_q == TB_FILL && taken_cf) begin
          w_has_taken = 1'b1;
          break;
        end

        // Reject indirect/call branches in both states
        if (instr_i.is_branch[i] && !direct) begin
          w_reject = 1'b1;
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

          // IDLE: first taken branch ? stop scanning, prepare for FILL
          if (state_q == TB_IDLE && taken_cf) begin
            w_taken_target = instr_i.target[i];
            w_exit_pc      = instr_i.target[i];  // override: exit ? target
            w_has_taken    = 1'b1;
            break;
          end
        end
      end // for each slot

      // ---- State transitions ----
      case (state_q)
        TB_IDLE: begin
          if (w_has_taken && w_has_base && !w_reject && !w_overflow) begin
            if (w_icnt >= INSTR_CNT_W'(MAX_INSTR_PER_TRACE)) begin
              // No room for target instructions ? skip (no benefit over I-cache)
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
              state_d = TB_FILL;
            end
          end
          // else: no taken branch or rejected ? stay IDLE
        end

        TB_FILL: begin
          if (w_had_input || w_reject || w_overflow) begin
            logic fill_added;
            fill_added = (w_icnt > accum_instr_cnt_q);

            if (fill_added) begin
              // Successfully got target instructions ? commit trace
              payload               = '0;
              payload.valid         = 1'b1;
              payload.base_pc       = accum_base_pc_q;
              payload.chunks        = w_chunks;
              payload.valid_chunks  = w_valid_chunks;
              payload.branch_flags  = w_br_flags;
              payload.num_branches  = w_num_br;
              payload.num_taken     = TAKEN_CNT_WIDTH'(1);
              payload.taken_targets[0] = accum_taken_target_q;
              payload.target_addr   = w_exit_pc;
              payload.lookup_branch_flags = w_br_flags;
              payload.lookup_num_branches = w_num_br;

              cand_addr = tc_index(accum_base_pc_q);

              commit_addr_d  = cand_addr;
              commit_tag_d   = make_trace_tag(accum_base_pc_q,
                                              accum_trig_cnt_q,
                                              accum_trig_flags_q);
              commit_valid_d = 1'b1;
              commit_data_d  = payload;
            end
            // else: no target instructions added ? discard (no benefit)

            state_d = TB_IDLE;
          end
          // else: no input yet ? stay in FILL (wait for target window)
        end
      endcase
    end
  end

`ifndef SYNTHESIS
  longint unsigned dbg_commit_q;
  longint unsigned dbg_fill_start_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dbg_commit_q     <= 0;
      dbg_fill_start_q <= 0;
    end else begin
      if (commit_valid_d)                              dbg_commit_q     <= dbg_commit_q + 1;
      if (state_d == TB_FILL && state_q == TB_IDLE)    dbg_fill_start_q <= dbg_fill_start_q + 1;
    end
  end

  final begin
    $display("[TC-BUILDER] V70 1-window: commits=%0d fill_starts=%0d",
             dbg_commit_q, dbg_fill_start_q);
  end
`endif

endmodule
