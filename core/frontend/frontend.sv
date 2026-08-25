// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 08.02.2018
// Description: Ariane Instruction Fetch Frontend with Trace Cache support.

module frontend
  import ariane_pkg::*;
  import trace_cache_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type bp_resolve_t = logic,
    parameter type fetch_entry_t = logic,
    parameter type icache_dreq_t = logic,
    parameter type icache_drsp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,
    input logic [CVA6Cfg.VLEN-1:0] boot_addr_i,
    input logic flush_bp_i,
    input logic flush_i,
    input logic halt_i,
    input logic halt_frontend_i,
    input logic set_pc_commit_i,
    input logic [CVA6Cfg.VLEN-1:0] pc_commit_i,
    input logic ex_valid_i,
    input bp_resolve_t resolved_branch_i,
    input logic eret_i,
    input logic [CVA6Cfg.VLEN-1:0] epc_i,
    input logic [CVA6Cfg.VLEN-1:0] trap_vector_base_i,
    input logic set_debug_pc_i,
    input logic debug_mode_i,
    output icache_dreq_t icache_dreq_o,
    input icache_drsp_t icache_dreq_i,
    output fetch_entry_t [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_o,
    output logic [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_valid_o,
    input logic [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_ready_i
);

`ifndef SYNTHESIS
  initial $display("[TC-RUN-MARKER] TC_V74_DEDUP_GUARD");
`endif

  localparam type bht_update_t = struct packed {
    logic                    valid;
    logic [CVA6Cfg.VLEN-1:0] pc;
    logic                    taken;
  };
  localparam type btb_prediction_t = struct packed {
    logic                    valid;
    logic [CVA6Cfg.VLEN-1:0] target_address;
  };
  localparam type btb_update_t = struct packed {
    logic                    valid;
    logic [CVA6Cfg.VLEN-1:0] pc;
    logic [CVA6Cfg.VLEN-1:0] target_address;
  };
  localparam type ras_t = struct packed {
    logic                    valid;
    logic [CVA6Cfg.VLEN-1:0] ra;
  };

  logic [CVA6Cfg.FETCH_WIDTH-1:0]          icache_data_q;
  logic                                    icache_valid_q;
  ariane_pkg::frontend_exception_t         icache_ex_valid_q;
  logic [CVA6Cfg.VLEN-1:0]                 icache_vaddr_q;
  logic [CVA6Cfg.GPLEN-1:0]                icache_gpaddr_q;
  logic [31:0]                             icache_tinst_q;
  logic                                    icache_gva_q;
  logic                                    instr_queue_ready;
  logic                                    instr_queue_empty;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]      instr_queue_consumed;
  btb_prediction_t                         btb_q;
  bht_prediction_t                         bht_q;
  logic                                    if_ready;
  logic [CVA6Cfg.VLEN-1:0]                 npc_d, npc_q;
  logic                                    npc_rst_load_q;
  logic                                    replay;
  logic                                    replay_eff;
  logic [CVA6Cfg.VLEN-1:0]                 replay_addr;

  // -----------------------------------------------------------------------
  // Trace cache state ? parallel lookup, same-cycle competition with the I$
  // -----------------------------------------------------------------------
  logic                                    tc_trace_hit;
  logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]   tc_trace_instructions;
  logic [TRACE_LEN_WIDTH-1:0]              tc_trace_length;
  logic [CHUNKS_PER_TRACE-1:0][15:0]       tc_trace_chunks;
  logic [CHUNKS_PER_TRACE-1:0]             tc_trace_valid_chunks;
  logic [TRACE_LEN-1:0][PC_WIDTH-1:0]      tc_trace_pcs;
  logic [CHUNKS_PER_TRACE-1:0]             tc_trace_branch_flags;
  logic [BR_CNT_WIDTH-1:0]                 tc_trace_num_branches;
  logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]      tc_trace_taken_targets;
  logic [TAKEN_CNT_WIDTH-1:0]              tc_trace_num_taken;
  logic [PC_WIDTH-1:0]                     tc_trace_next_pc;
  logic [31:0]                             tc_miss_total, tc_miss_empty, tc_miss_pc, tc_miss_path;
  logic                                    tc_lookup_result_valid;
  logic                                    tc_miss_reason_empty, tc_miss_reason_pc, tc_miss_reason_path;
  logic                                    tc_trace_used;
  logic                                    tc_active_use;
  logic                                    tc_active_hit;
  logic                                    tc_trace_starts_ok;
  logic                                    tc_trace_last_is_taken_cf;
  logic [BR_CNT_WIDTH-1:0]                 tc_trace_mapped_num_branches;
  logic [TAKEN_CNT_WIDTH-1:0]              tc_trace_mapped_num_taken;
  logic                                    tc_trace_branch_map_ok;
  logic                                    tc_trace_has_call;
  logic                                    tc_trace_has_indirect;

  // same-cycle hit decision ? replaces pending/replay model.
  logic                                    tc_hit_this_cycle;
  logic                                    tc_suppress_port1;  // tied to 0
  logic [SLOTS_PER_CYCLE-1:0]              tc_valid_mask;  // valid up to first taken

  // 1-entry pending trace buffer ? captures TC hit when IQ not ready.
  logic                                    tc_pending_valid_q;
  logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]   tc_pending_instructions_q;
  logic [TRACE_LEN_WIDTH-1:0]              tc_pending_length_q;
  logic [PC_WIDTH-1:0]                     tc_pending_next_pc_q;
  logic [TRACE_LEN-1:0][PC_WIDTH-1:0]      tc_pending_pcs_q;
  logic [CHUNKS_PER_TRACE-1:0]             tc_pending_branch_flags_q;
  logic [BR_CNT_WIDTH-1:0]                 tc_pending_num_branches_q;
  logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]      tc_pending_taken_targets_q;
  logic [TAKEN_CNT_WIDTH-1:0]              tc_pending_num_taken_q;
  logic [PC_WIDTH-1:0]                     tc_pending_base_pc_q;  // for PC safety check
  logic [7:0]                              tc_pending_ghr_q;      // GHR snapshot at capture time
  // Pending buffer control signals
  logic                                    tc_pending_capture;  // TC hit + IQ not ready
  logic                                    tc_pending_use;      // pending valid + IQ ready + safe
  logic                                    tc_pending_invalidate; // any path-breaking event
  // Suppress I$ data into IQ while pending buffer owns the trace.
  // Active during the capture cycle AND all subsequent cycles until the
  // pending buffer is consumed (tc_pending_use) or invalidated.  This
  // prevents the I$ scan from partially inserting instructions into
  // non-full IQ lanes that overlap with the captured trace.
  logic                                    tc_suppress_icache;

  // Per-slot TC feeding signals (combinational from trace outputs).
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                    tc_feed_valid;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]              tc_feed_instr;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0]  tc_feed_addr;
  cf_t  [CVA6Cfg.INSTR_PER_FETCH-1:0]                    tc_feed_cf;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0]  tc_feed_predict_addr;

  logic [CHUNKS_PER_TRACE-1:0]                          tc_branch_predictions;
  logic [CHUNKS_PER_TRACE-1:0]                          branch_flags_to_iq;
  // MUX outputs to instr_queue
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]             instr_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] addr_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                   valid_to_iq;
  cf_t  [CVA6Cfg.INSTR_PER_FETCH-1:0]                   cf_type_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] predict_addr_to_iq;
  logic [7:0]                                           tc_ghr_from_tc;        // current GHR from trace cache
  logic [7:0]                                           tc_ghr_to_iq;          // GHR snapshot for pipeline propagation
  ariane_pkg::frontend_exception_t                      exception_to_iq;
  logic [CVA6Cfg.VLEN-1:0]                              exception_addr_to_iq;
  logic [CVA6Cfg.GPLEN-1:0]                             exception_gpaddr_to_iq;
  logic [31:0]                                          exception_tinst_to_iq;
  logic                                                 exception_gva_to_iq;
  logic                                                 tc_disable_replay_q;
  logic                                                 tc_no_iq_gate_q;
  logic                                                 tc_no_path_gate_q;
  logic                                                 tc_loose_lo_gate_q;

  function automatic cf_t tc_replay_cf_type(input logic [31:0] instr_i);
    logic is_rvc;
    begin
      tc_replay_cf_type = ariane_pkg::NoCF;
      is_rvc = (instr_i[1:0] != 2'b11);

      if (!is_rvc) begin
        unique case (instr_i[6:0])
          riscv::OpcodeBranch: tc_replay_cf_type = ariane_pkg::Branch;
          riscv::OpcodeJal:    tc_replay_cf_type = ariane_pkg::Jump;
          riscv::OpcodeJalr: begin
            if (((instr_i[19:15] == 5'd1) || (instr_i[19:15] == 5'd5)) &&
                (instr_i[19:15] != instr_i[11:7]))
              tc_replay_cf_type = ariane_pkg::Return;
            else
              tc_replay_cf_type = ariane_pkg::JumpR;
          end
          default: ;
        endcase
      end else begin
        if ((instr_i[1:0] == riscv::OpcodeC1) &&
            ((instr_i[15:13] == riscv::OpcodeC1Beqz) ||
             (instr_i[15:13] == riscv::OpcodeC1Bnez))) begin
          tc_replay_cf_type = ariane_pkg::Branch;
        end else if ((instr_i[1:0] == riscv::OpcodeC1) &&
                     ((instr_i[15:13] == riscv::OpcodeC1J) ||
                      ((CVA6Cfg.XLEN == 32) && (instr_i[15:13] == riscv::OpcodeC1Jal)))) begin
          tc_replay_cf_type = ariane_pkg::Jump;
        end else if ((instr_i[1:0] == riscv::OpcodeC2) &&
                     (instr_i[15:13] == riscv::OpcodeC2JalrMvAdd) &&
                     (instr_i[6:2] == 5'b00000)) begin
          if (!instr_i[12] &&
              ((instr_i[11:7] == 5'd1) || (instr_i[11:7] == 5'd5)))
            tc_replay_cf_type = ariane_pkg::Return;
          else
            tc_replay_cf_type = ariane_pkg::JumpR;
        end
      end
    end
  endfunction

  function automatic logic tc_replay_is_call(input logic [31:0] instr_i);
    logic is_rvc;
    begin
      tc_replay_is_call = 1'b0;
      is_rvc = (instr_i[1:0] != 2'b11);

      if (!is_rvc) begin
        if ((instr_i[6:0] == riscv::OpcodeJal) &&
            ((instr_i[11:7] == 5'd1) || (instr_i[11:7] == 5'd5)))
          tc_replay_is_call = 1'b1;
      end else begin
        if ((CVA6Cfg.XLEN == 32) &&
            (instr_i[1:0] == riscv::OpcodeC1) &&
            (instr_i[15:13] == riscv::OpcodeC1Jal))
          tc_replay_is_call = 1'b1;
      end
    end
  endfunction

  logic [$clog2(CVA6Cfg.INSTR_PER_FETCH)-1:0] shamt;
  if (CVA6Cfg.RVC) begin : gen_shamt
    assign shamt = icache_dreq_i.vaddr[$clog2(CVA6Cfg.INSTR_PER_FETCH):1];
  end else begin
    assign shamt = 1'b0;
  end

  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] rvi_return, rvi_call, rvi_branch, rvi_jalr, rvi_jump;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] rvi_imm;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] rvc_branch, rvc_jump, rvc_jr, rvc_return, rvc_jalr, rvc_call;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] rvc_imm;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]              instr;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0]  addr;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                    instruction_valid;
  bht_prediction_t [CVA6Cfg.INSTR_PER_FETCH-1:0]         bht_prediction;
  btb_prediction_t [CVA6Cfg.INSTR_PER_FETCH-1:0]         btb_prediction;
  bht_prediction_t [CVA6Cfg.INSTR_PER_FETCH-1:0]         bht_prediction_shifted;
  btb_prediction_t [CVA6Cfg.INSTR_PER_FETCH-1:0]         btb_prediction_shifted;
  ras_t                                                   ras_predict;
  logic [CVA6Cfg.VLEN-1:0]                               vpc_btb, vpc_bht;
  logic                                                   is_mispredict;
  logic ras_push, ras_pop;
  logic [CVA6Cfg.VLEN-1:0]                               ras_update;
  logic [CVA6Cfg.VLEN-1:0]                               predict_address;
  cf_t [CVA6Cfg.INSTR_PER_FETCH-1:0]                     cf_type;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                    taken_rvi_cf, taken_rvc_cf;
  logic                                                   serving_unaligned;
  // combinational next-cycle alignment forecast from instr_realign.
  logic                                                   next_serving_unaligned;
  logic [CVA6Cfg.VLEN-1:0]                                next_unaligned_address;

  instr_realign #(.CVA6Cfg(CVA6Cfg)) i_instr_realign (
      .clk_i, .rst_ni,
      .flush_i                  (icache_dreq_o.kill_s2),
      .valid_i                  (icache_valid_q),
      .serving_unaligned_o      (serving_unaligned),
      .address_i                (icache_vaddr_q),
      .data_i                   (icache_data_q),
      .valid_o                  (instruction_valid),
      .addr_o                   (addr),
      .instr_o                  (instr),
      .next_serving_unaligned_o (next_serving_unaligned),
      .next_unaligned_address_o (next_unaligned_address)
  );

  if (CVA6Cfg.RVC) begin : gen_btb_prediction_shifted
    assign bht_prediction_shifted[0] = (serving_unaligned) ? bht_q : bht_prediction[addr[0][$clog2(CVA6Cfg.INSTR_PER_FETCH):1]];
    assign btb_prediction_shifted[0] = (serving_unaligned) ? btb_q : btb_prediction[addr[0][$clog2(CVA6Cfg.INSTR_PER_FETCH):1]];
    for (genvar i = 1; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_prediction_address
      assign bht_prediction_shifted[i] = bht_prediction[addr[i][$clog2(CVA6Cfg.INSTR_PER_FETCH):1]];
      assign btb_prediction_shifted[i] = btb_prediction[addr[i][$clog2(CVA6Cfg.INSTR_PER_FETCH):1]];
    end
  end else begin
    assign bht_prediction_shifted[0] = (serving_unaligned) ? bht_q : bht_prediction[addr[0][1]];
    assign btb_prediction_shifted[0] = (serving_unaligned) ? btb_q : btb_prediction[addr[0][1]];
  end

  // instruction scan always runs ? no gating during TC hit.
  // Side effects (RAS push/pop) are suppressed separately in tc_hit_this_cycle.
  logic bp_valid;
  logic tc_icache_sidefx_en;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] is_branch, is_call, is_jump, is_return, is_jalr;
  logic [CHUNKS_PER_TRACE-1:0] iq_branch_flags_out;
  logic [CHUNKS_PER_TRACE-1:0] tc_active_branch_flags;
  logic [CHUNKS_PER_TRACE-1:0] restored_branch_flags;

  assign tc_icache_sidefx_en = 1'b1;

  for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin
    assign is_branch[i] = instruction_valid[i] & (rvi_branch[i] | rvc_branch[i]);
    assign is_call[i]   = instruction_valid[i] & (rvi_call[i]   | rvc_call[i]);
    assign is_return[i] = instruction_valid[i] & (rvi_return[i] | rvc_return[i]);
    assign is_jump[i]   = instruction_valid[i] & (rvi_jump[i]   | rvc_jump[i]);
    assign is_jalr[i]   = instruction_valid[i] & ~is_return[i] & (rvi_jalr[i] | rvc_jalr[i] | rvc_jr[i]);
  end

  always_comb begin
    taken_rvi_cf = '0;
    taken_rvc_cf = '0;
    predict_address = '0;
    for (int i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) cf_type[i] = ariane_pkg::NoCF;
    ras_push = 1'b0;
    ras_pop  = 1'b0;
    ras_update = '0;

    for (int i = CVA6Cfg.INSTR_PER_FETCH - 1; i >= 0; i--) begin
      unique case ({is_branch[i], is_return[i], is_jump[i], is_jalr[i]})
        4'b0000: ;
        4'b0001: begin
          ras_pop = 1'b0;
          ras_push = 1'b0;
          if (CVA6Cfg.BTBEntries != 0 && btb_prediction_shifted[i].valid) begin
            predict_address = btb_prediction_shifted[i].target_address;
            cf_type[i] = ariane_pkg::JumpR;
          end
        end
        4'b0010: begin
          ras_pop = 1'b0;
          ras_push = 1'b0;
          taken_rvi_cf[i] = rvi_jump[i];
          taken_rvc_cf[i] = rvc_jump[i];
          cf_type[i] = ariane_pkg::Jump;
        end
        4'b0100: begin
          ras_pop = ras_predict.valid & instr_queue_consumed[i] & ~tc_hit_this_cycle;
          ras_push = 1'b0;
          if (ras_predict.valid) begin
            predict_address = ras_predict.ra;
            cf_type[i] = ariane_pkg::Return;
          end
        end
        4'b1000: begin
          ras_pop = 1'b0;
          ras_push = 1'b0;
          if (bht_prediction_shifted[i].valid) begin
            taken_rvi_cf[i] = rvi_branch[i] & bht_prediction_shifted[i].taken;
            taken_rvc_cf[i] = rvc_branch[i] & bht_prediction_shifted[i].taken;
          end else begin
            taken_rvi_cf[i] = rvi_branch[i] & rvi_imm[i][CVA6Cfg.VLEN-1];
            taken_rvc_cf[i] = rvc_branch[i] & rvc_imm[i][CVA6Cfg.VLEN-1];
          end
          if (taken_rvi_cf[i] || taken_rvc_cf[i]) cf_type[i] = ariane_pkg::Branch;
        end
        default: ;
      endcase

      if (is_call[i]) begin
        ras_push = instr_queue_consumed[i] & ~tc_hit_this_cycle;
        ras_update = addr[i] + (rvc_call[i] ? 2 : 4);
      end

      if (taken_rvc_cf[i] || taken_rvi_cf[i])
        predict_address = addr[i] + (taken_rvc_cf[i] ? rvc_imm[i] : rvi_imm[i]);
    end
  end

  always_comb begin
    bp_valid = 1'b0;
    for (int i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++)
      bp_valid |= ((cf_type[i] != NoCF && cf_type[i] != Return) ||
                   ((cf_type[i] == Return) && ras_predict.valid));
  end

  assign is_mispredict = resolved_branch_i.valid & resolved_branch_i.is_mispredict;

  // I$ runs continuously ? no TC gating.  TC competes same-cycle.
  assign icache_dreq_o.req     = instr_queue_ready & ~halt_frontend_i;
  assign if_ready              = icache_dreq_i.ready & instr_queue_ready & ~halt_frontend_i;
  // kill I$ stage-1 during pending capture to prevent stale
  // pipeline-drain responses from reaching icache_valid_q next cycle.
  assign icache_dreq_o.kill_s1 = is_mispredict | flush_i | replay_eff | tc_hit_this_cycle | tc_pending_capture;
  assign icache_dreq_o.kill_s2 = icache_dreq_o.kill_s1 | bp_valid;

  bht_update_t bht_update;
  btb_update_t btb_update;
  logic speculative_q, speculative_d;

  assign speculative_d      = (speculative_q && !resolved_branch_i.valid || |is_branch || |is_return || |is_jalr) && !flush_i;
  assign icache_dreq_o.spec = speculative_d;
  assign bht_update.valid          = resolved_branch_i.valid & (resolved_branch_i.cf_type == ariane_pkg::Branch);
  assign bht_update.pc             = resolved_branch_i.pc;
  assign bht_update.taken          = resolved_branch_i.is_taken;
  assign btb_update.valid          = resolved_branch_i.valid & resolved_branch_i.is_mispredict &
                                     (resolved_branch_i.cf_type == ariane_pkg::JumpR);
  assign btb_update.pc             = resolved_branch_i.pc;
  assign btb_update.target_address = resolved_branch_i.target_address;

  // -----------------------------------------------------------------------
  // TC replay classification helpers
  // -----------------------------------------------------------------------
  assign tc_trace_starts_ok = (tc_trace_length <= TRACE_LEN_WIDTH'(TRACE_LEN));

  always_comb begin
    int last_idx;
    int branch_slot_idx;
    int mapped_taken_cnt;

    tc_trace_last_is_taken_cf     = 1'b0;
    tc_trace_mapped_num_branches  = '0;
    tc_trace_mapped_num_taken     = '0;
    last_idx                      = 0;
    branch_slot_idx               = 0;
    mapped_taken_cnt              = 0;

    if (tc_trace_length != 0) begin
      last_idx = int'(tc_trace_length) - 1;

      for (int i = 0; i < TRACE_LEN; i++) begin
        if ((i < int'(tc_trace_length)) &&
            (tc_replay_cf_type(tc_trace_instructions[i]) != ariane_pkg::NoCF)) begin

          if ((i == last_idx) &&
              (branch_slot_idx < int'(tc_trace_num_branches)) &&
              tc_trace_branch_flags[branch_slot_idx]) begin
            tc_trace_last_is_taken_cf = 1'b1;
          end

          if ((branch_slot_idx < int'(tc_trace_num_branches)) &&
              tc_trace_branch_flags[branch_slot_idx]) begin
            mapped_taken_cnt++;
          end

          branch_slot_idx++;
        end
      end

      tc_trace_mapped_num_branches = BR_CNT_WIDTH'(branch_slot_idx);
      tc_trace_mapped_num_taken    = TAKEN_CNT_WIDTH'(mapped_taken_cnt);
    end
  end

  // With MAX_TAKEN=3, TAKEN_CNT_WIDTH=2 bits ? wide enough for
  // correct comparisons.  Check both branches and taken counts.
  assign tc_trace_branch_map_ok = (tc_trace_mapped_num_branches == tc_trace_num_branches) &&
                                  (tc_trace_mapped_num_taken == tc_trace_num_taken);

  always_comb begin
    tc_trace_has_call = 1'b0;
    tc_trace_has_indirect = 1'b0;
    for (int i = 0; i < TRACE_LEN; i++) begin
      if (i < int'(tc_trace_length)) begin
        if (tc_replay_is_call(tc_trace_instructions[i]))
          tc_trace_has_call = 1'b1;
        if ((tc_replay_cf_type(tc_trace_instructions[i]) == ariane_pkg::JumpR) ||
            (tc_replay_cf_type(tc_trace_instructions[i]) == ariane_pkg::Return))
          tc_trace_has_indirect = 1'b1;
      end
    end
  end

  // -----------------------------------------------------------------------
  // Same-cycle TC/I$ MUX ? no pending/replay model.
  // When tc_hit_this_cycle fires, trace data replaces I$ data for the IQ
  // push *in the same cycle*.  No extra latency on miss.
  // -----------------------------------------------------------------------

  // Combinational feed signals: format trace output for IQ push.
  // When tc_pending_use fires, feed from the buffered registers.
  //      When tc_direct_hit fires, feed from live SRAM trace outputs.
  logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]   tc_feed_src_instructions;
  logic [TRACE_LEN_WIDTH-1:0]              tc_feed_src_length;
  logic [TRACE_LEN-1:0][PC_WIDTH-1:0]      tc_feed_src_pcs;
  logic [CHUNKS_PER_TRACE-1:0]             tc_feed_src_branch_flags;
  logic [BR_CNT_WIDTH-1:0]                 tc_feed_src_num_branches;
  logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]      tc_feed_src_taken_targets;
  logic [TAKEN_CNT_WIDTH-1:0]              tc_feed_src_num_taken;
  logic [PC_WIDTH-1:0]                     tc_feed_src_next_pc;

  always_comb begin
    if (tc_pending_use) begin
      tc_feed_src_instructions = tc_pending_instructions_q;
      tc_feed_src_length       = tc_pending_length_q;
      tc_feed_src_pcs          = tc_pending_pcs_q;
      tc_feed_src_branch_flags = tc_pending_branch_flags_q;
      tc_feed_src_num_branches = tc_pending_num_branches_q;
      tc_feed_src_taken_targets = tc_pending_taken_targets_q;
      tc_feed_src_num_taken    = tc_pending_num_taken_q;
      tc_feed_src_next_pc      = tc_pending_next_pc_q;
    end else begin
      tc_feed_src_instructions = tc_trace_instructions;
      tc_feed_src_length       = tc_trace_length;
      tc_feed_src_pcs          = tc_trace_pcs;
      tc_feed_src_branch_flags = tc_trace_branch_flags;
      tc_feed_src_num_branches = tc_trace_num_branches;
      tc_feed_src_taken_targets = tc_trace_taken_targets;
      tc_feed_src_num_taken    = tc_trace_num_taken;
      tc_feed_src_next_pc      = tc_trace_next_pc;
    end
  end

  always_comb begin
    automatic cf_t feed_cf;
    automatic int  br_idx;
    automatic int  taken_idx;

    tc_feed_valid        = '0;
    tc_feed_instr        = '0;
    tc_feed_addr         = '0;
    tc_feed_cf           = '{default: ariane_pkg::NoCF};
    tc_feed_predict_addr = '0;

    br_idx    = 0;
    taken_idx = 0;

    for (int p = 0; p < CVA6Cfg.INSTR_PER_FETCH; p++) begin
      if (p < int'(tc_feed_src_length)) begin
        tc_feed_valid[p] = 1'b1;
        tc_feed_instr[p] = tc_feed_src_instructions[p];
        tc_feed_addr[p]  = tc_feed_src_pcs[p][CVA6Cfg.VLEN-1:0];

        // Determine cf_type from trace instruction + branch_flags.
        feed_cf = ariane_pkg::NoCF;
        if (tc_replay_cf_type(tc_feed_src_instructions[p]) != ariane_pkg::NoCF) begin
          if ((br_idx < int'(tc_feed_src_num_branches)) &&
              tc_feed_src_branch_flags[br_idx]) begin
            feed_cf = tc_replay_cf_type(tc_feed_src_instructions[p]);
          end
          br_idx++;
        end
        tc_feed_cf[p] = feed_cf;

        // Predict address: next instruction PC, or trace exit for the last.
        if (feed_cf != ariane_pkg::NoCF) begin
          // Taken CF: predict_addr is next PC in trace or trace exit.
          if (p + 1 < int'(tc_feed_src_length))
            tc_feed_predict_addr[p] = tc_feed_src_pcs[p + 1][CVA6Cfg.VLEN-1:0];
          else
            tc_feed_predict_addr[p] = tc_feed_src_next_pc[CVA6Cfg.VLEN-1:0];
        end
      end
    end
  end

  // -----------------------------------------------------------------------
  // TC-driven RAS synchronization.
  // When a trace containing a return/call is replayed, the RAS must be
  // updated to stay consistent.  The normal I$-path RAS push/pop are
  // suppressed on tc_hit_this_cycle ? these signals provide the TC path.
  // -----------------------------------------------------------------------
  logic                          tc_ras_pop;
  logic                          tc_ras_push;
  logic [CVA6Cfg.VLEN-1:0]      tc_ras_update;

  always_comb begin
    tc_ras_pop    = 1'b0;
    tc_ras_push   = 1'b0;
    tc_ras_update = '0;

    if (tc_hit_this_cycle) begin
      for (int p = 0; p < CVA6Cfg.INSTR_PER_FETCH; p++) begin
        if (p < int'(tc_feed_src_length)) begin
          // Return in trace ? pop RAS (keep stack in sync)
          if (tc_replay_cf_type(tc_feed_src_instructions[p]) == ariane_pkg::Return)
            tc_ras_pop = 1'b1;

          // Call in trace ? push return address onto RAS
          if (tc_replay_is_call(tc_feed_src_instructions[p])) begin
            tc_ras_push   = 1'b1;
            tc_ras_update = tc_feed_src_pcs[p][CVA6Cfg.VLEN-1:0]
                          + ((tc_feed_src_instructions[p][1:0] != 2'b11) ? CVA6Cfg.VLEN'(2) : CVA6Cfg.VLEN'(4));
          end
        end
      end
    end
  end

  // -----------------------------------------------------------------------
  // MUX: TC hit vs normal I-cache.  On tc_hit_this_cycle, TC trace data
  // replaces the I$ scan output going into the IQ.
  // -----------------------------------------------------------------------
  assign instr_to_iq            = tc_hit_this_cycle ? tc_feed_instr : instr;
  assign addr_to_iq             = tc_hit_this_cycle ? tc_feed_addr  : addr;
  // When the pending buffer is active (capture or holding),
  // suppress I$ data to the IQ.  This prevents the I$ scan from
  // partially inserting instructions into non-full IQ lanes that
  // would duplicate the captured trace when tc_pending_use fires.
  assign valid_to_iq            = tc_hit_this_cycle    ? tc_feed_valid :
                                  tc_suppress_icache   ? '0 :
                                                         instruction_valid;
  assign cf_type_to_iq          = tc_hit_this_cycle ? tc_feed_cf : cf_type;
  assign exception_to_iq        = tc_hit_this_cycle ? ariane_pkg::FE_NONE : icache_ex_valid_q;
  assign exception_addr_to_iq   = tc_hit_this_cycle ? '0 : icache_vaddr_q;
  assign exception_gpaddr_to_iq = tc_hit_this_cycle ? '0 : icache_gpaddr_q;
  assign exception_tinst_to_iq  = tc_hit_this_cycle ? '0 : icache_tinst_q;
  assign exception_gva_to_iq    = tc_hit_this_cycle ? 1'b0 : icache_gva_q;

  for (genvar gi = 0; gi < CVA6Cfg.INSTR_PER_FETCH; gi++) begin : gen_predict_addr_mux
    assign predict_addr_to_iq[gi] = tc_hit_this_cycle ? tc_feed_predict_addr[gi] : predict_address;
  end

  // GHR snapshot ? same for all slots in the window
  // use captured GHR when replaying from pending buffer
  assign tc_ghr_to_iq = tc_pending_use ? tc_pending_ghr_q : tc_ghr_from_tc;

  // During TC hit, supply the trace's own branch outcomes as branch_flags.
  assign branch_flags_to_iq = tc_hit_this_cycle ?
      tc_feed_src_branch_flags[CHUNKS_PER_TRACE-1:0] : tc_branch_predictions;

  // tc_active_branch_flags for restore on mispredict.
  always_comb begin
    tc_active_branch_flags = '0;
    if (tc_hit_this_cycle) begin
      tc_active_branch_flags = tc_feed_src_branch_flags[CHUNKS_PER_TRACE-1:0];
    end
  end

  assign restored_branch_flags = is_mispredict ? iq_branch_flags_out : tc_active_branch_flags;

  // -----------------------------------------------------------------------
  // NPC select
  // -----------------------------------------------------------------------
  always_comb begin : npc_select
    automatic logic [CVA6Cfg.VLEN-1:0] fetch_address;

    if (npc_rst_load_q) begin
      npc_d = boot_addr_i;
      fetch_address = boot_addr_i;
    end else begin
      fetch_address = npc_q;
      npc_d = npc_q;
    end

    if (bp_valid) begin
      fetch_address = predict_address;
      npc_d = predict_address;
    end

    if (if_ready)
      npc_d = {fetch_address[CVA6Cfg.VLEN-1:CVA6Cfg.FETCH_ALIGN_BITS] + 1,
               {CVA6Cfg.FETCH_ALIGN_BITS{1'b0}}};

    // [BHT scan cancel NPC redirect removed ? tc_scan_cancel is always 0]

    // Steer NPC to trace exit on same-cycle TC hit (direct or pending).
    if (tc_hit_this_cycle)
      npc_d = tc_feed_src_next_pc[CVA6Cfg.VLEN-1:0];

    if (replay_eff)      npc_d = replay_addr;
    if (is_mispredict)   npc_d = resolved_branch_i.target_address;
    if (eret_i)          npc_d = epc_i;
    if (ex_valid_i)      npc_d = trap_vector_base_i;
    if (set_pc_commit_i) npc_d = pc_commit_i + (halt_i ? '0 : {{CVA6Cfg.VLEN - 3{1'b0}}, 3'b100});

    if (CVA6Cfg.DebugEn && set_debug_pc_i)
      npc_d = CVA6Cfg.DmBaseAddress[CVA6Cfg.VLEN-1:0] + CVA6Cfg.HaltAddress[CVA6Cfg.VLEN-1:0];

    icache_dreq_o.vaddr = fetch_address;
  end

  logic [CVA6Cfg.FETCH_WIDTH-1:0] icache_data;
  assign icache_data = icache_dreq_i.data >> {shamt, 4'b0};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      npc_rst_load_q    <= 1'b1;
      npc_q             <= '0;
      speculative_q     <= '0;
      icache_data_q     <= '0;
      icache_valid_q    <= 1'b0;
      icache_vaddr_q    <= '0;
      icache_gpaddr_q   <= '0;
      icache_tinst_q    <= '0;
      icache_gva_q      <= 1'b0;
      icache_ex_valid_q <= ariane_pkg::FE_NONE;
      btb_q             <= '0;
      bht_q             <= '0;
    end else begin
      npc_rst_load_q <= 1'b0;
      npc_q          <= npc_d;
      speculative_q  <= speculative_d;
      // Suppress stale I$ response registration while the
      // pending buffer owns a trace.  Without this, a pipeline-drain
      // response could arrive, be registered, run through instr_realign
      // next cycle, and partially leak into the IQ via non-full lanes
      // (since valid_to_iq is gated, AND bp_valid from the stale scan
      // would spuriously steer NPC).
      icache_valid_q <= icache_dreq_i.valid & ~tc_suppress_icache;

      if (icache_dreq_i.valid) begin
        icache_data_q  <= icache_data;
        icache_vaddr_q <= icache_dreq_i.vaddr;

        if (CVA6Cfg.RVH) begin
          icache_gpaddr_q <= icache_dreq_i.ex.tval2[CVA6Cfg.GPLEN-1:0];
          icache_tinst_q  <= icache_dreq_i.ex.tinst;
          icache_gva_q    <= icache_dreq_i.ex.gva;
        end else begin
          icache_gpaddr_q <= '0;
          icache_tinst_q  <= '0;
          icache_gva_q    <= 1'b0;
        end

        if (CVA6Cfg.MmuPresent && icache_dreq_i.ex.cause == riscv::INSTR_GUEST_PAGE_FAULT)
          icache_ex_valid_q <= ariane_pkg::FE_INSTR_GUEST_PAGE_FAULT;
        else if (CVA6Cfg.MmuPresent && icache_dreq_i.ex.cause == riscv::INSTR_PAGE_FAULT)
          icache_ex_valid_q <= ariane_pkg::FE_INSTR_PAGE_FAULT;
        else if (icache_dreq_i.ex.cause == riscv::INSTR_ACCESS_FAULT)
          icache_ex_valid_q <= ariane_pkg::FE_INSTR_ACCESS_FAULT;
        else
          icache_ex_valid_q <= ariane_pkg::FE_NONE;

        btb_q <= btb_prediction[CVA6Cfg.INSTR_PER_FETCH-1];
        bht_q <= bht_prediction[CVA6Cfg.INSTR_PER_FETCH-1];
      end
    end
  end

  // -----------------------------------------------------------------------
  // Pending trace buffer ? 1-entry register
  // Captures a TC hit when IQ is not ready.  Used next cycle if IQ becomes
  // ready and the path context is still valid (no redirect, no flush).
  // -----------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_pending_valid_q          <= 1'b0;
      tc_pending_instructions_q   <= '0;
      tc_pending_length_q         <= '0;
      tc_pending_next_pc_q        <= '0;
      tc_pending_pcs_q            <= '0;
      tc_pending_branch_flags_q   <= '0;
      tc_pending_num_branches_q   <= '0;
      tc_pending_taken_targets_q  <= '0;
      tc_pending_num_taken_q      <= '0;
      tc_pending_base_pc_q        <= '0;
      tc_pending_ghr_q            <= '0;
    end else if (tc_pending_invalidate || tc_pending_use) begin
      // Invalidate on any path-breaking event, or after successful use.
      tc_pending_valid_q <= 1'b0;
    end else if (tc_pending_capture) begin
      // Capture the current TC hit into the pending buffer.
      tc_pending_valid_q          <= 1'b1;
      tc_pending_instructions_q   <= tc_trace_instructions;
      tc_pending_length_q         <= tc_trace_length;
      tc_pending_next_pc_q        <= tc_trace_next_pc;
      tc_pending_pcs_q            <= tc_trace_pcs;
      tc_pending_branch_flags_q   <= tc_trace_branch_flags;
      tc_pending_num_branches_q   <= tc_trace_num_branches;
      tc_pending_taken_targets_q  <= tc_trace_taken_targets;
      tc_pending_num_taken_q      <= tc_trace_num_taken;
      tc_pending_base_pc_q        <= tc_trace_pcs[0];
      tc_pending_ghr_q            <= tc_ghr_from_tc;  // snapshot GHR at capture
    end
  end

  if (CVA6Cfg.RASDepth == 0) begin
    assign ras_predict = '0;
  end else begin : ras_gen
    ras #(
        .CVA6Cfg(CVA6Cfg),
        .ras_t  (ras_t),
        .DEPTH  (CVA6Cfg.RASDepth)
    ) i_ras (
        .clk_i,
        .rst_ni,
        .flush_bp_i(flush_bp_i),
        .push_i(ras_push | tc_ras_push),         // include TC-driven push
        .pop_i(ras_pop | tc_ras_pop),             // include TC-driven pop
        .data_i(tc_ras_push ? tc_ras_update : ras_update),  // TC call return addr
        .data_o(ras_predict)
    );
  end

  assign vpc_btb = (CVA6Cfg.FpgaEn) ? icache_dreq_i.vaddr : icache_vaddr_q;
  // Restore original vpc_bht ? never MUX the primary BHT port.
  assign vpc_bht = (CVA6Cfg.FpgaEn && CVA6Cfg.FpgaAlteraEn && icache_dreq_i.valid) ? icache_dreq_i.vaddr : icache_vaddr_q;

  // Second BHT read port for TC branch scan (dedicated, no MUX conflicts).
  ariane_pkg::bht_prediction_t [CVA6Cfg.INSTR_PER_FETCH-1:0] tc_bht_prediction;

  // BHT scan removed ? no pre-replay branch cancellation.
  // Second BHT port tied to npc_q (unused result, kept for interface).
  logic [CVA6Cfg.VLEN-1:0] tc_scan_vpc;
  assign tc_scan_vpc = npc_q;

  if (CVA6Cfg.BTBEntries == 0) begin
    assign btb_prediction = '0;
  end else begin : btb_gen
    btb #(
        .CVA6Cfg         (CVA6Cfg),
        .btb_update_t    (btb_update_t),
        .btb_prediction_t(btb_prediction_t),
        .NR_ENTRIES      (CVA6Cfg.BTBEntries)
    ) i_btb (
        .clk_i,
        .rst_ni,
        .flush_bp_i(flush_bp_i),
        .debug_mode_i,
        .vpc_i(vpc_btb),
        .btb_update_i(btb_update),
        .btb_prediction_o(btb_prediction)
    );
  end

  if (CVA6Cfg.BHTEntries == 0) begin
    assign bht_prediction = '0;
    assign tc_bht_prediction = '0;
  end else if (CVA6Cfg.BPType == config_pkg::BHT) begin : bht_gen
    bht #(
        .CVA6Cfg     (CVA6Cfg),
        .bht_update_t(bht_update_t),
        .NR_ENTRIES  (CVA6Cfg.BHTEntries)
    ) i_bht (
        .clk_i,
        .rst_ni,
        .flush_bp_i(flush_bp_i),
        .debug_mode_i,
        .vpc_i(vpc_bht),
        .bht_update_i(bht_update),
        .bht_prediction_o(bht_prediction),
        .tc_vpc_i(tc_scan_vpc),
        .tc_bht_prediction_o(tc_bht_prediction)
    );
  end else if (CVA6Cfg.BPType == config_pkg::PH_BHT) begin : bht2lvl_gen
    bht2lvl #(
        .CVA6Cfg     (CVA6Cfg),
        .bht_update_t(bht_update_t)
    ) i_bht (
        .clk_i,
        .rst_ni,
        .flush_i(flush_bp_i),
        .vpc_i(icache_vaddr_q),
        .bht_update_i(bht_update),
        .bht_prediction_o(bht_prediction)
    );
    assign tc_bht_prediction = '0; // bht2lvl has no TC port; disable exit cancel
  end

  for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_instr_scan
    instr_scan #(.CVA6Cfg(CVA6Cfg)) i_instr_scan (
        .instr_i(instr[i]),
        .rvi_return_o(rvi_return[i]),
        .rvi_call_o(rvi_call[i]),
        .rvi_branch_o(rvi_branch[i]),
        .rvi_jalr_o(rvi_jalr[i]),
        .rvi_jump_o(rvi_jump[i]),
        .rvi_imm_o(rvi_imm[i]),
        .rvc_branch_o(rvc_branch[i]),
        .rvc_jump_o(rvc_jump[i]),
        .rvc_jr_o(rvc_jr[i]),
        .rvc_return_o(rvc_return[i]),
        .rvc_jalr_o(rvc_jalr[i]),
        .rvc_call_o(rvc_call[i]),
        .rvc_imm_o(rvc_imm[i])
    );
  end

  instr_queue #(
      .CVA6Cfg      (CVA6Cfg),
      .BRANCH_FLAGS_W(CHUNKS_PER_TRACE),
      .fetch_entry_t(fetch_entry_t)
  ) i_instr_queue (
      .clk_i,
      .rst_ni,
      // On TC hit, trace data replaces I$ data in the IQ MUX.
      // On miss, I$ data flows through unchanged.
      .flush_i            (flush_i || is_mispredict || set_pc_commit_i || eret_i ||
                           ex_valid_i),
      .instr_i            (instr_to_iq),
      .addr_i             (addr_to_iq),
      .exception_i        (exception_to_iq),
      .exception_addr_i   (exception_addr_to_iq),
      .exception_gpaddr_i (exception_gpaddr_to_iq),
      .exception_tinst_i  (exception_tinst_to_iq),
      .exception_gva_i    (exception_gva_to_iq),
      .predict_address_i  (predict_addr_to_iq),
      .branch_flags_i     (branch_flags_to_iq),
      .cf_type_i          (cf_type_to_iq),
      .tc_ghr_i           (tc_ghr_to_iq),
      .valid_i            (valid_to_iq),
      .consumed_o         (instr_queue_consumed),
      .ready_o            (instr_queue_ready),
      .empty_o            (instr_queue_empty),
      .replay_o           (replay),
      .replay_addr_o      (replay_addr),
      .branch_flags_o     (iq_branch_flags_out),
      .fetch_entry_o      (fetch_entry_o),
      .fetch_entry_valid_o(fetch_entry_valid_o),
      .fetch_entry_ready_i(fetch_entry_ready_i),
      .tc_feeding_i       (tc_hit_this_cycle),  // same-cycle TC feed
      .tc_suppress_port1_i(tc_suppress_port1),
      .reseed_pc_i        (tc_hit_this_cycle)
  );

  // -----------------------------------------------------------------------
  // Suppress IQ replay during TC hit (NPC is being re-steered).
  // -----------------------------------------------------------------------
  assign replay_eff = replay & ~tc_hit_this_cycle;

  // -----------------------------------------------------------------------
  // Trace cache signals for recording and lookup
  // -----------------------------------------------------------------------
  logic [SLOTS_PER_CYCLE-1:0]               tc_instr_valid;
  logic [SLOTS_PER_CYCLE-1:0][31:0]         tc_instr;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] tc_pc;
  logic [SLOTS_PER_CYCLE-1:0]               tc_is_branch;
  logic [SLOTS_PER_CYCLE-1:0]               tc_raw_taken;
  logic [SLOTS_PER_CYCLE-1:0]               tc_taken;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] tc_target;
  logic [BR_CNT_WIDTH-1:0]                  tc_lookup_num_branches;
  logic                                     tc_window_eligible;
  logic                                     tc_record_enable;
  logic                                     tc_builder_enable;

  assign tc_window_eligible = !halt_i && !halt_frontend_i && !debug_mode_i;
  assign tc_record_enable   = tc_window_eligible && addr[0][31];
  // Builder is suppressed on TC hit to avoid recording the replacement window.
  assign tc_builder_enable  = tc_record_enable && !tc_hit_this_cycle;

  for (genvar i = 0; i < SLOTS_PER_CYCLE; i++) begin : gen_tc_signals
    assign tc_instr_valid[i] = instruction_valid[i] & ~flush_i & tc_builder_enable;
    assign tc_pc[i]          = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, addr[i]};
    assign tc_instr[i]       = instr[i];
    assign tc_is_branch[i]   = is_branch[i] | is_jump[i] | is_jalr[i] | is_return[i] | is_call[i];
    // tc_raw_taken: uses instruction_valid (not tc_instr_valid) to avoid
    // circular dependency through tc_hit_this_cycle.
    assign tc_raw_taken[i]   = ((taken_rvi_cf[i] | taken_rvc_cf[i]) | is_jump[i] | is_call[i] |
                                (is_jalr[i] & btb_prediction_shifted[i].valid) |
                                (is_return[i] & ras_predict.valid)) & instruction_valid[i] & ~flush_i;

    always_comb begin
      if (taken_rvi_cf[i])
        tc_target[i] = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, (addr[i] + rvi_imm[i])};
      else if (taken_rvc_cf[i])
        tc_target[i] = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, (addr[i] + rvc_imm[i])};
      else if (is_return[i] && ras_predict.valid)
        tc_target[i] = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, ras_predict.ra};
      else if (is_jalr[i] && btb_prediction_shifted[i].valid)
        tc_target[i] = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, btb_prediction_shifted[i].target_address};
      else
        tc_target[i] = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, (addr[i] + (instr[i][1:0] != 2'b11 ? 2 : 4))};
    end
  end

  // tc_valid_mask: valid instruction slots up to and including the first
  // predicted-taken branch.  This replaces instr_queue_consumed for
  // branch_predictions computation (available same cycle, no IQ delay).
  always_comb begin
    tc_valid_mask = '0;
    for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
      if (instruction_valid[i] && !flush_i) begin
        tc_valid_mask[i] = 1'b1;
        if (tc_raw_taken[i]) break;
      end
    end
  end

  always_comb begin
    logic found_taken;
    found_taken = 1'b0;

    for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
      if (found_taken) begin
        tc_taken[i] = 1'b0;
      end else begin
        tc_taken[i] = tc_raw_taken[i];
        if (tc_raw_taken[i]) found_taken = 1'b1;
      end
    end
  end

  // -----------------------------------------------------------------------
  // Branch predictions for TC tag matching ? computed from the live
  // instruction scan (tc_valid_mask), NOT from instr_queue_consumed.
  // -----------------------------------------------------------------------
  logic tc_window_has_taken_pred; // true when ?1 branch is predicted-taken
  always_comb begin
    integer br_idx;
    automatic logic seen_taken;
    automatic logic slot_pred;
    tc_branch_predictions  = '0;
    tc_lookup_num_branches = '0;
    br_idx = 0;

    seen_taken = 1'b0;

    for (int i = 0; i < SLOTS_PER_CYCLE && br_idx < CHUNKS_PER_TRACE; i++) begin
      if (tc_valid_mask[i] && tc_is_branch[i]) begin
        tc_lookup_num_branches = tc_lookup_num_branches + BR_CNT_WIDTH'(1);

        if (seen_taken) begin
          slot_pred = 1'b0;
        end else if (is_jump[i] || is_call[i])
          slot_pred = 1'b1;
        else if (is_return[i])
          slot_pred = ras_predict.valid;
        else if (is_jalr[i])
          slot_pred = btb_prediction_shifted[i].valid;
        else
          slot_pred = bht_prediction_shifted[i].valid ?
                      bht_prediction_shifted[i].taken :
                      (rvi_branch[i] ? rvi_imm[i][CVA6Cfg.VLEN-1] :
                                       rvc_imm[i][CVA6Cfg.VLEN-1]);

        tc_branch_predictions[br_idx] = slot_pred;
        if (slot_pred) seen_taken = 1'b1;
        br_idx++;
      end
    end
    tc_window_has_taken_pred = seen_taken;
  end

  // TC SRAM lookup.  Read fires this cycle, data lands next cycle and is
  // tag-compared against that window's live branch predictions.
  logic                tc_lookup_valid;
  logic [PC_WIDTH-1:0] tc_lookup_pc;
  // Unaligned windows need the read fired one cycle early.  instr_realign
  // knows this cycle that the next one will serve an unaligned 32-bit
  // instruction straddling two blocks, so we index on the unaligned PC now
  // and the result arrives exactly when that instruction reaches the IQ.
  // Overriding the aligned lookup is safe -- its result would have landed on
  // the unaligned cycle and described the wrong window anyway.
  logic                    tc_early_unaligned_lookup;
  assign tc_early_unaligned_lookup = next_serving_unaligned && icache_valid_q &&
                                     tc_window_eligible && !flush_i;

  assign tc_lookup_valid = tc_early_unaligned_lookup ||
                           (icache_dreq_i.valid && !flush_i && tc_window_eligible);

  assign tc_lookup_pc    = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}},
                            tc_early_unaligned_lookup ? next_unaligned_address
                                                      : icache_dreq_i.vaddr};

  // 1-cycle delayed lookup PC, aligned with tc_lookup_result_valid. Must
  // mirror the actual key submitted to the SRAM (not icache_dreq_i.vaddr,
  // which diverges from tc_lookup_pc on early-unaligned lookups). Used by
  // TC-PCMISS / HOTPC observability in this file.
  logic [CVA6Cfg.VLEN-1:0] tc_lookup_pc_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)               tc_lookup_pc_q <= '0;
    else if (tc_lookup_valid)  tc_lookup_pc_q <= tc_lookup_pc[CVA6Cfg.VLEN-1:0];
  end

  trace_cache_top #(
      .MaxTraceInstr(MAX_TRACE_INSTR)
  ) i_trace_cache_top (
      .clk_i,
      .rst_ni,
      .instr_valid_i                  (tc_instr_valid),
      .instr_i                        (tc_instr),
      .pc_i                           (tc_pc),
      .is_branch_i                    (tc_is_branch),
      .branch_taken_i                 (tc_taken),
      .branch_target_i                (tc_target),
      .serving_unaligned_i            (serving_unaligned),
      .flush_i                        (flush_i || is_mispredict),
      .tc_replay_fired_i              (tc_hit_this_cycle),
      .instr_queue_ready_i            (instr_queue_ready && tc_builder_enable),
      .instr_queue_consumed_i         (tc_builder_enable ? instr_queue_consumed : '0),
      .branch_predictions_i           (tc_branch_predictions),
      .lookup_num_branches_i          (tc_lookup_num_branches),
      .window_has_taken_pred_i        (tc_window_has_taken_pred),
      .resolved_branch_valid_i        (resolved_branch_i.valid),
      .resolved_branch_pc_i           (resolved_branch_i.pc),
      .resolved_branch_is_taken_i     (resolved_branch_i.is_taken),
      .resolved_branch_is_mispredict_i(resolved_branch_i.is_mispredict),
      .resolved_branch_ghr_i          (resolved_branch_i.tc_ghr),
      .ghr_o                          (tc_ghr_from_tc),
      .lookup_valid_i                 (tc_lookup_valid),
      .lookup_pc_i                    (tc_lookup_pc),
      .lookup_predicts_unaligned_i    (tc_early_unaligned_lookup),
      .trace_hit_o                    (tc_trace_hit),
      .trace_instructions_o           (tc_trace_instructions),
      .trace_length_o                 (tc_trace_length),
      .trace_next_pc_o                (tc_trace_next_pc),
      .trace_chunks_o                 (tc_trace_chunks),
      .trace_valid_chunks_o           (tc_trace_valid_chunks),
      .trace_pcs_o                    (tc_trace_pcs),
      .trace_branch_flags_o           (tc_trace_branch_flags),
      .trace_num_branches_o           (tc_trace_num_branches),
      .trace_taken_targets_o          (tc_trace_taken_targets),
      .trace_num_taken_o              (tc_trace_num_taken),
      .trace_used_o                   (tc_trace_used),
      .lookup_result_valid_o          (tc_lookup_result_valid),
      .miss_reason_empty_o            (tc_miss_reason_empty),
      .miss_reason_pc_o               (tc_miss_reason_pc),
      .miss_reason_path_o             (tc_miss_reason_path),
      .tc_miss_total_o                (tc_miss_total),
      .tc_miss_empty_o                (tc_miss_empty),
      .tc_miss_pc_o                   (tc_miss_pc),
      .tc_miss_path_o                 (tc_miss_path),
      .mark_used_i                    (tc_hit_this_cycle)
  );

  logic tc_trace_policy_ok;
  // A hit fires kill_s1, one bubble more than an ordinary BTB redirect.  A
  // length-2 trace delivers exactly one extra instruction, so it breaks even
  // at best; 3 is the shortest trace worth the redirect.  Tried 4 -- no gain.
  localparam int unsigned TC_MIN_ACCEPT_LEN = 3;

`ifndef SYNTHESIS
  initial begin
    tc_disable_replay_q = $test$plusargs("TC_DISABLE_REPLAY");
    tc_no_iq_gate_q     = $test$plusargs("TC_NO_IQ_GATE");
    tc_no_path_gate_q   = $test$plusargs("TC_NO_PATH_GATE");  // V101: A/B the path gate
    tc_loose_lo_gate_q  = $test$plusargs("TC_LOOSE_LO_GATE"); // V102: restore V97 lo-value gate
  end

  initial begin
    $display("[TC-RUN-MARKER] TC_V74_DEDUP_GUARD");
    $display("[TC-POLICY] min_accept_len=%0d", TC_MIN_ACCEPT_LEN);
    $display("[TC-POLICY] full_path_validation=%0d (V101)", !tc_no_path_gate_q);
    $display("[TC-POLICY] lo_value_requires_iq_empty=%0d (V102)", !tc_loose_lo_gate_q);
    if (tc_disable_replay_q)
      $display("[TC-RUN-MARKER] TC_SWITCH_DISABLE_REPLAY_ACTIVE");
    if (tc_no_iq_gate_q)
      $display("[TC-RUN-MARKER] TC_NO_IQ_GATE_ACTIVE");
    if (tc_no_path_gate_q)
      $display("[TC-RUN-MARKER] TC_NO_PATH_GATE_ACTIVE");
  end
`else
  assign tc_disable_replay_q = 1'b0;
  assign tc_no_iq_gate_q     = 1'b0;
  assign tc_no_path_gate_q   = 1'b0;
  assign tc_loose_lo_gate_q  = 1'b0;
`endif

  assign tc_active_hit = tc_lookup_result_valid && tc_trace_hit &&
                         (tc_trace_length != '0) && !flush_i && !is_mispredict;

  // Only replay traces whose every branch was checked against the predictor.
  //
  // The tag covers the trigger window alone: the builder freezes trig_cnt /
  // trig_flags once it leaves TB_IDLE, so the tag holds the first window's
  // branch count while the payload holds the whole trace's.  A hit has
  // already forced stored_tag.num_branches == tc_lookup_num_branches, so the
  // two counts being equal means no branch escaped the compare.
  //
  // Branches past the first window are otherwise replayed as fact.  On
  // CoreMark they mispredicted 1.31% of the time (+8724 mispredicts, +2.32%
  // runtime) while 98% of replays landed in a non-empty IQ and bought
  // nothing.  Lookup-side policy only -- trace format and builder untouched.
  // +TC_NO_PATH_GATE disables it for A/B runs.
  logic tc_trace_path_validated;
  assign tc_trace_path_validated = (tc_trace_num_branches == tc_lookup_num_branches);

  // Replay anything that is self-consistent, long enough to pay for its
  // bubble, and fully path-checked.
  assign tc_trace_policy_ok = tc_trace_starts_ok &&
                              tc_trace_branch_map_ok &&
                              (tc_trace_length >= TRACE_LEN_WIDTH'(TC_MIN_ACCEPT_LEN)) &&
                              (tc_trace_path_validated || tc_no_path_gate_q) &&
                              !tc_disable_replay_q;

  // Drop the pending trace on anything that breaks the path it assumed.
  // The replay term uses ~tc_pending_valid_q rather than ~tc_hit_this_cycle:
  // the latter closes a comb loop through tc_pending_use -> tc_feeding_i ->
  // IQ replay_o -> back here.  Equivalent because replay is always 0 while a
  // pending trace is valid (tc_suppress_icache blocks the IQ push).
  assign tc_pending_invalidate = flush_i || is_mispredict || ex_valid_i ||
                                 eret_i || set_pc_commit_i ||
                                 (replay & ~tc_pending_valid_q) ||
                                 (CVA6Cfg.DebugEn && set_debug_pc_i);

  // Two-tier replay gate.  A hit costs one kill_s1 bubble, so it has to buy
  // back something the icache + BHT/BTB cannot already do:
  //   multi-taken: each extra taken branch saves a redirect the icache
  //                pipeline cannot avoid.  Replay whenever the IQ has room.
  //   single-taken: the icache already delivers a one-taken window in a
  //                cycle, so replay only pays while the frontend starves.
  //                Require the IQ to be empty.
  logic tc_trace_is_hi_value;
  logic tc_pending_is_hi_value;
  assign tc_trace_is_hi_value   = (tc_trace_num_taken     >= TAKEN_CNT_WIDTH'(2));
  assign tc_pending_is_hi_value = (tc_pending_num_taken_q >= TAKEN_CNT_WIDTH'(2));

  logic tc_value_gate_direct;
  logic tc_value_gate_pending_use;
  // briefly let single-taken traces through on IQ-ready instead of
  // IQ-empty.  On CoreMark that fired 98% of replays into a non-empty queue
  // for +31.8k cycles of pure redirect overhead at unchanged misprediction
  // count, so the strict gate is back.  Side effect: single-taken traces can
  // no longer reach the pending buffer (capture needs !ready, and empty
  // implies ready) and fall through to rejected_not_ready -- intended.
  // v91_iq_gate_suppress counts what this costs.  +TC_LOOSE_LO_GATE reverts.
  assign tc_value_gate_direct      = tc_no_iq_gate_q || tc_trace_is_hi_value
                                     || instr_queue_empty
                                     || (tc_loose_lo_gate_q && instr_queue_ready);
  assign tc_value_gate_pending_use = tc_no_iq_gate_q || tc_pending_is_hi_value
                                     || instr_queue_empty
                                     || (tc_loose_lo_gate_q && instr_queue_ready);

  // Capture a TC hit into the pending buffer when IQ is not ready.
  // Do not overwrite an existing valid pending trace (it takes priority).
  // gate is now value-aware ? hi-value hits get captured even when
  // the IQ has work backed up, so we don't lose multi-taken bubble savings.
  assign tc_pending_capture = tc_active_hit && tc_trace_policy_ok &&
                              !tc_trace_used && !instr_queue_ready &&
                              tc_value_gate_direct &&
                              !tc_pending_invalidate &&
                              !tc_pending_valid_q;

  // Suppress I$ data into IQ for the duration of pending buffer
  // ownership.  tc_pending_capture covers the capture cycle itself;
  // tc_pending_valid_q covers all subsequent cycles until use/invalidation.
  // No combinational loop: instr_queue_ready (= IQ ready_o) is based
  // solely on registered FIFO state, so tc_pending_capture does not
  // depend on valid_to_iq ? push ? FIFO full ? ready.
  assign tc_suppress_icache = tc_pending_capture || tc_pending_valid_q;

  // Use the pending buffer when it is valid, IQ is ready, and no
  // invalidation event has occurred.
  // value-aware gate (hi-value bypasses IQ-empty).
  assign tc_pending_use = tc_pending_valid_q && instr_queue_ready &&
                          tc_value_gate_pending_use &&
                          !tc_pending_invalidate;

  // tc_hit_this_cycle ? fires on direct same-cycle hit OR pending use.
  // replay gate is now value-aware (see tc_value_gate_direct above).
  logic tc_replay_iq_gate;
  assign tc_replay_iq_gate = tc_value_gate_direct;

  logic tc_direct_hit;
  assign tc_direct_hit = tc_active_hit &&
                         tc_trace_policy_ok &&
                         !tc_trace_used &&
                         instr_queue_ready &&
                         tc_replay_iq_gate &&
                         !tc_pending_use;

  assign tc_hit_this_cycle = tc_direct_hit || tc_pending_use;

  assign tc_suppress_port1 = 1'b0;
  assign tc_active_use = 1'b0;

// pragma translate_off
  longint unsigned tc_total_cycles_q;
  int unsigned tc_dbg_lookup_count_q;
  int unsigned tc_dbg_hit_count_q;
  int unsigned tc_dbg_accept_count_q;
  int unsigned tc_dbg_reject_not_ready_q;
  int unsigned tc_dbg_reject_used_q;
  int unsigned tc_dbg_reject_indirect_q;
  int unsigned tc_dbg_reject_singleblock_q;
  int unsigned tc_dbg_reject_short_q;
  int unsigned tc_dbg_reject_policy_q;
  int unsigned tc_dbg_reject_unvalidated_q;  // hits dropped by the full-path gate
  int unsigned tc_dbg_hit_unvalidated_q;     // hits whose path was NOT fully validated
  int unsigned tc_dbg_feed_done_q;
  int unsigned tc_dbg_feed_cycles_q;
  int unsigned tc_dbg_hold_count_q;
  int unsigned tc_dbg_pending_use_count_q;
  int unsigned tc_dbg_immediate_use_count_q;
  int unsigned tc_dbg_miss_empty_q;
  int unsigned tc_dbg_miss_pc_q;
  int unsigned tc_dbg_miss_path_q;
  int unsigned tc_dbg_feed_start_count_q;
  int unsigned tc_dbg_replay_pkt_count_q;
  int unsigned tc_dbg_feed_start_total_q;
  int unsigned tc_dbg_tc_pkt_cycles_q;
  int unsigned tc_dbg_tc_instr_total_q;
  int unsigned tc_dbg_icache_pkt_cycles_q;
  int unsigned tc_dbg_icache_instr_total_q;
  int unsigned tc_dbg_icache_block_cycles_q;
  logic        tc_dbg_seen_coremark_pc_q;
  int unsigned tc_dbg_pc_floor_breach_q;
  int unsigned tc_dbg_replay_remaining_q;
  int unsigned tc_dbg_replay_deq_trace_count_q;
  logic        tc_dbg_post_replay_armed_q;
  int unsigned tc_dbg_post_replay_count_q;
  int unsigned tc_dbg_post_replay_mismatch_q;
  int unsigned tc_dbg_window_total_q;
  int unsigned tc_dbg_window_unaligned_q;
  int unsigned tc_dbg_window_not_ready_q;
  int unsigned tc_dbg_window_cf0_q;
  int unsigned tc_dbg_window_cf1_q;
  int unsigned tc_dbg_window_cf2_q;
  int unsigned tc_dbg_window_cf3_q;
  int unsigned tc_dbg_window_cf4p_q;
  int unsigned tc_dbg_window_taken0_q;
  int unsigned tc_dbg_window_taken1_q;
  int unsigned tc_dbg_window_taken2_q;
  int unsigned tc_dbg_window_taken3_q;
  int unsigned tc_dbg_window_taken4p_q;
  int unsigned tc_dbg_window_single_taken_q;
  int unsigned tc_dbg_window_multi_taken_q;
  int unsigned tc_dbg_mispredict_count_q;
  int unsigned tc_dbg_accept_multiblock_q;
  int unsigned tc_dbg_trigger_single_cond_q;
  int unsigned tc_dbg_trigger_single_jump_q;
  int unsigned tc_dbg_trigger_single_call_q;
  int unsigned tc_dbg_trigger_single_return_q;
  int unsigned tc_dbg_trigger_single_jalr_q;
  int unsigned tc_dbg_trigger_single_unaligned_q;
  int unsigned tc_dbg_trigger_single_not_ready_q;
  int unsigned tc_dbg_iq_flush_on_capture_q;
  int unsigned tc_dbg_bht_scan_cancel_q;
  int unsigned tc_dbg_poison_block_q;
  int unsigned tc_dbg_poison_set_q;
  int unsigned tc_dbg_pending_capture_q;
  int unsigned tc_dbg_pending_use_q;
  int unsigned tc_dbg_pending_invalidate_q;
  // Backend stall / frontend bubble counters
  int unsigned tc_dbg_backend_stall_q;        // cycles backend not ready (fetch_entry_valid & !fetch_entry_ready)
  int unsigned tc_dbg_iq_empty_cycles_q;      // cycles IQ was empty (frontend starvation)
  int unsigned tc_dbg_iq_not_ready_cycles_q;  // cycles IQ was full (backpressure)
  int unsigned tc_dbg_icache_miss_cycles_q;   // cycles I$ not valid (cache miss stall)
  int unsigned tc_dbg_replay_profit_2tk_q;    // accepted replays with exactly 2 taken
  int unsigned tc_dbg_replay_profit_3tk_q;    // accepted replays with ?3 taken
  int unsigned tc_dbg_replay_len_hist_q [8];  // accepted trace length histogram [3..7+]
  // IQ-gate suppression counter
  int unsigned tc_dbg_iq_gate_suppress_q;     // hits suppressed because IQ was non-empty
  // Multi-taken coverage histograms.  Bucket tk3p is MAX_TAKEN (currently 3).
  localparam int unsigned TC_TAKEN_HIST_BINS = MAX_TAKEN + 1;
  int unsigned tc_dbg_hit_taken_hist_q [TC_TAKEN_HIST_BINS];
  int unsigned tc_dbg_accept_taken_hist_q [TC_TAKEN_HIST_BINS];
  int unsigned tc_dbg_not_ready_taken_hist_q [TC_TAKEN_HIST_BINS];
  int unsigned tc_dbg_iq_gate_taken_hist_q [TC_TAKEN_HIST_BINS];
  int unsigned tc_dbg_policy_taken_hist_q [TC_TAKEN_HIST_BINS];
  int unsigned tc_dbg_used_taken_hist_q [TC_TAKEN_HIST_BINS];

  // Hot-PC frontend tracker: hit/miss/policy for top mismatch PCs
  localparam logic [CVA6Cfg.VLEN-1:0] FE_HP0 = CVA6Cfg.VLEN'(64'h80002880);
  localparam logic [CVA6Cfg.VLEN-1:0] FE_HP1 = CVA6Cfg.VLEN'(64'h8000287a);
  localparam logic [CVA6Cfg.VLEN-1:0] FE_HP2 = CVA6Cfg.VLEN'(64'h80002888);
  localparam logic [CVA6Cfg.VLEN-1:0] FE_HP3 = CVA6Cfg.VLEN'(64'h8000287e);
  int unsigned fe_hp_lookup [4];    // lookup fired for this PC
  int unsigned fe_hp_hit    [4];    // trace hit for this PC
  int unsigned fe_hp_accept [4];    // hit + policy_ok + accepted
  int unsigned fe_hp_reject_policy [4]; // hit but policy rejected
  int unsigned fe_hp_miss_pc [4];   // miss: pc mismatch
  int unsigned fe_hp_miss_empty [4]; // miss: set empty
  int unsigned fe_hp_miss_path [4]; // miss: path mismatch

  // -----------------------------------------------------------------------
  // ROI (Region of Interest) measurement
  // The ROI is activated by the first committed instruction in the
  // Coremark code region (PC >= 0x80000000) and deactivated when PC
  // leaves that region (e.g., WFI spin at 0x0200018a).  All TC stats
  // within the ROI are counted separately so boot/tail overhead is
  // excluded.  Override with +ROI_START_PC=<hex> and +ROI_END_PC=<hex>.
  // -----------------------------------------------------------------------
  logic        roi_active_q;
  logic [63:0] roi_start_pc;
  logic [63:0] roi_end_pc;
  longint unsigned roi_cycle_count_q;
  int unsigned roi_instr_count_q;
  int unsigned roi_tc_accept_q;
  int unsigned roi_tc_instr_q;
  int unsigned roi_tc_pending_use_q;
  int unsigned roi_tc_lookup_q;
  int unsigned roi_tc_hit_q;
  int unsigned roi_tc_miss_empty_q;
  int unsigned roi_tc_miss_pc_q;
  int unsigned roi_tc_miss_path_q;
  int unsigned roi_icache_instr_q;
  int unsigned roi_tc_hit_taken_hist_q [TC_TAKEN_HIST_BINS];
  int unsigned roi_tc_accept_taken_hist_q [TC_TAKEN_HIST_BINS];

  // ROI plusarg configuration
  initial begin
    roi_start_pc = 64'h80000000;  // default: Coremark code region start
    roi_end_pc   = 64'h80010000;  // default: Coremark code region end (64KB)
    if ($value$plusargs("ROI_START_PC=%h", roi_start_pc))
      $display("[TC-ROI] Start PC overridden to 0x%h", roi_start_pc);
    if ($value$plusargs("ROI_END_PC=%h", roi_end_pc))
      $display("[TC-ROI] End PC overridden to 0x%h", roi_end_pc);
    $display("[TC-ROI] Region: [0x%h, 0x%h)", roi_start_pc, roi_end_pc);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_total_cycles_q         <= 0;
      tc_dbg_lookup_count_q     <= 0;
      tc_dbg_hit_count_q        <= 0;
      tc_dbg_accept_count_q     <= 0;
      tc_dbg_reject_not_ready_q <= 0;
      tc_dbg_reject_used_q      <= 0;
      tc_dbg_reject_indirect_q    <= 0;
      tc_dbg_reject_singleblock_q <= 0;
      tc_dbg_reject_short_q       <= 0;
      tc_dbg_reject_policy_q      <= 0;
      tc_dbg_reject_unvalidated_q <= 0;
      tc_dbg_hit_unvalidated_q    <= 0;
      tc_dbg_feed_done_q        <= 0;
      tc_dbg_feed_cycles_q      <= 0;
      tc_dbg_hold_count_q       <= 0;
      tc_dbg_pending_use_count_q <= 0;
      tc_dbg_immediate_use_count_q <= 0;
      tc_dbg_miss_empty_q       <= 0;
      tc_dbg_miss_pc_q          <= 0;
      tc_dbg_miss_path_q        <= 0;
      tc_dbg_feed_start_count_q <= 0;
      tc_dbg_replay_pkt_count_q <= 0;
      tc_dbg_feed_start_total_q <= 0;
      tc_dbg_tc_pkt_cycles_q    <= 0;
      tc_dbg_tc_instr_total_q   <= 0;
      tc_dbg_icache_pkt_cycles_q <= 0;
      tc_dbg_icache_instr_total_q <= 0;
      tc_dbg_icache_block_cycles_q <= 0;
      tc_dbg_seen_coremark_pc_q <= 1'b0;
      tc_dbg_pc_floor_breach_q <= 0;
      tc_dbg_replay_remaining_q <= 0;
      tc_dbg_replay_deq_trace_count_q <= 0;
      tc_dbg_post_replay_armed_q <= 1'b0;
      tc_dbg_post_replay_count_q <= 0;
      tc_dbg_post_replay_mismatch_q <= 0;
      tc_dbg_window_total_q <= 0;
      tc_dbg_window_unaligned_q <= 0;
      tc_dbg_window_not_ready_q <= 0;
      tc_dbg_window_cf0_q <= 0;
      tc_dbg_window_cf1_q <= 0;
      tc_dbg_window_cf2_q <= 0;
      tc_dbg_window_cf3_q <= 0;
      tc_dbg_window_cf4p_q <= 0;
      tc_dbg_window_taken0_q <= 0;
      tc_dbg_window_taken1_q <= 0;
      tc_dbg_window_taken2_q <= 0;
      tc_dbg_window_taken3_q <= 0;
      tc_dbg_window_taken4p_q <= 0;
      tc_dbg_window_single_taken_q <= 0;
      tc_dbg_window_multi_taken_q <= 0;
      tc_dbg_mispredict_count_q <= 0;
      tc_dbg_accept_multiblock_q <= 0;
      tc_dbg_trigger_single_cond_q <= 0;
      tc_dbg_trigger_single_jump_q <= 0;
      tc_dbg_trigger_single_call_q <= 0;
      tc_dbg_trigger_single_return_q <= 0;
      tc_dbg_trigger_single_jalr_q <= 0;
      tc_dbg_trigger_single_unaligned_q <= 0;
      tc_dbg_trigger_single_not_ready_q <= 0;
      tc_dbg_iq_flush_on_capture_q <= 0;
      tc_dbg_bht_scan_cancel_q <= 0;
      tc_dbg_poison_block_q <= 0;
      tc_dbg_poison_set_q <= 0;
      tc_dbg_pending_capture_q <= 0;
      tc_dbg_pending_use_q <= 0;
      tc_dbg_pending_invalidate_q <= 0;
      tc_dbg_backend_stall_q <= 0;
      tc_dbg_iq_empty_cycles_q <= 0;
      tc_dbg_iq_not_ready_cycles_q <= 0;
      tc_dbg_icache_miss_cycles_q <= 0;
      tc_dbg_replay_profit_2tk_q <= 0;
      tc_dbg_replay_profit_3tk_q <= 0;
      tc_dbg_iq_gate_suppress_q <= 0;
      for (int i = 0; i < 8; i++) tc_dbg_replay_len_hist_q[i] <= 0;
      for (int i = 0; i < TC_TAKEN_HIST_BINS; i++) begin
        tc_dbg_hit_taken_hist_q[i]       <= 0;
        tc_dbg_accept_taken_hist_q[i]    <= 0;
        tc_dbg_not_ready_taken_hist_q[i] <= 0;
        tc_dbg_iq_gate_taken_hist_q[i]   <= 0;
        tc_dbg_policy_taken_hist_q[i]    <= 0;
        tc_dbg_used_taken_hist_q[i]      <= 0;
        roi_tc_hit_taken_hist_q[i]       <= 0;
        roi_tc_accept_taken_hist_q[i]    <= 0;
      end
      for (int i = 0; i < 4; i++) begin
        fe_hp_lookup[i] <= 0; fe_hp_hit[i] <= 0; fe_hp_accept[i] <= 0;
        fe_hp_reject_policy[i] <= 0; fe_hp_miss_pc[i] <= 0;
        fe_hp_miss_empty[i] <= 0; fe_hp_miss_path[i] <= 0;
      end
      roi_active_q <= 1'b0;
      roi_cycle_count_q <= 0;
      roi_instr_count_q <= 0;
      roi_tc_accept_q <= 0;
      roi_tc_instr_q <= 0;
      roi_tc_pending_use_q <= 0;
      roi_tc_lookup_q <= 0;
      roi_tc_hit_q <= 0;
      roi_tc_miss_empty_q <= 0;
      roi_tc_miss_pc_q <= 0;
      roi_tc_miss_path_q <= 0;
      roi_icache_instr_q <= 0;
    end else begin
      int unsigned src_cnt;
      int unsigned cf_cnt;
      int unsigned taken_cnt;
      int unsigned first_taken_idx;
      int unsigned deq_count;
      int unsigned replay_remaining_after;
      int unsigned trace_taken_idx;
      int unsigned feed_taken_idx;
      tc_total_cycles_q <= tc_total_cycles_q + 1;
      deq_count = 0;
      replay_remaining_after = tc_dbg_replay_remaining_q;
      for (int p = 0; p < CVA6Cfg.NrIssuePorts; p++) begin
        if (fetch_entry_valid_o[p] && fetch_entry_ready_i[p])
          deq_count++;
      end

      if (tc_dbg_replay_remaining_q != 0 && deq_count != 0) begin
        if (deq_count >= tc_dbg_replay_remaining_q)
          replay_remaining_after = 0;
        else
          replay_remaining_after = tc_dbg_replay_remaining_q - deq_count;
      end

      if (resolved_branch_i.valid && resolved_branch_i.is_mispredict)
        tc_dbg_mispredict_count_q <= tc_dbg_mispredict_count_q + 1;

      if (tc_lookup_result_valid) begin
        tc_dbg_lookup_count_q <= tc_dbg_lookup_count_q + 1;

        if (tc_trace_hit) begin
          tc_dbg_hit_count_q <= tc_dbg_hit_count_q + 1;
          trace_taken_idx = int'(tc_trace_num_taken);
          if (trace_taken_idx > MAX_TAKEN)
            trace_taken_idx = MAX_TAKEN;
          tc_dbg_hit_taken_hist_q[trace_taken_idx] <=
              tc_dbg_hit_taken_hist_q[trace_taken_idx] + 1;

          // how many hits carry branches the tag never validated
          if (!tc_trace_path_validated)
            tc_dbg_hit_unvalidated_q <= tc_dbg_hit_unvalidated_q + 1;

          if (tc_trace_policy_ok) begin
            if (tc_trace_used) begin
              tc_dbg_reject_used_q <= tc_dbg_reject_used_q + 1;
              tc_dbg_used_taken_hist_q[trace_taken_idx] <=
                  tc_dbg_used_taken_hist_q[trace_taken_idx] + 1;
            end else if (!tc_direct_hit && !tc_pending_capture) begin
              tc_dbg_reject_not_ready_q <= tc_dbg_reject_not_ready_q + 1;
              tc_dbg_not_ready_taken_hist_q[trace_taken_idx] <=
                  tc_dbg_not_ready_taken_hist_q[trace_taken_idx] + 1;
              // Count how many of these were specifically blocked by IQ gate
              if (instr_queue_ready && !instr_queue_empty) begin
                tc_dbg_iq_gate_suppress_q <= tc_dbg_iq_gate_suppress_q + 1;
                tc_dbg_iq_gate_taken_hist_q[trace_taken_idx] <=
                    tc_dbg_iq_gate_taken_hist_q[trace_taken_idx] + 1;
              end
            end
          end else begin
            if (tc_trace_has_indirect || tc_trace_has_call)
              tc_dbg_reject_indirect_q <= tc_dbg_reject_indirect_q + 1;
            if (tc_trace_num_taken == '0)
              tc_dbg_reject_singleblock_q <= tc_dbg_reject_singleblock_q + 1;
            if (tc_trace_length < TRACE_LEN_WIDTH'(TC_MIN_ACCEPT_LEN))
              tc_dbg_reject_short_q <= tc_dbg_reject_short_q + 1;
            // attribute rejections to the full-path validation gate
            if (!tc_trace_path_validated && !tc_no_path_gate_q)
              tc_dbg_reject_unvalidated_q <= tc_dbg_reject_unvalidated_q + 1;
            tc_dbg_reject_policy_q <= tc_dbg_reject_policy_q + 1;
            tc_dbg_policy_taken_hist_q[trace_taken_idx] <=
                tc_dbg_policy_taken_hist_q[trace_taken_idx] + 1;
          end
        end else begin
          if (tc_trace_used && tc_trace_policy_ok)
            tc_dbg_reject_used_q <= tc_dbg_reject_used_q + 1;
          else begin
            if (tc_miss_reason_empty) tc_dbg_miss_empty_q <= tc_dbg_miss_empty_q + 1;
            if (tc_miss_reason_pc)    tc_dbg_miss_pc_q    <= tc_dbg_miss_pc_q + 1;
            if (tc_miss_reason_path)  tc_dbg_miss_path_q  <= tc_dbg_miss_path_q + 1;
          end
        end

        // Hot-PC per-lookup classification
        for (int i = 0; i < 4; i++) begin
          logic [CVA6Cfg.VLEN-1:0] hp;
          case (i)
            0: hp = FE_HP0;
            1: hp = FE_HP1;
            2: hp = FE_HP2;
            3: hp = FE_HP3;
          endcase
          if (tc_lookup_pc_q == hp) begin
            fe_hp_lookup[i] <= fe_hp_lookup[i] + 1;
            if (tc_trace_hit) begin
              fe_hp_hit[i] <= fe_hp_hit[i] + 1;
              if (tc_trace_policy_ok && !tc_trace_used)
                fe_hp_accept[i] <= fe_hp_accept[i] + 1;
              if (!tc_trace_policy_ok)
                fe_hp_reject_policy[i] <= fe_hp_reject_policy[i] + 1;
            end else begin
              if (tc_miss_reason_empty) fe_hp_miss_empty[i] <= fe_hp_miss_empty[i] + 1;
              if (tc_miss_reason_pc)    fe_hp_miss_pc[i]    <= fe_hp_miss_pc[i] + 1;
              if (tc_miss_reason_path)  fe_hp_miss_path[i]  <= fe_hp_miss_path[i] + 1;
            end
          end
        end
      end

      if (tc_hit_this_cycle)
        tc_dbg_feed_cycles_q <= tc_dbg_feed_cycles_q + 1;

      if (tc_hit_this_cycle)
        tc_dbg_feed_start_total_q <= tc_dbg_feed_start_total_q + 1;

      if (!tc_icache_sidefx_en)
        tc_dbg_icache_block_cycles_q <= tc_dbg_icache_block_cycles_q + 1;

      if (tc_icache_sidefx_en && (|instruction_valid)) begin
        tc_dbg_icache_pkt_cycles_q <= tc_dbg_icache_pkt_cycles_q + 1;
        src_cnt = 0;
        for (int k = 0; k < CVA6Cfg.INSTR_PER_FETCH; k++)
          if (instruction_valid[k]) src_cnt++;
        tc_dbg_icache_instr_total_q <= tc_dbg_icache_instr_total_q + src_cnt;
      end

      if ((|instruction_valid) && addr[0][31] && serving_unaligned)
        tc_dbg_window_unaligned_q <= tc_dbg_window_unaligned_q + 1;

      if ((|instruction_valid) && addr[0][31] && tc_window_eligible &&
          !instr_queue_ready)
        tc_dbg_window_not_ready_q <= tc_dbg_window_not_ready_q + 1;

      if (tc_icache_sidefx_en && (|instruction_valid) && addr[0][31]) begin
        cf_cnt = 0;
        taken_cnt = 0;
        first_taken_idx = SLOTS_PER_CYCLE;
        for (int k = 0; k < SLOTS_PER_CYCLE; k++) begin
          if (instruction_valid[k] && tc_is_branch[k])
            cf_cnt++;
          if (tc_raw_taken[k]) begin
            taken_cnt++;
            if (first_taken_idx == SLOTS_PER_CYCLE)
              first_taken_idx = k;
          end
        end

        tc_dbg_window_total_q <= tc_dbg_window_total_q + 1;

        unique case (cf_cnt)
          0:      tc_dbg_window_cf0_q <= tc_dbg_window_cf0_q + 1;
          1:      tc_dbg_window_cf1_q <= tc_dbg_window_cf1_q + 1;
          2:      tc_dbg_window_cf2_q <= tc_dbg_window_cf2_q + 1;
          3:      tc_dbg_window_cf3_q <= tc_dbg_window_cf3_q + 1;
          default: tc_dbg_window_cf4p_q <= tc_dbg_window_cf4p_q + 1;
        endcase

        unique case (taken_cnt)
          0:      tc_dbg_window_taken0_q <= tc_dbg_window_taken0_q + 1;
          1:      tc_dbg_window_taken1_q <= tc_dbg_window_taken1_q + 1;
          2:      tc_dbg_window_taken2_q <= tc_dbg_window_taken2_q + 1;
          3:      tc_dbg_window_taken3_q <= tc_dbg_window_taken3_q + 1;
          default: tc_dbg_window_taken4p_q <= tc_dbg_window_taken4p_q + 1;
        endcase

        if (taken_cnt == 1)
          tc_dbg_window_single_taken_q <= tc_dbg_window_single_taken_q + 1;
        else if (taken_cnt > 1)
          tc_dbg_window_multi_taken_q <= tc_dbg_window_multi_taken_q + 1;

        if (taken_cnt == 1 && first_taken_idx < SLOTS_PER_CYCLE) begin
          if (serving_unaligned)
            tc_dbg_trigger_single_unaligned_q <= tc_dbg_trigger_single_unaligned_q + 1;
          if (!instr_queue_ready)
            tc_dbg_trigger_single_not_ready_q <= tc_dbg_trigger_single_not_ready_q + 1;

          if (is_call[first_taken_idx])
            tc_dbg_trigger_single_call_q <= tc_dbg_trigger_single_call_q + 1;
          else if (is_return[first_taken_idx])
            tc_dbg_trigger_single_return_q <= tc_dbg_trigger_single_return_q + 1;
          else if (is_jalr[first_taken_idx])
            tc_dbg_trigger_single_jalr_q <= tc_dbg_trigger_single_jalr_q + 1;
          else if (is_jump[first_taken_idx])
            tc_dbg_trigger_single_jump_q <= tc_dbg_trigger_single_jump_q + 1;
          else if (is_branch[first_taken_idx])
            tc_dbg_trigger_single_cond_q <= tc_dbg_trigger_single_cond_q + 1;
        end
      end

      if (fetch_entry_valid_o[0] && fetch_entry_ready_i[0] &&
          (fetch_entry_o[0].address >= CVA6Cfg.VLEN'(64'h80000000)))
        tc_dbg_seen_coremark_pc_q <= 1'b1;

      if (tc_dbg_seen_coremark_pc_q && fetch_entry_valid_o[0] && fetch_entry_ready_i[0] &&
          (fetch_entry_o[0].address < CVA6Cfg.VLEN'(64'h80000000))) begin
        tc_dbg_pc_floor_breach_q <= tc_dbg_pc_floor_breach_q + 1;
        if (tc_dbg_pc_floor_breach_q < 16) begin
          $display("[TC-PC-FLOOR-BREACH] t=%0t pc=0x%h replay=%0b replay_addr=0x%h iq_ready=%0b tc_hit=%0b",
                   $time, fetch_entry_o[0].address, replay, replay_addr, instr_queue_ready, tc_hit_this_cycle);
        end
      end

      if (flush_i || is_mispredict) begin
        tc_dbg_replay_remaining_q <= 0;
        tc_dbg_post_replay_armed_q <= 1'b0;
      end else begin
        if (tc_hit_this_cycle) begin
          tc_dbg_replay_remaining_q <= tc_trace_length;
          tc_dbg_post_replay_armed_q <= 1'b0;
        end else begin
          if (tc_dbg_replay_remaining_q != 0 && deq_count != 0) begin
            if (deq_count >= tc_dbg_replay_remaining_q) begin
              tc_dbg_replay_remaining_q <= 0;
              tc_dbg_post_replay_armed_q <= 1'b1;
            end else begin
              tc_dbg_replay_remaining_q <= tc_dbg_replay_remaining_q - deq_count;
            end
          end

          if (tc_dbg_post_replay_armed_q && deq_count != 0)
            tc_dbg_post_replay_armed_q <= 1'b0;
        end
      end

      if ((tc_dbg_replay_remaining_q != 0) && (deq_count != 0) &&
          (tc_dbg_replay_deq_trace_count_q < 256)) begin
        for (int p = 0; p < CVA6Cfg.NrIssuePorts; p++) begin
          if (fetch_entry_valid_o[p] && fetch_entry_ready_i[p]) begin
            $display("[TC-REPLAY-DEQ] t=%0t port=%0d pc_out=0x%h instr=0x%08h cf=%0d pred=0x%h rem_before=%0d rem_after=%0d npc_q=0x%h",
                     $time,
                     p,
                     fetch_entry_o[p].address,
                     fetch_entry_o[p].instruction,
                     fetch_entry_o[p].branch_predict.cf,
                     fetch_entry_o[p].branch_predict.predict_address,
                     tc_dbg_replay_remaining_q,
                     replay_remaining_after,
                     npc_q);
          end
        end
        tc_dbg_replay_deq_trace_count_q <= tc_dbg_replay_deq_trace_count_q + deq_count;
      end

      if (tc_dbg_post_replay_armed_q && deq_count != 0) begin
        // Count every post-replay event; only $display a limited number
        // to prevent transcript flooding.  match=0 is EXPECTED when the IQ
        // contains stale I-cache entries before the TC trace data (the debug
        // counter tc_dbg_replay_remaining_q counts ALL dequeues, not just TC
        // instruction dequeues, so it reaches zero while stale entries remain).
        tc_dbg_post_replay_count_q <= tc_dbg_post_replay_count_q + 1;
        if (fetch_entry_o[0].address != tc_trace_next_pc[CVA6Cfg.VLEN-1:0])
          tc_dbg_post_replay_mismatch_q <= tc_dbg_post_replay_mismatch_q + 1;
        if (tc_dbg_post_replay_count_q < 32) begin
          $display("[TC-POST-REPLAY] t=%0t pc_out=0x%h instr=0x%08h cf=%0d npc_q=0x%h expected_next=0x%h match=%0b (#%0d)",
                   $time,
                   fetch_entry_o[0].address,
                   fetch_entry_o[0].instruction,
                   fetch_entry_o[0].branch_predict.cf,
                   npc_q,
                   tc_trace_next_pc[CVA6Cfg.VLEN-1:0],
                   (fetch_entry_o[0].address == tc_trace_next_pc[CVA6Cfg.VLEN-1:0]),
                   tc_dbg_post_replay_count_q + 1);
        end
      end

      // count TC hit cycles
      if (tc_hit_this_cycle) begin
        tc_dbg_tc_pkt_cycles_q <= tc_dbg_tc_pkt_cycles_q + 1;
        src_cnt = 0;
        for (int k = 0; k < CVA6Cfg.INSTR_PER_FETCH; k++)
          if (tc_feed_valid[k]) src_cnt++;
        tc_dbg_tc_instr_total_q <= tc_dbg_tc_instr_total_q + src_cnt;
      end

      if (tc_hit_this_cycle)
        tc_dbg_hold_count_q <= tc_dbg_hold_count_q + 1;

      if (tc_hit_this_cycle && !instr_queue_empty)
        tc_dbg_iq_flush_on_capture_q <= tc_dbg_iq_flush_on_capture_q + 1;

      // Backend stall and frontend bubble counters
      if (|fetch_entry_valid_o && !(|fetch_entry_ready_i))
        tc_dbg_backend_stall_q <= tc_dbg_backend_stall_q + 1;
      if (instr_queue_empty)
        tc_dbg_iq_empty_cycles_q <= tc_dbg_iq_empty_cycles_q + 1;
      if (!instr_queue_ready)
        tc_dbg_iq_not_ready_cycles_q <= tc_dbg_iq_not_ready_cycles_q + 1;
      if (!icache_dreq_i.valid && !flush_i && !is_mispredict)
        tc_dbg_icache_miss_cycles_q <= tc_dbg_icache_miss_cycles_q + 1;

      // Replay profitability tracking.
      if (tc_hit_this_cycle) begin
        feed_taken_idx = int'(tc_feed_src_num_taken);
        if (feed_taken_idx > MAX_TAKEN)
          feed_taken_idx = MAX_TAKEN;
        tc_dbg_accept_taken_hist_q[feed_taken_idx] <=
            tc_dbg_accept_taken_hist_q[feed_taken_idx] + 1;
        if (tc_feed_src_num_taken == TAKEN_CNT_WIDTH'(2))
          tc_dbg_replay_profit_2tk_q <= tc_dbg_replay_profit_2tk_q + 1;
        if (tc_feed_src_num_taken >= TAKEN_CNT_WIDTH'(3))
          tc_dbg_replay_profit_3tk_q <= tc_dbg_replay_profit_3tk_q + 1;
        // Length histogram: index 0=len3, 1=len4, ..., 4=len7, 5+=overflow
        begin
          int len_idx;
          len_idx = int'(tc_feed_src_length) - 3;
          if (len_idx < 0) len_idx = 0;
          if (len_idx > 7) len_idx = 7;
          tc_dbg_replay_len_hist_q[len_idx] <= tc_dbg_replay_len_hist_q[len_idx] + 1;
        end
      end

      // scan_cancel, poison counters removed.

      // pending buffer tracking
      if (tc_pending_capture)
        tc_dbg_pending_capture_q <= tc_dbg_pending_capture_q + 1;
      if (tc_pending_use)
        tc_dbg_pending_use_q <= tc_dbg_pending_use_q + 1;
      if (tc_pending_valid_q && tc_pending_invalidate)
        tc_dbg_pending_invalidate_q <= tc_dbg_pending_invalidate_q + 1;

      // ROI tracking
      // ROI activates when a committed instruction is in [roi_start_pc, roi_end_pc).
      // ROI deactivates when a committed instruction is outside that range
      // (after having been active), capturing exactly the benchmark region.
      if (fetch_entry_valid_o[0] && fetch_entry_ready_i[0]) begin
        logic [63:0] commit_pc_64;
        commit_pc_64 = {32'b0, fetch_entry_o[0].address};
        if (!roi_active_q) begin
          if (commit_pc_64 >= roi_start_pc && commit_pc_64 < roi_end_pc)
            roi_active_q <= 1'b1;
        end else begin
          if (commit_pc_64 < roi_start_pc || commit_pc_64 >= roi_end_pc)
            roi_active_q <= 1'b0;
        end
      end

      if (roi_active_q) begin
        int unsigned roi_src_cnt;
        roi_cycle_count_q <= roi_cycle_count_q + 1;

        // Count committed instructions (from IQ dequeue ports)
        for (int p = 0; p < CVA6Cfg.NrIssuePorts; p++)
          if (fetch_entry_valid_o[p] && fetch_entry_ready_i[p])
            roi_instr_count_q <= roi_instr_count_q + 1;

        // TC stats within ROI
        if (tc_hit_this_cycle) begin
          roi_tc_accept_q <= roi_tc_accept_q + 1;
          feed_taken_idx = int'(tc_feed_src_num_taken);
          if (feed_taken_idx > MAX_TAKEN)
            feed_taken_idx = MAX_TAKEN;
          roi_tc_accept_taken_hist_q[feed_taken_idx] <=
              roi_tc_accept_taken_hist_q[feed_taken_idx] + 1;
          roi_src_cnt = 0;
          for (int k = 0; k < CVA6Cfg.INSTR_PER_FETCH; k++)
            if (tc_feed_valid[k]) roi_src_cnt++;
          roi_tc_instr_q <= roi_tc_instr_q + roi_src_cnt;
        end

        if (tc_pending_use)
          roi_tc_pending_use_q <= roi_tc_pending_use_q + 1;

        if (tc_lookup_result_valid) begin
          roi_tc_lookup_q <= roi_tc_lookup_q + 1;
          if (tc_trace_hit) begin
            roi_tc_hit_q <= roi_tc_hit_q + 1;
            trace_taken_idx = int'(tc_trace_num_taken);
            if (trace_taken_idx > MAX_TAKEN)
              trace_taken_idx = MAX_TAKEN;
            roi_tc_hit_taken_hist_q[trace_taken_idx] <=
                roi_tc_hit_taken_hist_q[trace_taken_idx] + 1;
          end else begin
            if (tc_miss_reason_empty) roi_tc_miss_empty_q <= roi_tc_miss_empty_q + 1;
            if (tc_miss_reason_pc)    roi_tc_miss_pc_q    <= roi_tc_miss_pc_q + 1;
            if (tc_miss_reason_path)  roi_tc_miss_path_q  <= roi_tc_miss_path_q + 1;
          end
        end

        // I$ instruction count within ROI
        if (tc_icache_sidefx_en && (|instruction_valid) && !tc_hit_this_cycle) begin
          roi_src_cnt = 0;
          for (int k = 0; k < CVA6Cfg.INSTR_PER_FETCH; k++)
            if (instruction_valid[k]) roi_src_cnt++;
          roi_icache_instr_q <= roi_icache_instr_q + roi_src_cnt;
        end
      end

      if (tc_hit_this_cycle) begin
        tc_dbg_pending_use_count_q <= tc_dbg_pending_use_count_q + 1;
        tc_dbg_accept_count_q <= tc_dbg_accept_count_q + 1;
        if (tc_trace_num_branches != BR_CNT_WIDTH'(0))
          tc_dbg_accept_multiblock_q <= tc_dbg_accept_multiblock_q + 1;
      end

      if (tc_hit_this_cycle) begin
        tc_dbg_feed_done_q <= tc_dbg_feed_done_q + 1;
        if (tc_dbg_feed_done_q < 256) begin
          $display("[TC-FEED-DONE] t=%0t len=%0d num_taken=%0d iq_ready=%0b iq_empty=%0b next_pc=0x%h",
                   $time, tc_feed_src_length, tc_feed_src_num_taken,
                   instr_queue_ready, instr_queue_empty,
                   tc_feed_src_next_pc[CVA6Cfg.VLEN-1:0]);
        end
      end

      if (tc_hit_this_cycle && (tc_dbg_feed_start_count_q < 96)) begin
        tc_dbg_feed_start_count_q <= tc_dbg_feed_start_count_q + 1;
        $display("[TC-HIT-FEED] #%0d t=%0t len=%0d num_taken=%0d next_pc=0x%h",
                 tc_dbg_feed_start_count_q + 1, $time, tc_feed_src_length,
                 tc_feed_src_num_taken, tc_feed_src_next_pc[CVA6Cfg.VLEN-1:0]);
        for (int k = 0; k < TRACE_LEN; k++) begin
          if (k < int'(tc_feed_src_length)) begin
            $display("[TC-HIT-FEED]   src[%0d] pc=0x%h instr=0x%08h cf=%0d",
                     k, tc_feed_src_pcs[k][CVA6Cfg.VLEN-1:0], tc_feed_src_instructions[k],
                     tc_replay_cf_type(tc_feed_src_instructions[k]));
          end
        end
      end

      // no separate replay packet tracking ? TC feeds are same-cycle.

      if (tc_lookup_result_valid && (tc_dbg_lookup_count_q < 80)) begin
        if (tc_trace_hit) begin
          if (tc_hit_this_cycle) begin
            $display("[TC-HIT-ACCEPT] t=%0t pc0=0x%h len=%0d num_taken=%0d last_taken_last=%0b iq_ready=%0b iq_empty=%0b",
                     $time, tc_trace_pcs[0], tc_trace_length, tc_trace_num_taken, tc_trace_last_is_taken_cf, instr_queue_ready, instr_queue_empty);
          end else begin
            $display("[TC-HIT-DROP] t=%0t pc0=0x%h len=%0d num_taken=%0d starts_ok=%0b last_taken_last=%0b iq_ready=%0b iq_empty=%0b flush=%0b mispredict=%0b",
                     $time, tc_trace_pcs[0], tc_trace_length, tc_trace_num_taken,
                     tc_trace_starts_ok, tc_trace_last_is_taken_cf, instr_queue_ready,
                     instr_queue_empty, flush_i, is_mispredict);
          end
        end
      end
    end
  end

  final begin
    int unsigned tc_dbg_miss_count_q;
    int unsigned tc_dbg_hit_rate_q;
    int unsigned tc_dbg_accept_rate_q;
    int unsigned tc_dbg_feed_pct_q;
    int unsigned tc_dbg_pkt_total_q;
    int unsigned tc_dbg_pkt_tc_share_q;
    int unsigned tc_dbg_instr_total_q;
    int unsigned tc_dbg_instr_tc_share_q;

    tc_dbg_miss_count_q  = tc_dbg_lookup_count_q - tc_dbg_hit_count_q;
    if (tc_dbg_lookup_count_q > 0)
      tc_dbg_hit_rate_q = (tc_dbg_hit_count_q * 100) / tc_dbg_lookup_count_q;
    else
      tc_dbg_hit_rate_q = 0;
    if (tc_dbg_hit_count_q > 0)
      tc_dbg_accept_rate_q = (tc_dbg_accept_count_q * 100) / tc_dbg_hit_count_q;
    else
      tc_dbg_accept_rate_q = 0;
    if (tc_total_cycles_q > 0)
      tc_dbg_feed_pct_q = (tc_dbg_feed_cycles_q * 100) / tc_total_cycles_q;
    else
      tc_dbg_feed_pct_q = 0;
    tc_dbg_pkt_total_q   = tc_dbg_tc_pkt_cycles_q + tc_dbg_icache_pkt_cycles_q;
    if (tc_dbg_pkt_total_q > 0)
      tc_dbg_pkt_tc_share_q = (tc_dbg_tc_pkt_cycles_q * 100) / tc_dbg_pkt_total_q;
    else
      tc_dbg_pkt_tc_share_q = 0;
    tc_dbg_instr_total_q = tc_dbg_tc_instr_total_q + tc_dbg_icache_instr_total_q;
    if (tc_dbg_instr_total_q > 0)
      tc_dbg_instr_tc_share_q = (tc_dbg_tc_instr_total_q * 100) / tc_dbg_instr_total_q;
    else
      tc_dbg_instr_tc_share_q = 0;

    $display("[TC-FINAL] ========== Trace Cache V77 summary ==========");
    $display("[TC-FINAL] lookups=%0d hits=%0d misses=%0d hit_rate=%0d%%",
             tc_dbg_lookup_count_q, tc_dbg_hit_count_q, tc_dbg_miss_count_q, tc_dbg_hit_rate_q);
    $display("[TC-FINAL] accepted=%0d rejected_not_ready=%0d rejected_used=%0d rejected_policy=%0d accept_per_hit=%0d%%",
             tc_dbg_accept_count_q,
             tc_dbg_reject_not_ready_q, tc_dbg_reject_used_q, tc_dbg_reject_policy_q, tc_dbg_accept_rate_q);
    $display("[TC-V101] path_gate: hits_unvalidated=%0d rejected_by_gate=%0d gate_enabled=%0d",
             tc_dbg_hit_unvalidated_q, tc_dbg_reject_unvalidated_q, !tc_no_path_gate_q);
    $display("[TC-FINAL] multiblock: accepted=%0d rejected_indirect=%0d rejected_singleblock=%0d rejected_short=%0d",
             tc_dbg_accept_multiblock_q, tc_dbg_reject_indirect_q, tc_dbg_reject_singleblock_q, tc_dbg_reject_short_q);
    $display("[TC-FINAL] min_accept_len=%0d", TC_MIN_ACCEPT_LEN);
    $display("[TC-FINAL] tc_hit_cycles=%0d tc_hit_total=%0d tc_pkt_cycles=%0d icache_pkt_cycles=%0d tc_pkt_share=%0d%%",
             tc_dbg_feed_cycles_q, tc_dbg_feed_start_total_q, tc_dbg_tc_pkt_cycles_q, tc_dbg_icache_pkt_cycles_q, tc_dbg_pkt_tc_share_q);
    $display("[TC-FINAL] frontend_instr_mix: tc_instr=%0d icache_instr=%0d tc_instr_share=%0d%%",
             tc_dbg_tc_instr_total_q, tc_dbg_icache_instr_total_q, tc_dbg_instr_tc_share_q);
    $display("[TC-FINAL] frontend_windows: total=%0d unaligned=%0d iq_not_ready=%0d mispredicts=%0d",
             tc_dbg_window_total_q, tc_dbg_window_unaligned_q, tc_dbg_window_not_ready_q,
             tc_dbg_mispredict_count_q);
    $display("[TC-FINAL] window_cf_hist: cf0=%0d cf1=%0d cf2=%0d cf3=%0d cf4p=%0d",
             tc_dbg_window_cf0_q, tc_dbg_window_cf1_q, tc_dbg_window_cf2_q,
             tc_dbg_window_cf3_q, tc_dbg_window_cf4p_q);
    $display("[TC-FINAL] window_taken_hist: tk0=%0d tk1=%0d tk2=%0d tk3=%0d tk4p=%0d",
             tc_dbg_window_taken0_q, tc_dbg_window_taken1_q, tc_dbg_window_taken2_q,
             tc_dbg_window_taken3_q, tc_dbg_window_taken4p_q);
    $display("[TC-FINAL] trace_candidates: single_taken=%0d multi_taken=%0d",
             tc_dbg_window_single_taken_q, tc_dbg_window_multi_taken_q);
    $display("[TC-FINAL] single_taken_kind: cond=%0d jump=%0d call=%0d return=%0d jalr=%0d",
             tc_dbg_trigger_single_cond_q, tc_dbg_trigger_single_jump_q,
             tc_dbg_trigger_single_call_q, tc_dbg_trigger_single_return_q,
             tc_dbg_trigger_single_jalr_q);
    $display("[TC-FINAL] single_taken_blockers: unaligned=%0d iq_not_ready=%0d",
             tc_dbg_trigger_single_unaligned_q, tc_dbg_trigger_single_not_ready_q);
    $display("[TC-FINAL] tc_hit_nonempty_iq=%0d",
             tc_dbg_iq_flush_on_capture_q);
    $display("[TC-FINAL] miss_breakdown: empty=%0d pc=%0d path=%0d",
             tc_dbg_miss_empty_q, tc_dbg_miss_pc_q, tc_dbg_miss_path_q);
    $display("[TC-FINAL] pending_buf: captured=%0d used=%0d invalidated=%0d",
             tc_dbg_pending_capture_q, tc_dbg_pending_use_q, tc_dbg_pending_invalidate_q);
    $display("[TC-FINAL] backend_stall_cycles=%0d iq_empty_cycles=%0d iq_full_cycles=%0d icache_miss_cycles=%0d",
             tc_dbg_backend_stall_q, tc_dbg_iq_empty_cycles_q, tc_dbg_iq_not_ready_cycles_q, tc_dbg_icache_miss_cycles_q);
    $display("[TC-FINAL] replay_profit: 2taken=%0d 3p_taken=%0d",
             tc_dbg_replay_profit_2tk_q, tc_dbg_replay_profit_3tk_q);
    $display("[TC-FINAL] v91_iq_gate_suppress=%0d", tc_dbg_iq_gate_suppress_q);
    $display("[TC-MULTITAKEN] hit_taken_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             tc_dbg_hit_taken_hist_q[0], tc_dbg_hit_taken_hist_q[1],
             tc_dbg_hit_taken_hist_q[2], tc_dbg_hit_taken_hist_q[MAX_TAKEN]);
    $display("[TC-MULTITAKEN] accept_taken_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             tc_dbg_accept_taken_hist_q[0], tc_dbg_accept_taken_hist_q[1],
             tc_dbg_accept_taken_hist_q[2], tc_dbg_accept_taken_hist_q[MAX_TAKEN]);
    $display("[TC-MULTITAKEN] drop_not_ready_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             tc_dbg_not_ready_taken_hist_q[0], tc_dbg_not_ready_taken_hist_q[1],
             tc_dbg_not_ready_taken_hist_q[2], tc_dbg_not_ready_taken_hist_q[MAX_TAKEN]);
    $display("[TC-MULTITAKEN] drop_iq_gate_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             tc_dbg_iq_gate_taken_hist_q[0], tc_dbg_iq_gate_taken_hist_q[1],
             tc_dbg_iq_gate_taken_hist_q[2], tc_dbg_iq_gate_taken_hist_q[MAX_TAKEN]);
    $display("[TC-MULTITAKEN] drop_policy_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             tc_dbg_policy_taken_hist_q[0], tc_dbg_policy_taken_hist_q[1],
             tc_dbg_policy_taken_hist_q[2], tc_dbg_policy_taken_hist_q[MAX_TAKEN]);
    $display("[TC-MULTITAKEN] drop_used_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             tc_dbg_used_taken_hist_q[0], tc_dbg_used_taken_hist_q[1],
             tc_dbg_used_taken_hist_q[2], tc_dbg_used_taken_hist_q[MAX_TAKEN]);
    $display("[TC-FINAL] replay_len_hist: L3=%0d L4=%0d L5=%0d L6=%0d L7=%0d L8=%0d L9=%0d L10p=%0d",
             tc_dbg_replay_len_hist_q[0], tc_dbg_replay_len_hist_q[1], tc_dbg_replay_len_hist_q[2],
             tc_dbg_replay_len_hist_q[3], tc_dbg_replay_len_hist_q[4], tc_dbg_replay_len_hist_q[5],
             tc_dbg_replay_len_hist_q[6], tc_dbg_replay_len_hist_q[7]);
    $display("[TC-FINAL] ========================================");

    // ROI stats
    begin
      int unsigned roi_tc_hit_rate;
      int unsigned roi_tc_accept_rate;
      int unsigned roi_tc_instr_share;
      int unsigned roi_total_instr;
      if (roi_tc_lookup_q > 0)
        roi_tc_hit_rate = (roi_tc_hit_q * 100) / roi_tc_lookup_q;
      else
        roi_tc_hit_rate = 0;
      if (roi_tc_hit_q > 0)
        roi_tc_accept_rate = (roi_tc_accept_q * 100) / roi_tc_hit_q;
      else
        roi_tc_accept_rate = 0;
      roi_total_instr = roi_tc_instr_q + roi_icache_instr_q;
      if (roi_total_instr > 0)
        roi_tc_instr_share = (roi_tc_instr_q * 100) / roi_total_instr;
      else
        roi_tc_instr_share = 0;

      $display("[TC-ROI] ========== ROI Summary ==========");
      $display("[TC-ROI] region=[0x%h, 0x%h) active_cycles=%0d",
               roi_start_pc, roi_end_pc, roi_cycle_count_q);
      $display("[TC-ROI] committed_instr=%0d (from IQ dequeue within ROI)",
               roi_instr_count_q);
      $display("[TC-ROI] tc_lookups=%0d tc_hits=%0d hit_rate=%0d%%",
               roi_tc_lookup_q, roi_tc_hit_q, roi_tc_hit_rate);
      $display("[TC-ROI] tc_accepted=%0d accept_rate=%0d%% (of hits) pending_use=%0d",
               roi_tc_accept_q, roi_tc_accept_rate, roi_tc_pending_use_q);
      $display("[TC-MULTITAKEN-ROI] hit_taken_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
               roi_tc_hit_taken_hist_q[0], roi_tc_hit_taken_hist_q[1],
               roi_tc_hit_taken_hist_q[2], roi_tc_hit_taken_hist_q[MAX_TAKEN]);
      $display("[TC-MULTITAKEN-ROI] accept_taken_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
               roi_tc_accept_taken_hist_q[0], roi_tc_accept_taken_hist_q[1],
               roi_tc_accept_taken_hist_q[2], roi_tc_accept_taken_hist_q[MAX_TAKEN]);
      $display("[TC-ROI] tc_instr=%0d icache_instr=%0d tc_instr_share=%0d%%",
               roi_tc_instr_q, roi_icache_instr_q, roi_tc_instr_share);
      $display("[TC-ROI] miss_breakdown: empty=%0d pc=%0d path=%0d",
               roi_tc_miss_empty_q, roi_tc_miss_pc_q, roi_tc_miss_path_q);
      $display("[TC-ROI] ====================================");
    end

    // Hot-PC frontend summary
    $display("[TC-HOTPC-FE] ========== Hot PC Frontend Lifecycle ==========");
    $display("[TC-HOTPC-FE] HP0(0x80002880): lookup=%0d hit=%0d accept=%0d rej_policy=%0d miss_empty=%0d miss_pc=%0d miss_path=%0d",
             fe_hp_lookup[0], fe_hp_hit[0], fe_hp_accept[0], fe_hp_reject_policy[0],
             fe_hp_miss_empty[0], fe_hp_miss_pc[0], fe_hp_miss_path[0]);
    $display("[TC-HOTPC-FE] HP1(0x8000287a): lookup=%0d hit=%0d accept=%0d rej_policy=%0d miss_empty=%0d miss_pc=%0d miss_path=%0d",
             fe_hp_lookup[1], fe_hp_hit[1], fe_hp_accept[1], fe_hp_reject_policy[1],
             fe_hp_miss_empty[1], fe_hp_miss_pc[1], fe_hp_miss_path[1]);
    $display("[TC-HOTPC-FE] HP2(0x80002888): lookup=%0d hit=%0d accept=%0d rej_policy=%0d miss_empty=%0d miss_pc=%0d miss_path=%0d",
             fe_hp_lookup[2], fe_hp_hit[2], fe_hp_accept[2], fe_hp_reject_policy[2],
             fe_hp_miss_empty[2], fe_hp_miss_pc[2], fe_hp_miss_path[2]);
    $display("[TC-HOTPC-FE] HP3(0x8000287e): lookup=%0d hit=%0d accept=%0d rej_policy=%0d miss_empty=%0d miss_pc=%0d miss_path=%0d",
             fe_hp_lookup[3], fe_hp_hit[3], fe_hp_accept[3], fe_hp_reject_policy[3],
             fe_hp_miss_empty[3], fe_hp_miss_pc[3], fe_hp_miss_path[3]);
    $display("[TC-HOTPC-FE] ================================================");
  end
// pragma translate_on

endmodule
