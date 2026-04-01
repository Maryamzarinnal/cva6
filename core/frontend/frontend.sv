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
  initial $display("[TC-RUN-MARKER] TC_FIX_V56_STABILIZE_PENDING_AND_RAS");
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
  logic [CVA6Cfg.VLEN-1:0]                 replay_addr;

  // -----------------------------------------------------------------------
  // Trace Cache state
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
  logic                                    tc_lookup_cond;

  // TC pending-hit latch: preserve one conservative hit until the instr_queue becomes ready.
  logic                                    tc_pending_q, tc_pending_d;
  logic [TRACE_LEN_WIDTH-1:0]              tc_pending_len_q, tc_pending_len_d;
  logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]   tc_pending_instr_q, tc_pending_instr_d;
  logic [TRACE_LEN-1:0][CVA6Cfg.VLEN-1:0]  tc_pending_pcs_q, tc_pending_pcs_d;
  logic [CVA6Cfg.VLEN-1:0]                 tc_pending_next_pc_q, tc_pending_next_pc_d;
  logic [TRACE_LEN-1:0]                    tc_pending_taken_cf_q, tc_pending_taken_cf_d;
  logic [TRACE_LEN-1:0]                    tc_pending_cf_q, tc_pending_cf_d;
  logic [BR_CNT_WIDTH-1:0]                 tc_pending_num_branches_q, tc_pending_num_branches_d;
  logic [TAKEN_CNT_WIDTH-1:0]              tc_pending_num_taken_q, tc_pending_num_taken_d;
  logic                                    tc_pending_capture;
  logic                                    tc_pending_start;
  logic                                    tc_rehit_block_q, tc_rehit_block_d;
  logic [CVA6Cfg.VLEN-1:0]                 tc_rehit_pc_q, tc_rehit_pc_d;
  logic                                    tc_same_pc_rehit_block;
  logic                                    tc_lookup_oneshot_q, tc_lookup_oneshot_d;
  logic [CVA6Cfg.VLEN-1:0]                 tc_lookup_oneshot_pc_q, tc_lookup_oneshot_pc_d;
  logic                                    tc_same_lookup_oneshot_block;

  // Legacy feeding registers kept inert while we move to one-shot TC pushes.
  logic                                    tc_feeding_q, tc_feeding_d;
  logic [TRACE_LEN_WIDTH-1:0]              tc_feeding_len_q, tc_feeding_len_d;
  logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]   tc_feeding_instr_q, tc_feeding_instr_d;
  logic [TRACE_LEN-1:0][CVA6Cfg.VLEN-1:0]  tc_feeding_pcs_q, tc_feeding_pcs_d;
  logic [CVA6Cfg.VLEN-1:0]                 tc_feeding_next_pc_q, tc_feeding_next_pc_d;
  logic [TRACE_LEN-1:0]                    tc_feeding_consumed_q, tc_feeding_consumed_d;
  logic [TRACE_LEN-1:0]                    tc_feeding_taken_cf_q, tc_feeding_taken_cf_d;
  logic                                    tc_feeding_done;
  logic                                    tc_feeding_start;

  // TC replay signals to instr_queue
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]             replay_instr_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] replay_addr_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                   replay_valid_iq;
  cf_t  [CVA6Cfg.INSTR_PER_FETCH-1:0]                   replay_cf_type_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] replay_predict_addr_iq;
  // MUX outputs to instr_queue
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]             instr_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] addr_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                   valid_to_iq;
  cf_t  [CVA6Cfg.INSTR_PER_FETCH-1:0]                   cf_type_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] predict_addr_to_iq;
  logic [CVA6Cfg.VLEN-1:0]                              replay_predict_addr_single_iq;
  ariane_pkg::frontend_exception_t                      exception_to_iq;
  logic [CVA6Cfg.VLEN-1:0]                              exception_addr_to_iq;
  logic [CVA6Cfg.GPLEN-1:0]                             exception_gpaddr_to_iq;
  logic [31:0]                                          exception_tinst_to_iq;
  logic                                                 exception_gva_to_iq;
  logic                                                 tc_disable_replay_q;

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

  instr_realign #(.CVA6Cfg(CVA6Cfg)) i_instr_realign (
      .clk_i, .rst_ni,
      .flush_i            (icache_dreq_o.kill_s2),
      .valid_i            (icache_valid_q),
      .serving_unaligned_o(serving_unaligned),
      .address_i          (icache_vaddr_q),
      .data_i             (icache_data_q),
      .valid_o            (instruction_valid),
      .addr_o             (addr),
      .instr_o            (instr)
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

  logic bp_valid;
  logic tc_icache_sidefx_en;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] is_branch, is_call, is_jump, is_return, is_jalr;
  logic [CHUNKS_PER_TRACE-1:0] iq_branch_flags_out;
  logic [CHUNKS_PER_TRACE-1:0] tc_active_branch_flags;
  logic [CHUNKS_PER_TRACE-1:0] restored_branch_flags;

  assign tc_icache_sidefx_en = !tc_pending_q && !tc_pending_start;

  for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin
    assign is_branch[i] = tc_icache_sidefx_en & instruction_valid[i] & (rvi_branch[i] | rvc_branch[i]);
    assign is_call[i]   = tc_icache_sidefx_en & instruction_valid[i] & (rvi_call[i]   | rvc_call[i]);
    assign is_return[i] = tc_icache_sidefx_en & instruction_valid[i] & (rvi_return[i] | rvc_return[i]);
    assign is_jump[i]   = tc_icache_sidefx_en & instruction_valid[i] & (rvi_jump[i]   | rvc_jump[i]);
    assign is_jalr[i]   = tc_icache_sidefx_en & instruction_valid[i] & ~is_return[i] & (rvi_jalr[i] | rvc_jalr[i] | rvc_jr[i]);
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
          ras_pop = ras_predict.valid & instr_queue_consumed[i];
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
        ras_push = instr_queue_consumed[i];
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

  // Keep the I$ request side alive while a hit is held in pending. Pending is
  // only a saved TC hit; it must not block natural IQ draining.
  assign icache_dreq_o.req     = instr_queue_ready & ~halt_frontend_i & ~tc_pending_q;
  assign if_ready              = icache_dreq_i.ready & instr_queue_ready & ~halt_frontend_i &
                                 ~tc_pending_q;
  assign icache_dreq_o.kill_s1 = is_mispredict | flush_i | replay | tc_pending_start;
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
  // TC: Present the pending suffix as a one-shot packet into instr_queue.
  // -----------------------------------------------------------------------
  always_comb begin
    automatic cf_t cf_local;

    replay_instr_iq        = '0;
    replay_addr_iq         = '0;
    replay_valid_iq        = '0;
    replay_cf_type_iq      = '{default: ariane_pkg::NoCF};
    replay_predict_addr_iq = '0;

    for (int s = 0; s < TRACE_LEN; s++) begin
      if (s < int'(tc_pending_len_q)) begin
        cf_local = tc_pending_cf_q[s] ?
                   tc_replay_cf_type(tc_pending_instr_q[s]) : ariane_pkg::NoCF;

        replay_valid_iq[s]       = 1'b1;
        replay_instr_iq[s]       = tc_pending_instr_q[s];
        replay_addr_iq[s]        = tc_pending_pcs_q[s];
        replay_cf_type_iq[s]     = cf_local;

        if (cf_local != ariane_pkg::NoCF) begin
          if (s + 1 < int'(tc_pending_len_q))
            replay_predict_addr_iq[s] = tc_pending_pcs_q[s + 1];
          else
            replay_predict_addr_iq[s] = tc_pending_next_pc_q;
        end
      end
    end
  end

  // -----------------------------------------------------------------------
  // TC: one-shot push keeps legacy feeding state inert.
  // -----------------------------------------------------------------------
  always_comb begin
    tc_feeding_consumed_d = '0;
    tc_feeding_done       = tc_pending_start;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_pending_q          <= 1'b0;
      tc_pending_len_q      <= '0;
      tc_pending_instr_q    <= '0;
      tc_pending_pcs_q      <= '0;
      tc_pending_next_pc_q  <= '0;
      tc_pending_taken_cf_q <= '0;
      tc_pending_cf_q       <= '0;
      tc_pending_num_branches_q <= '0;
      tc_pending_num_taken_q <= '0;
      tc_rehit_block_q      <= 1'b0;
      tc_rehit_pc_q         <= '0;
      tc_lookup_oneshot_q   <= 1'b0;
      tc_lookup_oneshot_pc_q <= '0;
      tc_feeding_q          <= 1'b0;
      tc_feeding_len_q      <= '0;
      tc_feeding_instr_q    <= '0;
      tc_feeding_pcs_q      <= '0;
      tc_feeding_next_pc_q  <= '0;
      tc_feeding_consumed_q <= '0;
      tc_feeding_taken_cf_q <= '0;
    end else begin
      tc_pending_q          <= tc_pending_d;
      tc_pending_len_q      <= tc_pending_len_d;
      tc_pending_instr_q    <= tc_pending_instr_d;
      tc_pending_pcs_q      <= tc_pending_pcs_d;
      tc_pending_next_pc_q  <= tc_pending_next_pc_d;
      tc_pending_taken_cf_q <= tc_pending_taken_cf_d;
      tc_pending_cf_q       <= tc_pending_cf_d;
      tc_pending_num_branches_q <= tc_pending_num_branches_d;
      tc_pending_num_taken_q <= tc_pending_num_taken_d;
      tc_rehit_block_q      <= tc_rehit_block_d;
      tc_rehit_pc_q         <= tc_rehit_pc_d;
      tc_lookup_oneshot_q   <= tc_lookup_oneshot_d;
      tc_lookup_oneshot_pc_q <= tc_lookup_oneshot_pc_d;
      tc_feeding_q          <= tc_feeding_d;
      tc_feeding_len_q      <= tc_feeding_len_d;
      tc_feeding_instr_q    <= tc_feeding_instr_d;
      tc_feeding_pcs_q      <= tc_feeding_pcs_d;
      tc_feeding_next_pc_q  <= tc_feeding_next_pc_d;
      tc_feeding_consumed_q <= tc_feeding_consumed_d;
      tc_feeding_taken_cf_q <= tc_feeding_taken_cf_d;
    end
  end

  always_comb begin
    automatic int br_idx;

    tc_pending_d          = tc_pending_q;
    tc_pending_len_d      = tc_pending_len_q;
    tc_pending_instr_d    = tc_pending_instr_q;
    tc_pending_pcs_d      = tc_pending_pcs_q;
    tc_pending_next_pc_d  = tc_pending_next_pc_q;
    tc_pending_taken_cf_d = tc_pending_taken_cf_q;
    tc_pending_cf_d       = tc_pending_cf_q;
    tc_pending_num_branches_d = tc_pending_num_branches_q;
    tc_pending_num_taken_d = tc_pending_num_taken_q;

    // Keep pending trace data across transient ex_valid_i pulses.
    if (flush_i || is_mispredict || set_pc_commit_i || eret_i) begin
      tc_pending_d          = 1'b0;
      tc_pending_taken_cf_d = '0;
      tc_pending_cf_d       = '0;
      tc_pending_num_branches_d = '0;
      tc_pending_num_taken_d = '0;
    end else if (tc_feeding_start) begin
      tc_pending_d          = 1'b0;
      tc_pending_taken_cf_d = '0;
      tc_pending_cf_d       = '0;
      tc_pending_num_branches_d = '0;
      tc_pending_num_taken_d = '0;
    end else if (tc_pending_start) begin
      tc_pending_d          = 1'b0;
      tc_pending_taken_cf_d = '0;
      tc_pending_cf_d       = '0;
      tc_pending_num_branches_d = '0;
      tc_pending_num_taken_d = '0;
    end else if (tc_pending_capture) begin
      tc_pending_d          = 1'b1;
      tc_pending_len_d      = tc_trace_length;
      tc_pending_instr_d    = tc_trace_instructions;
      tc_pending_next_pc_d  = tc_trace_next_pc[CVA6Cfg.VLEN-1:0];
      tc_pending_taken_cf_d = '0;
      tc_pending_cf_d       = '0;
      tc_pending_num_branches_d = tc_trace_num_branches;
      tc_pending_num_taken_d = tc_trace_num_taken;
      br_idx = 0;
      for (int k = 0; k < TRACE_LEN; k++) begin
        tc_pending_pcs_d[k] = tc_trace_pcs[k][CVA6Cfg.VLEN-1:0];

        if ((k < int'(tc_trace_length)) &&
            (tc_replay_cf_type(tc_trace_instructions[k]) != ariane_pkg::NoCF) &&
            (br_idx < int'(tc_trace_num_branches))) begin
          tc_pending_cf_d[k] = 1'b1;
          tc_pending_taken_cf_d[k] = tc_trace_branch_flags[br_idx];
          br_idx++;
        end
      end

    end
  end

  always_comb begin
    tc_rehit_block_d = tc_rehit_block_q;
    tc_rehit_pc_d    = tc_rehit_pc_q;

    // Keep the replay re-hit lock alive across ordinary ex_valid_i traffic.
    // ex_valid_i can pulse during normal backend progress and was clearing the
    // guard before the stale lookup result for the just-launched trace vanished.
    // Keep the lock across mispredict redirects; otherwise the same stale hit
    // can relaunch immediately and livelock replay progress.
    if (flush_i || set_pc_commit_i || eret_i) begin
      tc_rehit_block_d = 1'b0;
      tc_rehit_pc_d    = '0;
    end else begin
      if (tc_feeding_start) begin
        tc_rehit_block_d = 1'b1;
        tc_rehit_pc_d    = tc_pending_start ? tc_pending_pcs_q[0] : tc_trace_pcs[0][CVA6Cfg.VLEN-1:0];
      end else if (tc_rehit_block_q &&
                   (trace_cache_pkg::pc_align_16(tc_pc[0]) != trace_cache_pkg::pc_align_16(tc_rehit_pc_q))) begin
        tc_rehit_block_d = 1'b0;
      end
    end
  end

  // Block both:
  // 1) a new live frontend revisit of the same trace-start PC, and
  // 2) a delayed lookup result that comes back for the same trace-start PC after
  //    replay already started. Case (2) is the one that creates FEED-START #2.
  assign tc_same_pc_rehit_block = tc_rehit_block_q &&
                                  (((tc_lookup_result_valid && tc_trace_hit) &&
                                    (trace_cache_pkg::pc_align_16(tc_trace_pcs[0][CVA6Cfg.VLEN-1:0]) ==
                                     trace_cache_pkg::pc_align_16(tc_rehit_pc_q))) ||
                                   ((!tc_lookup_result_valid) &&
                                    (trace_cache_pkg::pc_align_16(tc_pc[0]) ==
                                     trace_cache_pkg::pc_align_16(tc_rehit_pc_q))));

  always_comb begin
    tc_lookup_oneshot_d    = tc_lookup_oneshot_q;
    tc_lookup_oneshot_pc_d = tc_lookup_oneshot_pc_q;

    // Clear the one-shot lookup guard only on architectural redirects.
    // Keep one-shot guard across mispredict for the same reason as rehit lock.
    if (flush_i || set_pc_commit_i || eret_i) begin
      tc_lookup_oneshot_d    = 1'b0;
      tc_lookup_oneshot_pc_d = '0;
    end else begin
      // Lock the launched trace-start PC exactly when replay begins, then keep
      // blocking identical lookup results until that result disappears or changes
      // to a different start PC. This closes the window where tc_feeding_q drops
      // after the first replay packet but the old lookup result is still visible.
      if (tc_feeding_start) begin
        tc_lookup_oneshot_d    = 1'b1;
        tc_lookup_oneshot_pc_d = tc_pending_start ? tc_pending_pcs_q[0]
                                                  : tc_trace_pcs[0][CVA6Cfg.VLEN-1:0];
      end else if (tc_lookup_oneshot_q &&
                   (!tc_lookup_result_valid || !tc_trace_hit ||
                    (trace_cache_pkg::pc_align_16(tc_trace_pcs[0][CVA6Cfg.VLEN-1:0]) !=
                     trace_cache_pkg::pc_align_16(tc_lookup_oneshot_pc_q)))) begin
        tc_lookup_oneshot_d    = 1'b0;
        tc_lookup_oneshot_pc_d = '0;
      end
    end
  end

  assign tc_same_lookup_oneshot_block = tc_lookup_oneshot_q &&
                                        tc_lookup_result_valid && tc_trace_hit &&
                                        (trace_cache_pkg::pc_align_16(tc_trace_pcs[0][CVA6Cfg.VLEN-1:0]) ==
                                         trace_cache_pkg::pc_align_16(tc_lookup_oneshot_pc_q));

  assign tc_feeding_start = tc_pending_start;

  always_comb begin
    tc_feeding_d          = 1'b0;
    tc_feeding_len_d      = '0;
    tc_feeding_instr_d    = '0;
    tc_feeding_pcs_d      = '0;
    tc_feeding_next_pc_d  = '0;
    tc_feeding_taken_cf_d = '0;
  end

  // -----------------------------------------------------------------------
  // MUX: TC feeding vs normal I-cache
  // -----------------------------------------------------------------------
  assign instr_to_iq            = tc_pending_start ? replay_instr_iq   : instr;
  assign addr_to_iq             = tc_pending_start ? replay_addr_iq    : addr;
  assign valid_to_iq            = tc_pending_start ? replay_valid_iq
                                                   : (tc_pending_q ? '0 : instruction_valid);
  assign cf_type_to_iq          = tc_pending_start ? replay_cf_type_iq : cf_type;
  assign exception_to_iq        = tc_pending_start ? ariane_pkg::FE_NONE : icache_ex_valid_q;
  assign exception_addr_to_iq   = tc_pending_start ? '0 : icache_vaddr_q;
  assign exception_gpaddr_to_iq = tc_pending_start ? '0 : icache_gpaddr_q;
  assign exception_tinst_to_iq  = tc_pending_start ? '0 : icache_tinst_q;
  assign exception_gva_to_iq    = tc_pending_start ? 1'b0 : icache_gva_q;
  always_comb begin
    replay_predict_addr_single_iq = '0;
    for (int r = 0; r < CVA6Cfg.INSTR_PER_FETCH; r++) begin
      if (replay_valid_iq[r] && (replay_cf_type_iq[r] != ariane_pkg::NoCF))
        replay_predict_addr_single_iq = replay_predict_addr_iq[r];
    end
  end

  for (genvar gi = 0; gi < CVA6Cfg.INSTR_PER_FETCH; gi++) begin : gen_predict_addr_mux
    assign predict_addr_to_iq[gi] = tc_pending_start ? replay_predict_addr_single_iq : predict_address;
  end

  always_comb begin
    tc_active_branch_flags = '0;
    if (tc_pending_start) begin
      for (int bf = 0; bf < TRACE_LEN; bf++)
        tc_active_branch_flags[bf] = tc_pending_taken_cf_q[bf];
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

    if (tc_pending_start)
      npc_d = tc_pending_next_pc_q;

    if (replay)          npc_d = replay_addr;
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
      icache_valid_q <= icache_dreq_i.valid & tc_icache_sidefx_en;

      if (icache_dreq_i.valid && tc_icache_sidefx_en) begin
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
        .push_i(ras_push),
        .pop_i(ras_pop),
        .data_i(ras_update),
        .data_o(ras_predict)
    );
  end

  assign vpc_btb = (CVA6Cfg.FpgaEn) ? icache_dreq_i.vaddr : icache_vaddr_q;
  assign vpc_bht = (CVA6Cfg.FpgaEn && CVA6Cfg.FpgaAlteraEn && icache_dreq_i.valid) ? icache_dreq_i.vaddr : icache_vaddr_q;

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
        .bht_prediction_o(bht_prediction)
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
      .fetch_entry_t(fetch_entry_t)
  ) i_instr_queue (
      .clk_i,
      .rst_ni,
      // Keep frontend queue state aligned with architectural redirects.
      // In practice we observed low-PC leakage right after resume/redirect
      // windows; flushing/reseeding the queue on these events prevents stale
      // pc/address FIFO state from escaping into fetch_entry_o.
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
      .branch_flags_i     (tc_active_branch_flags),
      .cf_type_i          (cf_type_to_iq),
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
      .tc_feeding_i       (tc_pending_start),
      .reseed_pc_i        (tc_pending_start)
  );

  // -----------------------------------------------------------------------
  // Trace Cache signals for recording
  // -----------------------------------------------------------------------
  logic [SLOTS_PER_CYCLE-1:0]               tc_instr_valid;
  logic [SLOTS_PER_CYCLE-1:0][31:0]         tc_instr;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] tc_pc;
  logic [SLOTS_PER_CYCLE-1:0]               tc_is_branch;
  logic [SLOTS_PER_CYCLE-1:0]               tc_raw_taken;
  logic [SLOTS_PER_CYCLE-1:0]               tc_taken;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] tc_target;
  logic [CHUNKS_PER_TRACE-1:0]              tc_branch_predictions;
  logic [CHUNKS_PER_TRACE-1:0]              tc_lookup_branch_predictions;
  logic [BR_CNT_WIDTH-1:0]                  tc_lookup_num_branches;
  logic                                     tc_window_eligible;
  logic                                     tc_record_enable;

  assign tc_window_eligible = !serving_unaligned && !halt_i && !halt_frontend_i && !debug_mode_i;
  assign tc_record_enable = tc_window_eligible && !tc_pending_q && !tc_pending_start
                          && addr[0][31];

  for (genvar i = 0; i < SLOTS_PER_CYCLE; i++) begin : gen_tc_signals
    assign tc_instr_valid[i] = instruction_valid[i] & ~flush_i & tc_record_enable;
    assign tc_pc[i]          = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, addr[i]};
    assign tc_instr[i]       = instr[i];
    assign tc_is_branch[i]   = is_branch[i] | is_jump[i] | is_jalr[i] | is_return[i] | is_call[i];
    assign tc_raw_taken[i]   = ((taken_rvi_cf[i] | taken_rvc_cf[i]) | is_jump[i] | is_call[i] |
                                (is_jalr[i] & btb_prediction_shifted[i].valid) |
                                (is_return[i] & ras_predict.valid)) & tc_instr_valid[i];

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

  logic consumed_has_taken_branch;
  always_comb begin
    consumed_has_taken_branch = 1'b0;
    for (int i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin
      if (instr_queue_consumed[i] && tc_is_branch[i] && tc_taken[i])
        consumed_has_taken_branch = 1'b1;
    end
  end

  assign tc_lookup_cond = (|instr_queue_consumed) && consumed_has_taken_branch &&
                          tc_record_enable;

  always_comb begin
    integer br_idx;
    tc_branch_predictions = '0;
    tc_lookup_num_branches = '0;
    br_idx = 0;

    for (int i = 0; i < SLOTS_PER_CYCLE && br_idx < CHUNKS_PER_TRACE; i++) begin
      if (instr_queue_consumed[i] && tc_is_branch[i]) begin
        tc_lookup_num_branches = tc_lookup_num_branches + BR_CNT_WIDTH'(1);

        if (is_jump[i] || is_call[i])
          tc_branch_predictions[br_idx] = 1'b1;
        else if (is_return[i])
          tc_branch_predictions[br_idx] = ras_predict.valid;
        else if (is_jalr[i])
          tc_branch_predictions[br_idx] = btb_prediction_shifted[i].valid;
        else
          tc_branch_predictions[br_idx] = bht_prediction_shifted[i].valid ?
                                          bht_prediction_shifted[i].taken :
                                          (rvi_branch[i] ? rvi_imm[i][CVA6Cfg.VLEN-1] :
                                                           rvc_imm[i][CVA6Cfg.VLEN-1]);
        br_idx++;
      end
    end
  end

  assign tc_lookup_branch_predictions = is_mispredict ? restored_branch_flags
                                                       : tc_branch_predictions;

  logic [PC_WIDTH-1:0] tc_lookup_pc_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      tc_lookup_pc_q <= '0;
    else if (tc_lookup_cond && !flush_i)
      tc_lookup_pc_q <= tc_pc[0];
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
      .instr_queue_ready_i            (instr_queue_ready && tc_record_enable),
      .instr_queue_consumed_i         (tc_record_enable ? instr_queue_consumed : '0),
      .branch_predictions_i           (tc_record_enable ? tc_lookup_branch_predictions : '0),
      .lookup_num_branches_i          (tc_record_enable ? tc_lookup_num_branches : '0),
      .resolved_branch_valid_i        (resolved_branch_i.valid),
      .resolved_branch_pc_i           (resolved_branch_i.pc),
      .resolved_branch_is_taken_i     (resolved_branch_i.is_taken),
      .resolved_branch_is_mispredict_i(resolved_branch_i.is_mispredict),
      .lookup_valid_i                 (tc_lookup_cond),
      .lookup_pc_i                    (tc_pc[0]),
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
      .mark_used_i                    (tc_pending_capture)
  );

  logic tc_trace_policy_ok;

`ifndef SYNTHESIS
  initial begin
    tc_disable_replay_q = $test$plusargs("TC_DISABLE_REPLAY");
  end

  initial begin
    $display("[TC-RUN-MARKER] TC_FIX_V56_STABILIZE_PENDING_AND_RAS");
    if (tc_disable_replay_q)
      $display("[TC-RUN-MARKER] TC_SWITCH_DISABLE_REPLAY_ACTIVE");
  end
`else
  assign tc_disable_replay_q = 1'b0;
`endif

  assign tc_active_hit = tc_lookup_result_valid && tc_trace_hit &&
                         (tc_trace_length != '0) && !flush_i && !is_mispredict;

  // Replay policy for phase-1 direct multi-block traces:
  // - the trigger-window branch stays on the normal frontend path
  // - allow direct branches/jumps inside the payload
  // - still reject indirect CFs and calls, because the current replay path
  //   still does not preserve those side effects cleanly
  assign tc_trace_policy_ok = tc_trace_starts_ok &&
                              tc_trace_branch_map_ok &&
                              (tc_trace_num_taken <= TAKEN_CNT_WIDTH'(1)) &&
                              !tc_trace_has_call &&
                              !tc_trace_has_indirect &&
                              !tc_disable_replay_q;

  assign tc_pending_capture = tc_active_hit &&
                              tc_trace_policy_ok &&
                              !tc_pending_q &&
                              !tc_trace_used &&
                              !tc_same_pc_rehit_block &&
                              !tc_same_lookup_oneshot_block;

  assign tc_pending_start = tc_pending_q &&
                            instr_queue_ready &&
                            instr_queue_empty &&
                            !flush_i &&
                            !is_mispredict;

  assign tc_active_use = 1'b0;

// pragma translate_off
  longint unsigned tc_total_cycles_q;
  int unsigned tc_dbg_lookup_count_q;
  int unsigned tc_dbg_hit_count_q;
  int unsigned tc_dbg_accept_count_q;
  int unsigned tc_dbg_reject_not_ready_q;
  int unsigned tc_dbg_reject_used_q;
  int unsigned tc_dbg_reject_indirect_q;
  int unsigned tc_dbg_reject_policy_q;
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

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_total_cycles_q         <= 0;
      tc_dbg_lookup_count_q     <= 0;
      tc_dbg_hit_count_q        <= 0;
      tc_dbg_accept_count_q     <= 0;
      tc_dbg_reject_not_ready_q <= 0;
      tc_dbg_reject_used_q      <= 0;
      tc_dbg_reject_indirect_q  <= 0;
      tc_dbg_reject_policy_q    <= 0;
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
    end else begin
      int unsigned src_cnt;
      int unsigned cf_cnt;
      int unsigned taken_cnt;
      int unsigned first_taken_idx;
      tc_total_cycles_q <= tc_total_cycles_q + 1;

      if (resolved_branch_i.valid && resolved_branch_i.is_mispredict)
        tc_dbg_mispredict_count_q <= tc_dbg_mispredict_count_q + 1;

      if (tc_lookup_result_valid) begin
        tc_dbg_lookup_count_q <= tc_dbg_lookup_count_q + 1;

        if (tc_trace_hit) begin
          tc_dbg_hit_count_q <= tc_dbg_hit_count_q + 1;

          if (tc_trace_policy_ok) begin
            if (tc_trace_used)
              tc_dbg_reject_used_q <= tc_dbg_reject_used_q + 1;
            else if (!tc_pending_capture)
              tc_dbg_reject_not_ready_q <= tc_dbg_reject_not_ready_q + 1;
          end else begin
            if (tc_trace_has_indirect || tc_trace_has_call)
              tc_dbg_reject_indirect_q <= tc_dbg_reject_indirect_q + 1;
            tc_dbg_reject_policy_q <= tc_dbg_reject_policy_q + 1;
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
      end

      if (tc_pending_q)
        tc_dbg_feed_cycles_q <= tc_dbg_feed_cycles_q + 1;

      if (tc_feeding_start)
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

      if ((|instruction_valid) && addr[0][31] && serving_unaligned &&
          !tc_pending_q && !tc_pending_start)
        tc_dbg_window_unaligned_q <= tc_dbg_window_unaligned_q + 1;

      if ((|instruction_valid) && addr[0][31] && tc_window_eligible &&
          !instr_queue_ready && !tc_pending_q && !tc_pending_start)
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
          $display("[TC-PC-FLOOR-BREACH] t=%0t pc=0x%h replay=%0b replay_addr=0x%h iq_ready=%0b tc_feeding=%0b pending=%0b",
                   $time, fetch_entry_o[0].address, replay, replay_addr, instr_queue_ready, tc_feeding_q, tc_pending_q);
        end
      end

      if (tc_pending_start && (|replay_valid_iq)) begin
        tc_dbg_tc_pkt_cycles_q <= tc_dbg_tc_pkt_cycles_q + 1;
        src_cnt = 0;
        for (int k = 0; k < CVA6Cfg.INSTR_PER_FETCH; k++)
          if (replay_valid_iq[k]) src_cnt++;
        tc_dbg_tc_instr_total_q <= tc_dbg_tc_instr_total_q + src_cnt;
      end

      if (tc_pending_capture)
        tc_dbg_hold_count_q <= tc_dbg_hold_count_q + 1;

      if (tc_pending_start) begin
        tc_dbg_pending_use_count_q <= tc_dbg_pending_use_count_q + 1;
        tc_dbg_accept_count_q <= tc_dbg_accept_count_q + 1;
        if (tc_pending_num_branches_q != BR_CNT_WIDTH'(0))
          tc_dbg_accept_multiblock_q <= tc_dbg_accept_multiblock_q + 1;
      end

      if (tc_feeding_done) begin
        tc_dbg_feed_done_q <= tc_dbg_feed_done_q + 1;
        if (tc_dbg_feed_done_q < 256) begin
          $display("[TC-FEED-DONE] t=%0t len_q=%0d iq_cons=%b iq_ready=%0b iq_empty=%0b next_pc=0x%h",
                   $time, tc_pending_len_q, instr_queue_consumed, instr_queue_ready,
                   instr_queue_empty, tc_pending_next_pc_q);
        end
      end

      if (tc_feeding_start && (tc_dbg_feed_start_count_q < 96)) begin
        tc_dbg_feed_start_count_q <= tc_dbg_feed_start_count_q + 1;
        if (tc_pending_start) begin
          $display("[TC-PENDING-USE] t=%0t pc0=0x%h len=%0d num_taken=%0d iq_ready=%0b iq_empty=%0b",
                   $time, tc_pending_pcs_q[0], tc_pending_len_q, tc_pending_num_taken_q, instr_queue_ready, instr_queue_empty);
          $display("[TC-FEED-START] #%0d t=%0t len=%0d next_pc=0x%h src0_pc=0x%h num_br=%0d num_taken=%0d",
                   tc_dbg_feed_start_count_q + 1, $time, tc_pending_len_q, tc_pending_next_pc_q,
                   tc_pending_pcs_q[0], tc_pending_num_branches_q, tc_pending_num_taken_q);
          for (int k = 0; k < TRACE_LEN; k++) begin
            if (k < int'(tc_pending_len_q)) begin
              $display("[TC-FEED-START]   src[%0d] pc=0x%h instr=0x%08h cf=%0d taken_cf=%0b",
                       k, tc_pending_pcs_q[k], tc_pending_instr_q[k],
                       tc_replay_cf_type(tc_pending_instr_q[k]), tc_pending_taken_cf_q[k]);
            end
          end
        end
      end

      if (tc_pending_start && (|replay_valid_iq) && (tc_dbg_replay_pkt_count_q < 256)) begin
        tc_dbg_replay_pkt_count_q <= tc_dbg_replay_pkt_count_q + 1;
        $display("[TC-REPLAY-PKT] #%0d t=%0t len_q=%0d iq_cons=%b iq_ready=%0b iq_empty=%0b",
                 tc_dbg_replay_pkt_count_q + 1, $time, tc_pending_len_q, instr_queue_consumed,
                 instr_queue_ready, instr_queue_empty);
        for (int k = 0; k < CVA6Cfg.INSTR_PER_FETCH; k++) begin
          if (replay_valid_iq[k]) begin
            $display("[TC-REPLAY-PKT]   dst[%0d]->src[%0d] pc=0x%h instr=0x%08h cf=%0d pred=0x%h",
                     k, k, replay_addr_iq[k], replay_instr_iq[k],
                     replay_cf_type_iq[k], replay_predict_addr_iq[k]);
          end
        end
      end

      if (tc_lookup_result_valid && (tc_dbg_lookup_count_q < 80)) begin
        if (tc_trace_hit) begin
          if (tc_pending_capture) begin
            $display("[TC-HIT-HOLD] t=%0t pc0=0x%h len=%0d num_taken=%0d last_taken_last=%0b iq_ready=%0b iq_empty=%0b",
                     $time, tc_trace_pcs[0], tc_trace_length, tc_trace_num_taken, tc_trace_last_is_taken_cf, instr_queue_ready, instr_queue_empty);
          end else if (tc_same_pc_rehit_block) begin
            $display("[TC-REHIT-BLOCK] t=%0t pc0=0x%h len=%0d num_taken=%0d iq_ready=%0b iq_empty=%0b lock_pc=0x%h",
                     $time, tc_trace_pcs[0], tc_trace_length, tc_trace_num_taken,
                     instr_queue_ready, instr_queue_empty, tc_rehit_pc_q);
          end else begin
            $display("[TC-HIT-DROP] t=%0t pc0=0x%h len=%0d num_taken=%0d starts_ok=%0b last_taken_last=%0b iq_ready=%0b iq_empty=%0b pending=%0b feeding=%0b flush=%0b mispredict=%0b",
                     $time, tc_trace_pcs[0], tc_trace_length, tc_trace_num_taken,
                     tc_trace_starts_ok, tc_trace_last_is_taken_cf, instr_queue_ready,
                     instr_queue_empty, tc_pending_q, tc_feeding_q, flush_i, is_mispredict);
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

    $display("[TC-FINAL] ========== Trace Cache summary ==========");
    $display("[TC-FINAL] lookups=%0d hits=%0d misses=%0d hit_rate=%0d%%",
             tc_dbg_lookup_count_q, tc_dbg_hit_count_q, tc_dbg_miss_count_q, tc_dbg_hit_rate_q);
    $display("[TC-FINAL] accepted=%0d held=%0d pending_use=%0d rejected_not_ready=%0d rejected_used=%0d rejected_policy=%0d accept_per_hit=%0d%%",
             tc_dbg_accept_count_q, tc_dbg_hold_count_q, tc_dbg_pending_use_count_q,
             tc_dbg_reject_not_ready_q, tc_dbg_reject_used_q, tc_dbg_reject_policy_q, tc_dbg_accept_rate_q);
    $display("[TC-FINAL] multiblock: accepted=%0d rejected_indirect=%0d",
             tc_dbg_accept_multiblock_q, tc_dbg_reject_indirect_q);
    $display("[TC-FINAL] immediate_use=%0d pending_enabled=%0b",
             tc_dbg_immediate_use_count_q, 1'b1);
    $display("[TC-FINAL] replay_done=%0d replay_cycles=%0d replay_cycle_share=%0d%%",
             tc_dbg_feed_done_q, tc_dbg_feed_cycles_q, tc_dbg_feed_pct_q);
    $display("[TC-FINAL] frontend_use=%0d tc_pkt_cycles=%0d icache_pkt_cycles=%0d tc_pkt_share=%0d%%",
             tc_dbg_feed_start_total_q, tc_dbg_tc_pkt_cycles_q, tc_dbg_icache_pkt_cycles_q, tc_dbg_pkt_tc_share_q);
    $display("[TC-FINAL] frontend_instr_mix: tc_instr=%0d icache_instr=%0d tc_instr_share=%0d%% icache_blocked_cycles=%0d",
             tc_dbg_tc_instr_total_q, tc_dbg_icache_instr_total_q, tc_dbg_instr_tc_share_q,
             tc_dbg_icache_block_cycles_q);
    $display("[TC-FINAL] pc_floor_breach=%0d (pc<0x80000000 after entering coremark region)",
             tc_dbg_pc_floor_breach_q);
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
    $display("[TC-FINAL] miss_breakdown: empty=%0d pc=%0d path=%0d",
             tc_dbg_miss_empty_q, tc_dbg_miss_pc_q, tc_dbg_miss_path_q);
    $display("[TC-FINAL] ========================================");
  end
// pragma translate_on

endmodule
