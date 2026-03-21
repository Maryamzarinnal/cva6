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
  logic                                    tc_active_use;
  logic                                    tc_active_hit;
  logic                                    tc_trace_starts_ok;
  logic                                    tc_trace_last_is_taken_cf;

  // TC feeding registers
  logic                                    tc_feeding_q, tc_feeding_d;
  logic [TRACE_LEN_WIDTH-1:0]              tc_feeding_len_q, tc_feeding_len_d;
  logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]   tc_feeding_instr_q, tc_feeding_instr_d;
  logic [TRACE_LEN-1:0][CVA6Cfg.VLEN-1:0]  tc_feeding_pcs_q, tc_feeding_pcs_d;
  logic [CVA6Cfg.VLEN-1:0]                 tc_feeding_next_pc_q, tc_feeding_next_pc_d;
  logic [TRACE_LEN-1:0]                    tc_feeding_consumed_q, tc_feeding_consumed_d;
  logic                                    tc_feeding_done;
  logic                                    tc_feeding_start;

  // TC replay signals to instr_queue
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]             replay_instr_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] replay_addr_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                   replay_valid_iq;
  cf_t  [CVA6Cfg.INSTR_PER_FETCH-1:0]                   replay_cf_type_iq;
  logic [CVA6Cfg.VLEN-1:0]                              replay_predict_addr_slot0;

  // MUX outputs to instr_queue
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0]             instr_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] addr_to_iq;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0]                   valid_to_iq;
  cf_t  [CVA6Cfg.INSTR_PER_FETCH-1:0]                   cf_type_to_iq;
  logic [CVA6Cfg.VLEN-1:0]                              predict_addr_to_iq;
  ariane_pkg::frontend_exception_t                      exception_to_iq;
  logic [CVA6Cfg.VLEN-1:0]                              exception_addr_to_iq;
  logic [CVA6Cfg.GPLEN-1:0]                             exception_gpaddr_to_iq;
  logic [31:0]                                          exception_tinst_to_iq;
  logic                                                 exception_gva_to_iq;

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
        if ((instr_i[15:13] == riscv::OpcodeC1Beqz) ||
            (instr_i[15:13] == riscv::OpcodeC1Bnez)) begin
          tc_replay_cf_type = ariane_pkg::Branch;
        end else if ((instr_i[15:13] == riscv::OpcodeC1J) ||
                     ((CVA6Cfg.XLEN == 32) && (instr_i[15:13] == riscv::OpcodeC1Jal))) begin
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
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] is_branch, is_call, is_jump, is_return, is_jalr;

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
          ras_pop = ras_predict.valid & instr_queue_consumed[i];
          ras_push = 1'b0;
          predict_address = ras_predict.ra;
          cf_type[i] = ariane_pkg::Return;
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

  assign icache_dreq_o.req     = instr_queue_ready & ~halt_frontend_i & ~tc_feeding_q;
  assign if_ready              = icache_dreq_i.ready & instr_queue_ready & ~halt_frontend_i & ~tc_feeding_q;
  assign icache_dreq_o.kill_s1 = is_mispredict | flush_i | replay;
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
    int cf_before_last;

    tc_trace_last_is_taken_cf = 1'b0;
    last_idx = 0;
    cf_before_last = 0;

    if (tc_trace_length != 0) begin
      last_idx = int'(tc_trace_length) - 1;

      for (int i = 0; i < TRACE_LEN; i++) begin
        if ((i < last_idx) && (tc_replay_cf_type(tc_trace_instructions[i]) != ariane_pkg::NoCF))
          cf_before_last++;
      end

      if ((tc_replay_cf_type(tc_trace_instructions[last_idx]) != ariane_pkg::NoCF) &&
          (cf_before_last < CHUNKS_PER_TRACE) &&
          tc_trace_branch_flags[cf_before_last]) begin
        tc_trace_last_is_taken_cf = 1'b1;
      end
    end
  end

  // -----------------------------------------------------------------------
  // TC: Present trace to original instr_queue
  // Conservative policy:
  // - only one taken CF in the whole trace
  // - that taken CF must be the last valid trace instruction
  // -----------------------------------------------------------------------
  always_comb begin
    replay_instr_iq           = '0;
    replay_addr_iq            = '0;
    replay_valid_iq           = '0;
    replay_cf_type_iq         = '{default: ariane_pkg::NoCF};
    replay_predict_addr_slot0 = tc_feeding_next_pc_q;

    for (int s = 0; s < CVA6Cfg.INSTR_PER_FETCH; s++) begin
      if ((s < int'(tc_feeding_len_q)) && !tc_feeding_consumed_q[s]) begin
        replay_valid_iq[s] = 1'b1;
        replay_instr_iq[s] = tc_feeding_instr_q[s];
        replay_addr_iq[s]  = tc_feeding_pcs_q[s];

        if (s == int'(tc_feeding_len_q) - 1)
          replay_cf_type_iq[s] = tc_replay_cf_type(tc_feeding_instr_q[s]);
      end
    end
  end

  // -----------------------------------------------------------------------
  // TC: Consumed mask + feeding done
  // -----------------------------------------------------------------------
  always_comb begin
    tc_feeding_consumed_d = tc_feeding_consumed_q;
    tc_feeding_done       = 1'b0;

    if (flush_i || is_mispredict || set_pc_commit_i || ex_valid_i || eret_i) begin
      tc_feeding_consumed_d = '0;
    end else begin
      if (tc_feeding_start)
        tc_feeding_consumed_d = '0;

      if (tc_feeding_q) begin
        for (int j = 0; j < TRACE_LEN; j++) begin
          if ((j < int'(tc_feeding_len_q)) && instr_queue_consumed[j])
            tc_feeding_consumed_d[j] = 1'b1;
        end

        tc_feeding_done = 1'b1;
        for (int j = 0; j < TRACE_LEN; j++) begin
          if ((j < int'(tc_feeding_len_q)) && !tc_feeding_consumed_d[j])
            tc_feeding_done = 1'b0;
        end

        if (tc_feeding_done)
          tc_feeding_consumed_d = '0;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_feeding_q          <= 1'b0;
      tc_feeding_len_q      <= '0;
      tc_feeding_instr_q    <= '0;
      tc_feeding_pcs_q      <= '0;
      tc_feeding_next_pc_q  <= '0;
      tc_feeding_consumed_q <= '0;
    end else begin
      tc_feeding_q          <= tc_feeding_d;
      tc_feeding_len_q      <= tc_feeding_len_d;
      tc_feeding_instr_q    <= tc_feeding_instr_d;
      tc_feeding_pcs_q      <= tc_feeding_pcs_d;
      tc_feeding_next_pc_q  <= tc_feeding_next_pc_d;
      tc_feeding_consumed_q <= tc_feeding_consumed_d;
    end
  end

  assign tc_feeding_start = tc_active_use && !tc_feeding_q;

  always_comb begin
    tc_feeding_d         = tc_feeding_q;
    tc_feeding_len_d     = tc_feeding_len_q;
    tc_feeding_instr_d   = tc_feeding_instr_q;
    tc_feeding_pcs_d     = tc_feeding_pcs_q;
    tc_feeding_next_pc_d = tc_feeding_next_pc_q;

    if (flush_i || is_mispredict || set_pc_commit_i || ex_valid_i || eret_i)
      tc_feeding_d = 1'b0;
    else if (tc_feeding_done)
      tc_feeding_d = 1'b0;
    else if (tc_feeding_start) begin
      tc_feeding_d         = 1'b1;
      tc_feeding_len_d     = tc_trace_length;
      tc_feeding_instr_d   = tc_trace_instructions;
      tc_feeding_next_pc_d = tc_trace_next_pc[CVA6Cfg.VLEN-1:0];
      for (int k = 0; k < TRACE_LEN; k++)
        tc_feeding_pcs_d[k] = tc_trace_pcs[k][CVA6Cfg.VLEN-1:0];
    end
  end

  // -----------------------------------------------------------------------
  // MUX: TC feeding vs normal I-cache
  // -----------------------------------------------------------------------
  assign instr_to_iq            = tc_feeding_q ? replay_instr_iq   : instr;
  assign addr_to_iq             = tc_feeding_q ? replay_addr_iq    : addr;
  assign valid_to_iq            = tc_feeding_q ? replay_valid_iq   : instruction_valid;
  assign cf_type_to_iq          = tc_feeding_q ? replay_cf_type_iq : cf_type;
  assign exception_to_iq        = tc_feeding_q ? ariane_pkg::FE_NONE : icache_ex_valid_q;
  assign exception_addr_to_iq   = tc_feeding_q ? '0 : icache_vaddr_q;
  assign exception_gpaddr_to_iq = tc_feeding_q ? '0 : icache_gpaddr_q;
  assign exception_tinst_to_iq  = tc_feeding_q ? '0 : icache_tinst_q;
  assign exception_gva_to_iq    = tc_feeding_q ? 1'b0 : icache_gva_q;
  assign predict_addr_to_iq     = tc_feeding_q ? replay_predict_addr_slot0 : predict_address;

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

    if (tc_feeding_done && tc_feeding_q)
      npc_d = tc_feeding_next_pc_q;

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
      icache_valid_q <= icache_dreq_i.valid;

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
      .flush_i            (flush_i),
      .instr_i            (instr_to_iq),
      .addr_i             (addr_to_iq),
      .exception_i        (exception_to_iq),
      .exception_addr_i   (exception_addr_to_iq),
      .exception_gpaddr_i (exception_gpaddr_to_iq),
      .exception_tinst_i  (exception_tinst_to_iq),
      .exception_gva_i    (exception_gva_to_iq),
      .predict_address_i  (predict_addr_to_iq),
      .cf_type_i          (cf_type_to_iq),
      .valid_i            (valid_to_iq),
      .consumed_o         (instr_queue_consumed),
      .ready_o            (instr_queue_ready),
      .replay_o           (replay),
      .replay_addr_o      (replay_addr),
      .fetch_entry_o      (fetch_entry_o),
      .fetch_entry_valid_o(fetch_entry_valid_o),
      .fetch_entry_ready_i(fetch_entry_ready_i)
  );

  // -----------------------------------------------------------------------
  // Trace Cache signals for recording
  // -----------------------------------------------------------------------
  logic [SLOTS_PER_CYCLE-1:0]               tc_instr_valid;
  logic [SLOTS_PER_CYCLE-1:0][31:0]         tc_instr;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] tc_pc;
  logic [SLOTS_PER_CYCLE-1:0]               tc_is_branch;
  logic [SLOTS_PER_CYCLE-1:0]               tc_taken;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] tc_target;
  logic [CHUNKS_PER_TRACE-1:0]              tc_branch_predictions;
  logic [BR_CNT_WIDTH-1:0]                  tc_lookup_num_branches;
  logic                                     tc_window_eligible;

  assign tc_window_eligible = !serving_unaligned && !halt_i && !halt_frontend_i && !debug_mode_i;

  for (genvar i = 0; i < SLOTS_PER_CYCLE; i++) begin : gen_tc_signals
    assign tc_instr_valid[i] = instruction_valid[i] & ~flush_i & tc_window_eligible;
    assign tc_pc[i]          = {{(PC_WIDTH-CVA6Cfg.VLEN){1'b0}}, addr[i]};
    assign tc_instr[i]       = instr[i];
    assign tc_is_branch[i]   = is_branch[i] | is_jump[i] | is_jalr[i] | is_return[i] | is_call[i];

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
      automatic logic raw_taken;
      raw_taken = (taken_rvi_cf[i] | taken_rvc_cf[i]) | is_jump[i] | is_call[i] |
                  (is_jalr[i] & btb_prediction_shifted[i].valid) |
                  (is_return[i] & ras_predict.valid);

      if (found_taken) begin
        tc_taken[i] = 1'b0;
      end else begin
        tc_taken[i] = raw_taken & tc_instr_valid[i];
        if (raw_taken & tc_instr_valid[i]) found_taken = 1'b1;
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

  logic tc_lookup_cond;
  assign tc_lookup_cond = (|instr_queue_consumed) && consumed_has_taken_branch &&
                          !tc_feeding_q && tc_window_eligible;

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

  logic [PC_WIDTH-1:0] tc_lookup_pc_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      tc_lookup_pc_q <= '0;
    else if (tc_lookup_cond && !flush_i)
      tc_lookup_pc_q <= trace_cache_pkg::pc_align_16(tc_pc[0]);
  end

  trace_cache_top #(
      .MaxTraceInstr(CVA6Cfg.INSTR_PER_FETCH)
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
      .flush_i                        (flush_i || is_mispredict || serving_unaligned),
      .instr_queue_ready_i            (instr_queue_ready && tc_window_eligible),
      .instr_queue_consumed_i         (tc_window_eligible ? instr_queue_consumed : '0),
      .branch_predictions_i           (tc_window_eligible ? tc_branch_predictions : '0),
      .lookup_num_branches_i          (tc_window_eligible ? tc_lookup_num_branches : '0),
      .resolved_branch_valid_i        (resolved_branch_i.valid),
      .resolved_branch_pc_i           (resolved_branch_i.pc),
      .resolved_branch_is_taken_i     (resolved_branch_i.is_taken),
      .resolved_branch_is_mispredict_i(resolved_branch_i.is_mispredict),
      .lookup_valid_i                 (tc_lookup_cond),
      .lookup_pc_i                    (trace_cache_pkg::pc_align_16(tc_pc[0])),
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
      .lookup_result_valid_o          (tc_lookup_result_valid),
      .miss_reason_empty_o            (tc_miss_reason_empty),
      .miss_reason_pc_o               (tc_miss_reason_pc),
      .miss_reason_path_o             (tc_miss_reason_path),
      .tc_miss_total_o                (tc_miss_total),
      .tc_miss_empty_o                (tc_miss_empty),
      .tc_miss_pc_o                   (tc_miss_pc),
      .tc_miss_path_o                 (tc_miss_path)
  );

  assign tc_active_hit = tc_lookup_result_valid && tc_trace_hit &&
                         (tc_trace_length != '0) && !flush_i && !is_mispredict;

  // Conservative replay:
  // exactly one taken control-flow in the trace,
  // and that taken control-flow must be the last valid instruction.
  assign tc_active_use = tc_active_hit &&
                         tc_trace_starts_ok &&
                         (tc_trace_num_taken == TAKEN_CNT_WIDTH'(1)) &&
                         tc_trace_last_is_taken_cf &&
                         !tc_feeding_q &&
                         (tc_trace_pcs[0] >= PC_WIDTH'(64'h80001000));

`ifndef SYNTHESIS
  logic [15:0] tc_same_pc_replay_count_q;
  logic [PC_WIDTH-1:0] tc_last_feed_base_pc_q;
`endif

// pragma translate_off
  logic tc_feeding_q_prev;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) tc_feeding_q_prev <= 1'b0;
    else         tc_feeding_q_prev <= tc_feeding_q;
  end

  int unsigned tc_global_hits, tc_global_misses;
  int unsigned tc_fe_miss_empty, tc_fe_miss_pc, tc_fe_miss_path;
  int unsigned tc_feeds_completed, tc_feed_cycles_total;
  int unsigned tc_commit_count_q;
  longint unsigned tc_total_cycles_q;
  logic counting_active, stats_printed;
  int unsigned tc_hits, tc_misses;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) tc_total_cycles_q <= 0;
    else         tc_total_cycles_q <= tc_total_cycles_q + 1;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      counting_active    <= 1'b0;
      stats_printed      <= 1'b0;
      tc_hits            <= 0;
      tc_misses          <= 0;
      tc_global_hits     <= 0;
      tc_global_misses   <= 0;
      tc_fe_miss_empty   <= 0;
      tc_fe_miss_pc      <= 0;
      tc_fe_miss_path    <= 0;
      tc_feeds_completed <= 0;
      tc_feed_cycles_total <= 0;
      tc_commit_count_q  <= 0;
    end else begin
      if (pc_commit_i == 64'h80001568) begin
        counting_active <= 1'b1;
        stats_printed   <= 1'b0;
      end

      if (pc_commit_i == 64'h80001576 && !stats_printed) begin
        counting_active <= 1'b0;
        stats_printed   <= 1'b1;
        $display("\n[BENCH-TC] === Trace Cache (CoreMark only) ===");
        $display("[BENCH-TC] hits=%0d misses=%0d rate=%0d%%",
                 tc_hits, tc_misses,
                 (tc_hits + tc_misses) > 0 ? (tc_hits * 100) / (tc_hits + tc_misses) : 0);
      end

      if (counting_active && tc_lookup_result_valid) begin
        if (tc_trace_hit) tc_hits <= tc_hits + 1;
        else              tc_misses <= tc_misses + 1;
      end

      if (i_trace_cache_top.i_trace_builder.commit_valid_q) begin
        tc_commit_count_q <= tc_commit_count_q + 1;
        if ((tc_commit_count_q + 1) % 500 == 0)
          $display("[TC-STATS] hits=%0d misses=%0d (every 500 commits, #%0d)",
                   tc_hits, tc_misses, tc_commit_count_q + 1);
      end

      if (tc_lookup_result_valid) begin
        if (tc_trace_hit) begin
          tc_global_hits <= tc_global_hits + 1;
        end else begin
          tc_global_misses <= tc_global_misses + 1;
          if (tc_miss_reason_empty) tc_fe_miss_empty <= tc_fe_miss_empty + 1;
          if (tc_miss_reason_pc)    tc_fe_miss_pc    <= tc_fe_miss_pc + 1;
          if (tc_miss_reason_path)  tc_fe_miss_path  <= tc_fe_miss_path + 1;
        end
      end

      if (tc_feeding_q)    tc_feed_cycles_total <= tc_feed_cycles_total + 1;
      if (tc_feeding_done) tc_feeds_completed   <= tc_feeds_completed + 1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_last_feed_base_pc_q   <= '1;
      tc_same_pc_replay_count_q <= 16'd0;
    end else if (tc_feeding_start) begin
      if (tc_trace_pcs[0] != tc_last_feed_base_pc_q)
        tc_same_pc_replay_count_q <= 16'd1;
      else
        tc_same_pc_replay_count_q <= tc_same_pc_replay_count_q + 1'b1;
      tc_last_feed_base_pc_q <= tc_trace_pcs[0];
    end
  end

  always_ff @(posedge clk_i) begin
    if (tc_feeding_done && ((tc_feeds_completed + 1) % 100 == 0))
      $display("[TC-PERIODIC] feeds=%0d hits=%0d misses=%0d hit_rate=%0d%% @ %0t",
               tc_feeds_completed + 1, tc_global_hits, tc_global_misses,
               (tc_global_hits + tc_global_misses) > 0 ?
                 (tc_global_hits * 100) / (tc_global_hits + tc_global_misses) : 0,
               $time);
  end

  final begin
    int tc_total_lookups, tc_hit_pct, tc_fetch_pct;
    tc_total_lookups = tc_global_hits + tc_global_misses;
    tc_hit_pct   = (tc_total_lookups > 0) ? (tc_global_hits * 100) / tc_total_lookups : 0;
    tc_fetch_pct = (tc_total_cycles_q > 0) ? (int'(tc_feed_cycles_total) * 100 / int'(tc_total_cycles_q)) : 0;

    $display("[TC-FINAL] ========== Trace Cache summary ==========");
    $display("[TC-FINAL] lookups=%0d hits=%0d misses=%0d hit_rate=%0d%%",
             tc_total_lookups, tc_global_hits, tc_global_misses, tc_hit_pct);
    $display("[TC-FINAL] feeds=%0d feed_cycles=%0d -> %0d%% of run from trace",
             tc_feeds_completed, tc_feed_cycles_total, tc_fetch_pct);
    $display("[TC-FINAL] miss_breakdown: empty=%0d pc=%0d path=%0d",
             tc_fe_miss_empty, tc_fe_miss_pc, tc_fe_miss_path);
    $display("[TC-FINAL] ========================================");
  end
// pragma translate_on

endmodule