// Copyright 2018 - 2019 ETH Zurich and University of Bologna.
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
// Date: 26.10.2018
//
// Description: Instruction Queue with per-slot predict-address FIFOs.
//   Modified: predict_address_i is per-slot [INSTR_PER_FETCH][VLEN].
//   Predict addresses track the same physical instruction FIFOs instead of
//   collapsing through one shared address path. Branch flags remain a separate
//   single FIFO snapshot because the current frontend still launches at most
//   one taken control-flow per packet.

module instr_queue
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned BRANCH_FLAGS_W = 16,
    parameter type fetch_entry_t = logic
) (
    input logic clk_i,
    input logic rst_ni,
    input logic flush_i,
    input logic [CVA6Cfg.INSTR_PER_FETCH-1:0][31:0] instr_i,
    input logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] addr_i,
    input logic [CVA6Cfg.INSTR_PER_FETCH-1:0] valid_i,
    output logic ready_o,
    output logic empty_o,
    output logic [CVA6Cfg.INSTR_PER_FETCH-1:0] consumed_o,
    input ariane_pkg::frontend_exception_t exception_i,
    input logic [CVA6Cfg.VLEN-1:0] exception_addr_i,
    input logic [CVA6Cfg.GPLEN-1:0] exception_gpaddr_i,
    input logic [31:0] exception_tinst_i,
    input logic exception_gva_i,
    // Per-slot predict address (normal: all same; trace: per-CF target)
    input logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] predict_address_i,
    input logic [BRANCH_FLAGS_W-1:0] branch_flags_i,
    input ariane_pkg::cf_t [CVA6Cfg.INSTR_PER_FETCH-1:0] cf_type_i,
    // Trace cache: bypass branch_mask so multiple CFs pass through
    input logic tc_feeding_i,
    input logic reseed_pc_i,
    output logic replay_o,
    output logic [CVA6Cfg.VLEN-1:0] replay_addr_o,
    output logic [BRANCH_FLAGS_W-1:0] branch_flags_o,
    output fetch_entry_t [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_o,
    output logic [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_valid_o,
    input logic [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_ready_i
);

  localparam NID = CVA6Cfg.SuperscalarEn ? 1 : 0;
  localparam int unsigned ADDR_FIFO_FLAGS_W = BRANCH_FLAGS_W;
  localparam int unsigned ADDR_FIFO_W = CVA6Cfg.VLEN + ADDR_FIFO_FLAGS_W;

  typedef struct packed {
    logic [31:0]                     instr;
    ariane_pkg::cf_t                 cf;
    ariane_pkg::frontend_exception_t ex;
    logic [CVA6Cfg.VLEN-1:0]         ex_vaddr;
    logic [CVA6Cfg.GPLEN-1:0]        ex_gpaddr;
    logic [31:0]                     ex_tinst;
    logic                            ex_gva;
  } instr_data_t;

  logic [CVA6Cfg.LOG2_INSTR_PER_FETCH-1:0] branch_index;
  instr_data_t [CVA6Cfg.INSTR_PER_FETCH-1:0] instr_data_in, instr_data_out;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] push_instr, push_instr_fifo;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] push_instr_eff;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] pop_instr;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] instr_queue_full;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] instr_queue_empty;
  logic                               instr_overflow;
  // Per-slot predict-address FIFOs aligned with the instruction FIFOs.
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0][CVA6Cfg.VLEN-1:0] addr_data_out;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] empty_addr;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] push_addr, pop_addr;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] full_addr;
  logic                               push_address;
  logic                               full_address;
  logic                               address_overflow;
  logic                               pop_branch_flags_single;
  logic                               full_branch_flags_single;
  logic                               empty_branch_flags_single;
  logic [BRANCH_FLAGS_W-1:0]          branch_flags_out_single;
  logic                               tc_replay_pkt_can_push;

  logic [CVA6Cfg.LOG2_INSTR_PER_FETCH-1:0] idx_is_d, idx_is_q;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] idx_ds_d, idx_ds_q;
  logic [CVA6Cfg.NrIssuePorts:0][CVA6Cfg.INSTR_PER_FETCH-1:0] idx_ds;

  logic [CVA6Cfg.VLEN-1:0] pc_d, pc_q;
  logic [CVA6Cfg.NrIssuePorts:0][CVA6Cfg.VLEN-1:0] pc_j;
  logic reset_address_d, reset_address_q;

  logic [CVA6Cfg.NrIssuePorts-1:0] fetch_entry_is_cf, fetch_entry_fire;

  logic [CVA6Cfg.INSTR_PER_FETCH*2-2:0] branch_mask_extended;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] branch_mask;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] taken;
  logic [CVA6Cfg.LOG2_INSTR_PER_FETCH:0] popcount;
  logic [CVA6Cfg.LOG2_INSTR_PER_FETCH-1:0] shamt;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] valid;
  logic [CVA6Cfg.INSTR_PER_FETCH*2-1:0] consumed_extended;
  logic [CVA6Cfg.INSTR_PER_FETCH*2-1:0] fifo_pos_extended;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] fifo_pos;
  logic [CVA6Cfg.INSTR_PER_FETCH*2-1:0][31:0] instr;
  ariane_pkg::cf_t [CVA6Cfg.INSTR_PER_FETCH*2-1:0] cf;
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] instr_overflow_fifo;

  // Duplicate & rotate predict addresses (same rotation as instructions)
  logic [CVA6Cfg.INSTR_PER_FETCH*2-1:0][CVA6Cfg.VLEN-1:0] pred_addr_dup;

  assign empty_o = &instr_queue_empty;

  assign full_address = (|full_addr) | full_branch_flags_single;
  assign ready_o = ~(|instr_queue_full) & ~full_address;
  // During trace-cache replay, only enqueue when the full packet can be accepted.
  // This prevents partial packet insertion that can reorder replay semantics.
  assign tc_replay_pkt_can_push = !tc_feeding_i || ready_o;
  assign push_instr_eff         = tc_replay_pkt_can_push ? push_instr : '0;

  if (CVA6Cfg.RVC) begin : gen_multiple_instr_per_fetch_with_C

    for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_unpack_taken
      assign taken[i] = cf_type_i[i] != ariane_pkg::NoCF;
    end

    lzc #(
        .WIDTH(CVA6Cfg.INSTR_PER_FETCH),
        .MODE (0)
    ) i_lzc_branch_index (
        .in_i   (taken),
        .cnt_o  (branch_index),
        .empty_o()
    );

    assign branch_mask_extended = {{{CVA6Cfg.INSTR_PER_FETCH-1}{1'b0}}, {{CVA6Cfg.INSTR_PER_FETCH}{1'b1}}} << branch_index;
    assign branch_mask = branch_mask_extended[CVA6Cfg.INSTR_PER_FETCH * 2 - 2:CVA6Cfg.INSTR_PER_FETCH - 1];

    // tc_feeding_i: bypass branch_mask so ALL valid trace instructions pass through
    assign valid = tc_feeding_i ? valid_i : (valid_i & branch_mask);

    assign consumed_extended = {push_instr_fifo, push_instr_fifo} >> idx_is_q;
    assign consumed_o = consumed_extended[CVA6Cfg.INSTR_PER_FETCH-1:0];

    popcount #(
        .INPUT_WIDTH(CVA6Cfg.INSTR_PER_FETCH)
    ) i_popcount (
        .data_i    (push_instr_fifo),
        .popcount_o(popcount)
    );
    assign shamt = popcount[$bits(shamt)-1:0];
    assign idx_is_d = idx_is_q + shamt;

    assign fifo_pos_extended = {valid, valid} << idx_is_q;
    assign fifo_pos = fifo_pos_extended[CVA6Cfg.INSTR_PER_FETCH*2-1:CVA6Cfg.INSTR_PER_FETCH];
    assign push_instr = fifo_pos & ~instr_queue_full;

    for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_duplicate_instr_input
      assign instr[i] = instr_i[i];
      assign instr[i+CVA6Cfg.INSTR_PER_FETCH] = instr_i[i];
      assign cf[i] = cf_type_i[i];
      assign cf[i+CVA6Cfg.INSTR_PER_FETCH] = cf_type_i[i];
      // Duplicate predict addresses for rotation
      assign pred_addr_dup[i] = predict_address_i[i];
      assign pred_addr_dup[i+CVA6Cfg.INSTR_PER_FETCH] = predict_address_i[i];
    end

    for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_fifo_input_select
      /* verilator lint_off WIDTH */
      assign instr_data_in[i].instr = instr[CVA6Cfg.INSTR_PER_FETCH+i-idx_is_q];
      assign instr_data_in[i].cf = cf[CVA6Cfg.INSTR_PER_FETCH+i-idx_is_q];
      assign instr_data_in[i].ex = exception_i;
      assign instr_data_in[i].ex_vaddr = exception_addr_i;
      if (CVA6Cfg.RVH) begin : gen_hyp_ex_with_C
        assign instr_data_in[i].ex_gpaddr = exception_gpaddr_i;
        assign instr_data_in[i].ex_tinst = exception_tinst_i;
        assign instr_data_in[i].ex_gva = exception_gva_i;
      end else begin : gen_no_hyp_ex_with_C
        assign instr_data_in[i].ex_gpaddr = '0;
        assign instr_data_in[i].ex_tinst = '0;
        assign instr_data_in[i].ex_gva = 1'b0;
      end
      /* verilator lint_on WIDTH */
    end
  end else begin : gen_multiple_instr_per_fetch_without_C
    assign taken = '0;
    assign branch_index = '0;
    assign branch_mask_extended = '0;
    assign branch_mask = '0;
    assign consumed_extended = '0;
    assign fifo_pos_extended = '0;
    assign fifo_pos = '0;
    assign instr = '0;
    assign popcount = '0;
    assign shamt = '0;
    assign valid = '0;
    assign pred_addr_dup = '0;
    assign consumed_o = push_instr_fifo[0];
    assign push_instr = valid_i & ~instr_queue_full;
    /* verilator lint_off WIDTH */
    assign instr_data_in[0].instr = instr_i[0];
    assign instr_data_in[0].cf = cf_type_i[0];
    assign instr_data_in[0].ex = exception_i;
    assign instr_data_in[0].ex_vaddr = exception_addr_i;
    if (CVA6Cfg.RVH) begin : gen_hyp_ex_without_C
      assign instr_data_in[0].ex_gpaddr = exception_gpaddr_i;
      assign instr_data_in[0].ex_tinst = exception_tinst_i;
      assign instr_data_in[0].ex_gva = exception_gva_i;
    end else begin : gen_no_hyp_ex_without_C
      assign instr_data_in[0].ex_gpaddr = '0;
      assign instr_data_in[0].ex_tinst = '0;
      assign instr_data_in[0].ex_gva = 1'b0;
    end
    /* verilator lint_on WIDTH */
  end

  // ----------------------
  // Replay Logic
  // ----------------------
  if (CVA6Cfg.RVC == 1'b1) begin : gen_instr_overflow_fifo_with_C
    assign instr_overflow_fifo = instr_queue_full & fifo_pos;
  end else begin : gen_instr_overflow_fifo_without_C
    assign instr_overflow_fifo = instr_queue_full & valid_i;
  end
  assign instr_overflow = |instr_overflow_fifo;

  // Per-slot address push attempt (before overflow check)
  logic [CVA6Cfg.INSTR_PER_FETCH-1:0] push_addr_attempt;
  always_comb begin
    push_address = 1'b0;
    for (int i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin
      push_addr_attempt[i] = push_instr_eff[i] & (instr_data_in[i].cf != ariane_pkg::NoCF);
      push_address |= push_addr_attempt[i];
    end
  end
  assign address_overflow = (|(full_addr & push_addr_attempt)) |
                            (full_branch_flags_single & push_address);
  // Replay backpressure is handled by tc_feeding consumed bookkeeping.
  // Keep legacy replay request behavior for non-TC traffic only.
  assign replay_o = tc_feeding_i ? 1'b0 : (instr_overflow | address_overflow);

  if (CVA6Cfg.RVC) begin : gen_replay_addr_o_with_c
    assign replay_addr_o = (address_overflow) ? addr_i[0] : addr_i[shamt];
  end else begin : gen_replay_addr_o_without_C
    assign replay_addr_o = addr_i[0];
  end

  // ----------------------
  // Downstream interface
  // ----------------------
  // After flush/reset, block dequeue for one cycle until pc_q is re-seeded
  // from addr_i[0]. This prevents stale low PC values (e.g. 0x0/0x2/0x4)
  // from being observed when a new packet arrives.
  assign fetch_entry_valid_o[0] = ~(&instr_queue_empty) & ~reset_address_q;
  if (CVA6Cfg.SuperscalarEn) begin : gen_fetch_entry_valid_1
    assign fetch_entry_valid_o[NID] = ~|(instr_queue_empty & idx_ds[1]) & ~(&fetch_entry_is_cf) &
                                      ~reset_address_q;
  end

  assign idx_ds[0] = idx_ds_q;
  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    if (CVA6Cfg.INSTR_PER_FETCH > 1) begin
      assign idx_ds[i+1] = {
        idx_ds[i][CVA6Cfg.INSTR_PER_FETCH-2:0], idx_ds[i][CVA6Cfg.INSTR_PER_FETCH-1]
      };
    end else begin
      assign idx_ds[i+1] = idx_ds[i];
    end
  end

  if (CVA6Cfg.RVC) begin : gen_downstream_itf_with_c
    always_comb begin
      idx_ds_d  = idx_ds_q;
      pop_instr = '0;
      pop_addr  = '0;
      for (int unsigned i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
        fetch_entry_o[i].instruction = '0;
        fetch_entry_o[i].address = pc_j[i];
        fetch_entry_o[i].ex.valid = 1'b0;
        fetch_entry_o[i].ex.cause = '0;
        fetch_entry_o[i].ex.tval = '0;
        fetch_entry_o[i].ex.tval2 = '0;
        fetch_entry_o[i].ex.gva = 1'b0;
        fetch_entry_o[i].ex.tinst = '0;
        fetch_entry_o[i].branch_predict.predict_address = '0;
        fetch_entry_o[i].branch_predict.cf = ariane_pkg::NoCF;
      end
      for (int unsigned i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin
        if (idx_ds[0][i]) begin
          if (instr_data_out[i].ex == ariane_pkg::FE_INSTR_ACCESS_FAULT) begin
            fetch_entry_o[0].ex.cause = riscv::INSTR_ACCESS_FAULT;
          end else if (CVA6Cfg.RVH && instr_data_out[i].ex == ariane_pkg::FE_INSTR_GUEST_PAGE_FAULT) begin
            fetch_entry_o[0].ex.cause = riscv::INSTR_GUEST_PAGE_FAULT;
          end else begin
            fetch_entry_o[0].ex.cause = riscv::INSTR_PAGE_FAULT;
          end
          fetch_entry_o[0].instruction = instr_data_out[i].instr;
          fetch_entry_o[0].ex.valid = instr_data_out[i].ex != ariane_pkg::FE_NONE;
          if (CVA6Cfg.TvalEn)
            fetch_entry_o[0].ex.tval = {
              {(CVA6Cfg.XLEN - CVA6Cfg.VLEN) {1'b0}}, instr_data_out[i].ex_vaddr
            };
          if (CVA6Cfg.RVH) begin
            fetch_entry_o[0].ex.tval2 = instr_data_out[i].ex_gpaddr;
            fetch_entry_o[0].ex.tinst = instr_data_out[i].ex_tinst;
            fetch_entry_o[0].ex.gva   = instr_data_out[i].ex_gva;
          end
          fetch_entry_o[0].branch_predict.cf = instr_data_out[i].cf;
          fetch_entry_o[0].branch_predict.predict_address =
              (instr_data_out[i].cf != ariane_pkg::NoCF) ? addr_data_out[i] : '0;
          pop_instr[i] = fetch_entry_fire[0];
          pop_addr[i]  = fetch_entry_fire[0] & (instr_data_out[i].cf != ariane_pkg::NoCF);
        end
        if (CVA6Cfg.SuperscalarEn) begin
          if (idx_ds[1][i]) begin
            if (instr_data_out[i].ex == ariane_pkg::FE_INSTR_ACCESS_FAULT) begin
              fetch_entry_o[NID].ex.cause = riscv::INSTR_ACCESS_FAULT;
            end else begin
              fetch_entry_o[NID].ex.cause = riscv::INSTR_PAGE_FAULT;
            end
            fetch_entry_o[NID].instruction = instr_data_out[i].instr;
            fetch_entry_o[NID].ex.valid = instr_data_out[i].ex != ariane_pkg::FE_NONE;
            fetch_entry_o[NID].ex.tval = {{64 - CVA6Cfg.VLEN{1'b0}}, instr_data_out[i].ex_vaddr};
            fetch_entry_o[NID].branch_predict.cf = instr_data_out[i].cf;
            fetch_entry_o[NID].branch_predict.predict_address =
                (instr_data_out[i].cf != ariane_pkg::NoCF) ? addr_data_out[i] : '0;
            pop_instr[i] = fetch_entry_fire[NID];
            pop_addr[i]  = fetch_entry_fire[NID] & (instr_data_out[i].cf != ariane_pkg::NoCF);
          end
        end
      end
      if (fetch_entry_fire[0]) begin
        if (CVA6Cfg.SuperscalarEn) begin
          idx_ds_d = fetch_entry_fire[NID] ? idx_ds[2] : idx_ds[1];
        end else begin
          idx_ds_d = idx_ds[1];
        end
      end
    end
  end else begin : gen_downstream_itf_without_c
    always_comb begin
      idx_ds_d = '0;
      idx_is_d = '0;
      pop_addr = '0;
      fetch_entry_o[0].instruction = instr_data_out[0].instr;
      fetch_entry_o[0].address = pc_q;
      fetch_entry_o[0].ex.valid = instr_data_out[0].ex != ariane_pkg::FE_NONE;
      if (instr_data_out[0].ex == ariane_pkg::FE_INSTR_ACCESS_FAULT) begin
        fetch_entry_o[0].ex.cause = riscv::INSTR_ACCESS_FAULT;
      end else begin
        fetch_entry_o[0].ex.cause = riscv::INSTR_PAGE_FAULT;
      end
      if (CVA6Cfg.TvalEn)
        fetch_entry_o[0].ex.tval = {{64 - CVA6Cfg.VLEN{1'b0}}, instr_data_out[0].ex_vaddr};
      else fetch_entry_o[0].ex.tval = '0;
      if (CVA6Cfg.RVH) begin
        fetch_entry_o[0].ex.tval2 = instr_data_out[0].ex_gpaddr;
        fetch_entry_o[0].ex.tinst = instr_data_out[0].ex_tinst;
        fetch_entry_o[0].ex.gva   = instr_data_out[0].ex_gva;
      end else begin
        fetch_entry_o[0].ex.tval2 = '0;
        fetch_entry_o[0].ex.tinst = '0;
        fetch_entry_o[0].ex.gva   = 1'b0;
      end
      fetch_entry_o[0].branch_predict.predict_address =
          (instr_data_out[0].cf != ariane_pkg::NoCF) ? addr_data_out[0] : '0;
      fetch_entry_o[0].branch_predict.cf = instr_data_out[0].cf;
      pop_instr[0] = fetch_entry_valid_o[0] & fetch_entry_ready_i[0];
      pop_addr[0]  = pop_instr[0] & (instr_data_out[0].cf != ariane_pkg::NoCF);
    end
  end

  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    assign fetch_entry_is_cf[i] = fetch_entry_o[i].branch_predict.cf != ariane_pkg::NoCF;
    assign fetch_entry_fire[i]  = fetch_entry_valid_o[i] & fetch_entry_ready_i[i];
  end

  // Keep one branch-flags snapshot FIFO: current frontend policy still admits
  // at most one taken control-flow that can redirect per dequeued packet.
  assign pop_branch_flags_single = |(fetch_entry_is_cf & fetch_entry_fire);

  // ----------------------
  // Calculate (Next) PC
  // ----------------------
  assign pc_j[0] = pc_q;
  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    assign pc_j[i+1] = fetch_entry_is_cf[i] ? fetch_entry_o[i].branch_predict.predict_address : (
      pc_j[i] + ((fetch_entry_o[i].instruction[1:0] != 2'b11) ? 'd2 : 'd4)
    );
  end

  always_comb begin
    pc_d = pc_q;
    reset_address_d = flush_i ? 1'b1 : reset_address_q;
    if (fetch_entry_fire[0]) begin
      pc_d = pc_j[1];
      if (CVA6Cfg.SuperscalarEn) begin
        if (fetch_entry_fire[NID]) pc_d = pc_j[2];
      end
    end
    if (valid_i[0] && (reset_address_q || reseed_pc_i)) begin
      pc_d = addr_i[0];
      reset_address_d = 1'b0;
    end
  end

  // ----------------------
  // Instruction FIFOs (unchanged)
  // ----------------------
  for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_instr_fifo
    assign push_instr_fifo[i] = push_instr_eff[i] & ~address_overflow;
    cva6_fifo_v3 #(
        .FPGA_ALTERA(CVA6Cfg.FpgaAlteraEn),
        .DEPTH(ariane_pkg::FETCH_FIFO_DEPTH),
        .dtype(instr_data_t),
        .FPGA_EN(CVA6Cfg.FpgaEn)
    ) i_fifo_instr_data (
        .clk_i, .rst_ni, .flush_i,
        .testmode_i(1'b0),
        .full_o (instr_queue_full[i]),
        .empty_o(instr_queue_empty[i]),
        .usage_o(),
        .data_i (instr_data_in[i]),
        .push_i (push_instr_fifo[i]),
        .data_o (instr_data_out[i]),
        .pop_i  (pop_instr[i])
    );
  end

  // ----------------------
  // Per-slot Address FIFOs (4 FIFOs, one per instruction slot)
  // ----------------------
  for (genvar i = 0; i < CVA6Cfg.INSTR_PER_FETCH; i++) begin : gen_addr_fifo
    assign push_addr[i] = push_addr_attempt[i] & ~address_overflow;

    cva6_fifo_v3 #(
        .FPGA_ALTERA(CVA6Cfg.FpgaAlteraEn),
        .DEPTH      (ariane_pkg::FETCH_ADDR_FIFO_DEPTH),
        .DATA_WIDTH (CVA6Cfg.VLEN),
        .FPGA_EN    (CVA6Cfg.FpgaEn)
    ) i_fifo_address (
        .clk_i, .rst_ni, .flush_i,
        .testmode_i(1'b0),
        .full_o (full_addr[i]),
        .empty_o(empty_addr[i]),
        .usage_o(),
        /* verilator lint_off WIDTH */
        .data_i (CVA6Cfg.RVC ? pred_addr_dup[CVA6Cfg.INSTR_PER_FETCH+i-idx_is_q] : predict_address_i[0]),
        /* verilator lint_on WIDTH */
        .push_i (push_addr[i]),
        .data_o (addr_data_out[i]),
        .pop_i  (pop_addr[i])
    );
  end

  // Keep a single FIFO only for branch-flags snapshots.
  cva6_fifo_v3 #(
      .FPGA_ALTERA(CVA6Cfg.FpgaAlteraEn),
      .DEPTH      (ariane_pkg::FETCH_ADDR_FIFO_DEPTH),
      .DATA_WIDTH (BRANCH_FLAGS_W),
      .FPGA_EN    (CVA6Cfg.FpgaEn)
  ) i_fifo_branch_flags_single (
      .clk_i, .rst_ni, .flush_i,
      .testmode_i(1'b0),
      .full_o (full_branch_flags_single),
      .empty_o(empty_branch_flags_single),
      .usage_o(),
      .data_i (branch_flags_i),
      .push_i (push_address & ~full_branch_flags_single),
      .data_o (branch_flags_out_single),
      .pop_i  (pop_branch_flags_single & ~empty_branch_flags_single)
  );

  assign branch_flags_o = empty_branch_flags_single ? '0 : branch_flags_out_single;

`ifndef SYNTHESIS
  function automatic logic iq_has_push_hole(input logic [CVA6Cfg.INSTR_PER_FETCH-1:0] bits_i);
    logic seen_one;
    logic seen_zero_after_one;
    begin
      iq_has_push_hole = 1'b0;
      seen_one = 1'b0;
      seen_zero_after_one = 1'b0;
      for (int h = 0; h < CVA6Cfg.INSTR_PER_FETCH; h++) begin
        if (bits_i[h]) begin
          seen_one = 1'b1;
          if (seen_zero_after_one)
            iq_has_push_hole = 1'b1;
        end else if (seen_one) begin
          seen_zero_after_one = 1'b1;
        end
      end
    end
  endfunction

  logic [7:0] iq_addr_underflow_guard_q;
  logic [7:0] iq_replay_req_debug_q;
  logic [7:0] iq_push_hole_debug_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni || flush_i) begin
      iq_addr_underflow_guard_q <= '0;
      iq_replay_req_debug_q <= '0;
      iq_push_hole_debug_q <= '0;
    end else if (pop_branch_flags_single && empty_branch_flags_single &&
                 (iq_addr_underflow_guard_q < 8'h20)) begin
      iq_addr_underflow_guard_q <= iq_addr_underflow_guard_q + 1'b1;
      $display("[IQ-BRANCHFLAGS-UNDERFLOW-GUARD] t=%0t pop_single while empty pc_q=0x%h idx_ds_q=%b tc_feeding=%0b",
               $time, pc_q, idx_ds_q, tc_feeding_i);
    end else begin
      if ((instr_overflow || address_overflow) && (iq_replay_req_debug_q < 8'h40)) begin
        iq_replay_req_debug_q <= iq_replay_req_debug_q + 1'b1;
        $display("[IQ-REPLAY-REQ] t=%0t idx_is_q=%0d valid=%b fifo_pos=%b full=%b push=%b push_fifo=%b ovf=%b addr_ovf=%0b shamt=%0d replay_addr=0x%h tc_feeding=%0b",
                 $time, idx_is_q, valid, fifo_pos, instr_queue_full, push_instr,
                 push_instr_fifo, instr_overflow_fifo, address_overflow, shamt,
                 replay_addr_o, tc_feeding_i);
      end

      if (iq_has_push_hole(push_instr_fifo) && (iq_push_hole_debug_q < 8'h40)) begin
        iq_push_hole_debug_q <= iq_push_hole_debug_q + 1'b1;
        $display("[IQ-PUSH-HOLE] t=%0t idx_is_q=%0d valid=%b fifo_pos=%b full=%b push=%b push_fifo=%b ovf=%b shamt=%0d replay_addr=0x%h",
                 $time, idx_is_q, valid, fifo_pos, instr_queue_full, push_instr,
                 push_instr_fifo, instr_overflow_fifo, shamt, replay_addr_o);
      end
    end
  end
`endif

  unread i_unread_branch_mask (.d_i(|branch_mask_extended));
  unread i_unread_fifo_pos (.d_i(|fifo_pos_extended));

  if (CVA6Cfg.RVC) begin : gen_pc_q_with_c
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        idx_ds_q        <= 'b1;
        idx_is_q        <= '0;
        pc_q            <= '0;
        reset_address_q <= 1'b1;
      end else begin
        pc_q            <= pc_d;
        reset_address_q <= reset_address_d;
        if (flush_i) begin
          idx_ds_q        <= 'b1;
          idx_is_q        <= '0;
          reset_address_q <= 1'b1;
        end else begin
          idx_ds_q <= idx_ds_d;
          idx_is_q <= idx_is_d;
        end
      end
    end
  end else begin : gen_pc_q_without_C
    assign idx_ds_q = '0;
    assign idx_is_q = '0;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        pc_q            <= '0;
        reset_address_q <= 1'b1;
      end else begin
        pc_q            <= pc_d;
        reset_address_q <= reset_address_d;
        if (flush_i) reset_address_q <= 1'b1;
      end
    end
  end

  // pragma translate_off
  output_select_onehot :
  assert property (@(posedge clk_i) $onehot0(idx_ds_q))
  else begin
    $error("Output select should be one-hot encoded");
    $stop();
  end
  // pragma translate_on
endmodule
