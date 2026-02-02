`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_cache_top #(
  parameter int unsigned WINDOW_SIZE  = 4,
  parameter int unsigned TRACE_LEN    = 4,
  parameter int unsigned MAX_BRANCHES = 2,
  parameter int unsigned ADDRW        = 10,
  parameter int unsigned GHR_W        = 8
)(
  input  logic clk_i,
  input  logic rst_ni,

  // --------------
  // from frontend
  // --------------

  input  logic        instr_valid_i,
  input  logic [31:0] instr_i,
  input  logic [31:0] pc_i,
  input  logic        is_branch_i,
  input  logic        branch_taken_i,
  input  logic [31:0] next_pc_i,

  input  logic        flush_i,
  input  logic        instr_queue_ready_i
);

  // --------------
  //interface toward trace_builder
  // --------------
  tracebuilder_instr_if instr_if (
    .clk_i (clk_i),
    .rst_ni(rst_ni)
  );

  // --------------
  // We are the producer
  // --------------

  always_comb begin
    instr_if.valid     = instr_valid_i & instr_queue_ready_i & ~flush_i ;
    instr_if.pc        = pc_i;
    instr_if.inst      = instr_i;
    instr_if.is_branch = is_branch_i;
    instr_if.taken     = branch_taken_i;
    instr_if.next_pc   = next_pc_i;
  end

  // --------------
  //    Local GHR
  // --------------

  logic [GHR_W-1:0] ghr;

  tc_ghr #(
    .GHR_W(GHR_W)
  ) i_tc_ghr (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .flush_i         (flush_i),
    .branch_valid_i  (instr_valid_i & is_branch_i),
    .branch_taken_i  (branch_taken_i),
    .ghr_o           (ghr)
  );

  //-----------------------------
  //        Trace builder
  //-----------------------------
  logic               mem_req;
  logic               mem_we;
  logic [ADDRW-1:0]   mem_addr;
  logic [255:0]       mem_wdata;
  logic [31:0]        mem_be;

  logic               tag_valid;
  logic [31:0]        tag_start_pc;
  logic [GHR_W-1:0]   tag_start_ghr;
  logic [3:0]         tag_trace_len;
  logic [ADDRW-1:0]   tag_sram_addr;

  trace_builder #(
    .WINDOW_SIZE  (WINDOW_SIZE),
    .TRACE_LEN    (TRACE_LEN),
    .MAX_BRANCHES (MAX_BRANCHES),
    .ADDRW        (ADDRW),
    .GHR_W        (GHR_W)
  ) i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i        (instr_if),
    .ghr_i          (ghr),

    .trace_valid_o  (),
    .trace_data_o   (),

    .mem_req_o      (mem_req),
    .mem_we_o       (mem_we),
    .mem_addr_o     (mem_addr),
    .mem_wdata_o    (mem_wdata),
    .mem_be_o       (mem_be),

    .tag_valid_o     (tag_valid),
    .tag_start_pc_o  (tag_start_pc),
    .tag_start_ghr_o (tag_start_ghr),
    .tag_trace_len_o (tag_trace_len),
    .tag_sram_addr_o (tag_sram_addr)
  );

  // -----------------------------
  //      Trace SRAM 
  // -----------------------------
  tc_sram #(
    .NumWords  (1 << ADDRW),
    .DataWidth (256),
    .NumPorts  (1)
  ) i_trace_sram (
    .clk_i,
    .rst_ni,
    .req_i   ({mem_req}),
    .we_i    ({mem_we}),
    .addr_i  ({mem_addr}),
    .wdata_i ({mem_wdata}),
    .be_i    ({mem_be}),
    .rdata_o ()
  );

  // --------------
  //       Tag SRAM
  // --------------
  tag_sram #(
    .TAG_W (32 + GHR_W),
    .ADDRW(ADDRW)
  ) i_tag_sram (
    .clk_i,
    .rst_ni,
    .req_i    (tag_valid),
    .we_i     (tag_valid),
    .addr_i   (tag_sram_addr),
    .wdata_i  ({tag_start_pc, tag_start_ghr}),
    .valid_i  (1'b1),
    .valid_o  (),
    .rdata_o  ()
  );
  always_ff @(posedge clk_i) begin
    if (instr_if.valid) begin
      $display("[TC-IN] pc=%h inst=%h br=%0b taken=%0b",
             instr_if.pc,
             instr_if.inst,
             instr_if.is_branch,
             instr_if.taken);
    end
  end
endmodule
