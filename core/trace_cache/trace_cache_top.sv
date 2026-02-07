`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_cache_top (
  input  logic clk_i,
  input  logic rst_ni,

  // Multi-slot instruction input from frontend
  input  logic [SLOTS_PER_CYCLE-1:0]             instr_valid_i,
  input  logic [SLOTS_PER_CYCLE-1:0][31:0]       instr_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] pc_i,
  input  logic [SLOTS_PER_CYCLE-1:0]             is_branch_i,
  input  logic [SLOTS_PER_CYCLE-1:0]             branch_taken_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] branch_target_i,

  input  logic flush_i,
  input  logic instr_queue_ready_i
);

  // ========================================================================
  // Instruction Interface
  // ========================================================================
  
  tracebuilder_instr_if instr_if (
    .clk_i (clk_i),
    .rst_ni(rst_ni)
  );

  // Connect frontend inputs to interface
  assign instr_if.valid  = instr_valid_i & {SLOTS_PER_CYCLE{instr_queue_ready_i & ~flush_i}};
  assign instr_if.pc     = pc_i;
  assign instr_if.inst   = instr_i;
  assign instr_if.is_branch = is_branch_i;
  assign instr_if.taken  = branch_taken_i;
  assign instr_if.target = branch_target_i;

  // ========================================================================
  // Global History Register
  // ========================================================================
  
  logic [GHR_WIDTH-1:0] ghr;

  tc_ghr i_tc_ghr (
    .clk_i,
    .rst_ni,
    .flush_i,
    .branch_valid_i  (|(instr_valid_i & is_branch_i)),
    .branch_taken_i  (|(instr_valid_i & is_branch_i & branch_taken_i)),
    .ghr_o           (ghr)
  );

  // ========================================================================
  // Trace Builder
  // ========================================================================
  
  logic                   mem_req;
  logic                   mem_we;
  logic [TRACE_ADDRW-1:0] mem_addr;
  logic [TRACE_WIDTH-1:0] mem_wdata;
  logic [BE_WIDTH-1:0]    mem_be;

  logic                   tag_valid;
  logic [PC_WIDTH-1:0]    tag_start_pc;
  logic [GHR_WIDTH-1:0]   tag_start_ghr;
  logic [3:0]             tag_trace_len;
  logic [TRACE_ADDRW-1:0] tag_sram_addr;

  trace_builder i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i         (instr_if),
    .ghr_i           (ghr),
    .flush_i         (flush_i),

    .trace_valid_o   (),
    .trace_data_o    (),

    .mem_req_o       (mem_req),
    .mem_we_o        (mem_we),
    .mem_addr_o      (mem_addr),
    .mem_wdata_o     (mem_wdata),
    .mem_be_o        (mem_be),

    .tag_valid_o     (tag_valid),
    .tag_start_pc_o  (tag_start_pc),
    .tag_start_ghr_o (tag_start_ghr),
    .tag_trace_len_o (tag_trace_len),
    .tag_sram_addr_o (tag_sram_addr)
  );

  // ========================================================================
  // Trace SRAM (stores complete traces)
  // ========================================================================
  
  tc_sram #(
    .NumWords  (1 << TRACE_ADDRW),  // 1024 entries
    .DataWidth (TRACE_WIDTH),       // 331 bits per trace
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

  // ========================================================================
  // Tag SRAM (stores trace metadata for lookup)
  // ========================================================================
  
  tag_sram #(
    .TAG_W (PC_WIDTH + GHR_WIDTH),  // 64-bit PC + 8-bit GHR = 72 bits
    .ADDRW (TRACE_ADDRW)
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

endmodule