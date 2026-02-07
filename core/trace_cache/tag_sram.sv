`timescale 1ns/1ps
import trace_cache_pkg::*;

module tag_sram (
    input  logic                 clk_i,
    input  logic                 rst_ni,
    input  logic                 req_i,
    input  logic                 we_i,
    input  logic [TRACE_ADDRW-1:0] addr_i,
    input  logic [TAG_W-1:0]     wdata_i,
    input  logic                 valid_i,
    output logic                 valid_o,
    output logic [TAG_W-1:0]     rdata_o
);

  // ========================================================================
  // Derived Constants
  // ========================================================================
  
  // Tag width: 64-bit PC + GHR width
  localparam int unsigned TAG_W = PC_WIDTH + GHR_WIDTH;  // 72 bits
  
  // Memory width: tag + valid bit
  localparam int unsigned MEM_W = TAG_W + 1;  // 73 bits

  // ========================================================================
  // SRAM Storage
  // ========================================================================
  
  logic [MEM_W-1:0] mem [0:(1<<TRACE_ADDRW)-1];
  logic [TRACE_ADDRW-1:0] addr_q;

  // ========================================================================
  // Address Register
  // ========================================================================
  
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      addr_q <= '0;
    else if (req_i)
      addr_q <= addr_i;
  end

  // ========================================================================
  // Write Logic
  // ========================================================================
  
  always_ff @(posedge clk_i) begin
    if (req_i && we_i)
      mem[addr_i] <= {valid_i, wdata_i};
  end

  // ========================================================================
  // Read Logic
  // ========================================================================
  
  assign {valid_o, rdata_o} = mem[addr_q];

endmodule