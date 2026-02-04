`timescale 1ns/1ps
import trace_cache_pkg::*;

module tag_sram (
    input  logic                     clk_i,
    input  logic                     rst_ni,
    input  logic                     req_i,       // Request signal
    input  logic                     we_i,        // Write enable
    input  logic [TRACE_ADDRW-1:0]   addr_i,      // Address
    input  logic [TAG_W-1:0]         wdata_i,     // Write data 
    input  logic                     valid_i,     // Valid bit to write
    output logic                     valid_o,     // Read valid bit
    output logic [TAG_W-1:0]         rdata_o      // Read tag data
);

  // ========================================================================
  // Derived Constants
  // ========================================================================
  
  // Tag width: Full 64-bit PC + GHR width
  localparam int unsigned TAG_W = PC_WIDTH_FULL + GHR_WIDTH;
  
  localparam int unsigned MEM_W = TAG_W + 1;

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
      addr_q <= addr_i;  // Latch address on request
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
  
  // Read uses the registered address from previous cycle
  assign {valid_o, rdata_o} = mem[addr_q];

endmodule