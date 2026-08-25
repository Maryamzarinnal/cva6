`timescale 1ns/1ps
import trace_cache_pkg::*;

module tag_sram (
    input  logic                            clk_i,
    input  logic                            rst_ni,
    input  logic                            req_i,
    input  logic                            we_i,
    input  logic [TRACE_ADDRW-1:0]          addr_i,
    input  trace_tag_t                      wdata_i,
    output trace_tag_t                      rdata_o
);

  trace_tag_t mem [0:(1 << TRACE_ADDRW)-1];
  logic [TRACE_ADDRW-1:0] addr_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      addr_q <= '0;
    else if (req_i)
      addr_q <= addr_i;
  end

  // No reset of the array itself.  Resetting every entry would infer a
  // flip-flop per bit and stop this mapping onto an SRAM macro.  Cold-start
  // safety comes from the valid_ff bits in trace_cache_top, which do reset to
  // zero and gate the hit, so an uninitialised tag can never produce one.
  //
  // Plain `always` rather than `always_ff` on purpose: the single-driver rule
  // is specific to always_ff, and the initial block below is a second driver
  // of mem.  A clocked always infers the same storage.
  always @(posedge clk_i) begin
    if (req_i && we_i)
      mem[addr_i] <= wdata_i;
  end

`ifndef SYNTHESIS
  // Simulation only.  Real SRAM powers up with arbitrary contents, which is
  // harmless because valid_ff gates the hit -- but four-state X is not the
  // same as arbitrary bits: it propagates through the dedup tag compare and
  // poisons the write-control logic that sets valid_ff, leaving the cache
  // permanently empty.  Zeroing at time 0 reproduces silicon behaviour
  // without inferring any reset logic.
  initial begin
    for (int i = 0; i < (1 << TRACE_ADDRW); i++)
      mem[i] = '0;
  end
`endif

  assign rdata_o = mem[addr_q];

endmodule
