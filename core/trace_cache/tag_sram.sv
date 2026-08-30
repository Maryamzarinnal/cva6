`timescale 1ns/1ps
import trace_cache_pkg::*;

// ////////////////////////////////////////////////////////////////////////
// SYNTHESIS BLACKBOX -- STEP 1 of 4
// Uncomment the parameter list below (and comment the plain "module tag_sram ("
// line) to give the module the geometry parameters the blackbox needs.
// Defaults match the current instantiation, so trace_cache_top needs no change.
//
// module tag_sram #(
//   parameter int unsigned NumWords  = (1 << TRACE_ADDRW),   // 256
//   parameter int unsigned DataWidth = $bits(trace_tag_t),   // 72
//   parameter int unsigned ByteWidth = 32'd8,
//   parameter int unsigned NumPorts  = 32'd1,
//   parameter int unsigned Latency   = 32'd1,
//   parameter              ImplKey   = "none"
// ) (
// ////////////////////////////////////////////////////////////////////////

module tag_sram (
    input  logic                            clk_i,
    input  logic                            rst_ni,
    input  logic                            req_i,
    input  logic                            we_i,
    input  logic [TRACE_ADDRW-1:0]          addr_i,
    input  trace_tag_t                      wdata_i,
    output trace_tag_t                      rdata_o
);

// ////////////////////////////////////////////////////////////////////////
// SYNTHESIS BLACKBOX -- STEP 2 of 4
// Uncomment these five lines to wrap everything below in the simulation-only
// branch.  The tool then sees the blackbox from STEP 3 instead of this array.
//
// `ifdef SYNTHESIS
//   `define TAG_SRAM_SYNTHESIS
// `elsif TARGET_SYNTHESIS
//   `define TAG_SRAM_SYNTHESIS
// `endif
//
// `ifndef TAG_SRAM_SYNTHESIS
// ////////////////////////////////////////////////////////////////////////

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

// ////////////////////////////////////////////////////////////////////////
// SYNTHESIS BLACKBOX -- STEP 3 of 4
// Uncomment this block.  It closes the simulation branch and gives the tool
// an empty cell instead, with the array size attached so the flow can pick
// the matching memory macro.
//
// `else
//   (* sram_num_words = NumWords, sram_data_width = DataWidth,
//      sram_byte_width = ByteWidth, sram_num_ports = NumPorts,
//      sram_latency = Latency, sram_impl_key = ImplKey *)
//   tag_sram_blackbox #(
//     .NumWords  ( NumWords  ),
//     .DataWidth ( DataWidth ),
//     .ByteWidth ( ByteWidth ),
//     .NumPorts  ( NumPorts  ),
//     .Latency   ( Latency   ),
//     .ImplKey   ( ImplKey   )
//   ) i_tag_sram_blackbox (
//     .clk_i   ( clk_i   ),
//     .rst_ni  ( rst_ni  ),
//     .req_i   ( req_i   ),
//     .we_i    ( we_i    ),
//     .addr_i  ( addr_i  ),
//     .wdata_i ( wdata_i ),
//     .rdata_o ( rdata_o )
//   );
// `endif
// ////////////////////////////////////////////////////////////////////////

endmodule

// ////////////////////////////////////////////////////////////////////////
// SYNTHESIS BLACKBOX -- STEP 4 of 4
// Uncomment this empty module.  It has the same ports as tag_sram but no
// contents: that is what tells the tool not to build the memory out of
// flip-flops and to expect a macro instead.
//
// (* black_box, syn_black_box = 1 *)
// module tag_sram_blackbox #(
//   parameter int unsigned NumWords  = 32'd256,
//   parameter int unsigned DataWidth = 32'd72,
//   parameter int unsigned ByteWidth = 32'd8,
//   parameter int unsigned NumPorts  = 32'd1,
//   parameter int unsigned Latency   = 32'd1,
//   parameter              ImplKey   = "none"
// ) (
//   input  logic                   clk_i,
//   input  logic                   rst_ni,
//   input  logic                   req_i,
//   input  logic                   we_i,
//   input  logic [TRACE_ADDRW-1:0] addr_i,
//   input  trace_tag_t             wdata_i,
//   output trace_tag_t             rdata_o
// );
// endmodule
// ////////////////////////////////////////////////////////////////////////
