`timescale 1ns/1ps

module tag_sram #(
    parameter int unsigned TAG_W = 48,   // PC(32) + GHR(16)
    parameter int unsigned ADDRW = 10    // depth = 2^ADDRW
)(
    input  logic               clk_i,
    input  logic               rst_ni,

    input  logic               req_i,
    input  logic               we_i,
    input  logic [ADDRW-1:0]   addr_i,
    input  logic [TAG_W-1:0]   wdata_i,
    input  logic               valid_i,      

    output logic               valid_o,      // read valid bit
    output logic [TAG_W-1:0]   rdata_o       // read tag
);

  // Total memory width = tag + valid = 49 bits
  localparam int unsigned MEM_W = TAG_W + 1;

  // SRAM
  logic [MEM_W-1:0] mem [0:(1<<ADDRW)-1];

  // Registered read address
  logic [ADDRW-1:0] addr_q;

  // Register address for sync-read
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      addr_q <= '0;
    else if (req_i)
      addr_q <= addr_i;
  end

  // Write: synchronous
  always_ff @(posedge clk_i) begin
    if (req_i && we_i)
      mem[addr_i] <= {valid_i, wdata_i}; // store: valid + tag
  end

  // Read: synchronous via addr_q
  assign {valid_o, rdata_o} = mem[addr_q];

endmodule
