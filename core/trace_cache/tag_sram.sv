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

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < (1 << TRACE_ADDRW); i++)
        mem[i] <= '0;
    end else if (req_i && we_i) begin
      mem[addr_i] <= wdata_i;
    end
  end

  assign rdata_o = mem[addr_q];

endmodule
