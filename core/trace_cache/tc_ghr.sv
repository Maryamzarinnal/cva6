`timescale 1ns/1ps
import trace_cache_pkg::*;

// tcGlobal History Register for the trace cache.

module tc_ghr (
    input  logic                 clk_i,
    input  logic                 flush_i,        // reset history to all-zeros
    input  logic                 rst_ni,
    input  logic                 branch_valid_i, // a branch instruction is present this cycle
    input  logic                 branch_taken_i, // that branch was taken
    output logic [GHR_WIDTH-1:0] ghr_o           // current history (MSB=oldest, LSB=newest)
);

  logic [GHR_WIDTH-1:0] ghr_q, ghr_d;

  always_comb begin
    if (flush_i) begin
      ghr_d = '0;
    end else if (branch_valid_i) begin
      ghr_d = {ghr_q[GHR_WIDTH-2:0], branch_taken_i};
    end else begin
      ghr_d = ghr_q;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      ghr_q <= '0;
    else
      ghr_q <= ghr_d;
  end

  assign ghr_o = ghr_q;

endmodule