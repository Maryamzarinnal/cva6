`timescale 1ns/1ps
import trace_cache_pkg::*;

module tc_ghr (
    input  logic                 clk_i,
    input  logic                 rst_ni,
    input  logic                 flush_i,
    input  logic                 branch_valid_i,  
    input  logic                 branch_taken_i,  
    output logic [GHR_WIDTH-1:0] ghr_o            
);


  // Most recent branch is in LSB, oldest in MSB
  logic [GHR_WIDTH-1:0] ghr_q, ghr_d;

  always_comb begin
    if (flush_i) begin
      ghr_d = '0;
    end else if (branch_valid_i) begin
      // Shift left and insert new branch outcome in LSB
      ghr_d = {ghr_q[GHR_WIDTH-2:0], branch_taken_i};
    end else begin
      // No update, hold current value
      ghr_d = ghr_q;
    end
  end
  
  // Sequential logic: register the GHR
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      ghr_q <= '0;
    else
      ghr_q <= ghr_d;
  end

  // Output assignment
  assign ghr_o = ghr_q;

endmodule