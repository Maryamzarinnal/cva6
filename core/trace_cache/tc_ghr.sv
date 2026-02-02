`timescale 1ns/1ps

module tc_ghr #(
    parameter int unsigned GHR_W = 16
)(
    input  logic             clk_i,
    input  logic             rst_ni,

    input  logic             flush_i,

    input  logic             branch_valid_i,  
    input  logic             branch_taken_i,  

    output logic [GHR_W-1:0] ghr_o
);

  logic [GHR_W-1:0] ghr_q, ghr_d;

  // Next-state logic
  always_comb begin
    if (flush_i) begin
      ghr_d = '0;
    end else if (branch_valid_i) begin
      ghr_d = {ghr_q[GHR_W-2:0], branch_taken_i};
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
