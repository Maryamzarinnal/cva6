`timescale 1ns/1ps

module trace_tag_compare #(
    parameter int unsigned PC_W  = 32,
    parameter int unsigned GHR_W = 16
)(
    input  logic [PC_W-1:0]          pc_i,
    input  logic [GHR_W-1:0]         ghr_i,

    input  logic                     valid_rd_i,
    input  logic [PC_W+GHR_W-1:0]    tag_rd_i,

    output logic                     hit_o,
    output logic [PC_W+GHR_W-1:0]    tag_new_o
);

  assign tag_new_o = {pc_i, ghr_i};
  assign hit_o     = valid_rd_i && (tag_new_o == tag_rd_i);

endmodule
