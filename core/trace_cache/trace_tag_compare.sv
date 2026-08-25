`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_tag_compare (
    input  logic [PC_WIDTH-1:0]                 base_pc_i,
    input  logic [TRIGGER_BRANCH_CNT_WIDTH-1:0] num_branches_i,
    input  logic [TRIGGER_BRANCH_BITS-1:0]      branch_flags_i,
    input  logic [GHR_WIDTH-1:0]                ghr_i,  // path history

    input  trace_tag_t                          stored_tag_i,

    output logic                                hit_o,
    output trace_tag_t                          lookup_tag_o
);

  assign lookup_tag_o = make_trace_tag(base_pc_i, num_branches_i, branch_flags_i, ghr_i);
  assign hit_o        = trace_tag_match(lookup_tag_o, stored_tag_i);

endmodule
