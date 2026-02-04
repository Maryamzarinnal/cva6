`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_tag_compare (
    input  logic [PC_WIDTH-1:0]              pc_i,      
    input  logic [GHR_WIDTH-1:0]             ghr_i,
    
    input  logic                             valid_rd_i,
    input  logic [PC_WIDTH+GHR_WIDTH-1:0]    tag_rd_i,  
    
    output logic                             hit_o,
    output logic [PC_WIDTH+GHR_WIDTH-1:0]    tag_new_o  
);

  assign tag_new_o = {pc_i, ghr_i};
  assign hit_o     = valid_rd_i && (tag_new_o == tag_rd_i);

endmodule