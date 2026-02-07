`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_tag_compare (
    input  logic [PC_WIDTH-1:0]  pc_i,
    input  logic [GHR_WIDTH-1:0] ghr_i,
    
    input  logic                 valid_rd_i,
    input  logic [TAG_W-1:0]     tag_rd_i,
    
    output logic                 hit_o,
    output logic [TAG_W-1:0]     tag_new_o
);

  // ========================================================================
  // Derived Constants
  // ========================================================================
  
  // Tag width: 64-bit PC + GHR width
  localparam int unsigned TAG_W = PC_WIDTH + GHR_WIDTH;  // 72 bits

  // ========================================================================
  // Tag Comparison Logic
  // ========================================================================
  
  // Construct new tag from current PC and GHR
  assign tag_new_o = {pc_i, ghr_i};
  
  // Tag hit when entry is valid AND tag matches
  assign hit_o = valid_rd_i && (tag_new_o == tag_rd_i);

endmodule