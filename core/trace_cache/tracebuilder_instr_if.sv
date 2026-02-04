`timescale 1ns/1ps
import trace_cache_pkg::*;

interface tracebuilder_instr_if #(
  parameter int SLOTS = 4
)(
  input logic clk_i,
  input logic rst_ni
);

  logic [SLOTS-1:0]              valid;
  logic                          ready;
  logic [SLOTS-1:0][PC_WIDTH-1:0] pc;        
  logic [SLOTS-1:0][31:0]        inst;       
  logic [SLOTS-1:0]              is_branch;
  logic [SLOTS-1:0]              taken;
  logic [SLOTS-1:0][PC_WIDTH-1:0] next_pc;   

  modport producer (
    output valid, pc, inst, is_branch, taken, next_pc,
    input  ready
  );

  modport consumer (
    input  valid, pc, inst, is_branch, taken, next_pc,
    output ready
  );

endinterface