`timescale 1ns/1ps
import trace_cache_pkg::*;

interface tracebuilder_instr_if (
    input logic clk_i,
    input logic rst_ni
);

  // One entry per fetch slot 
  logic [SLOTS_PER_CYCLE-1:0]               valid;      
  logic [SLOTS_PER_CYCLE-1:0][31:0]         inst;       // raw instruction bits 
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] pc;         // PC of each instruction
  logic [SLOTS_PER_CYCLE-1:0]               is_branch;  // is this instruction a branch/jump/call/return?
  logic [SLOTS_PER_CYCLE-1:0]               taken;      
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] target;     

  // Handshake 
  logic ready;

  modport producer (
    output valid, inst, pc, is_branch, taken, target,
    input  ready
  );

  modport consumer (
    input  valid, inst, pc, is_branch, taken, target,
    output ready
  );

endinterface