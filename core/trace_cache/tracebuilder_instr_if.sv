`timescale 1ns/1ps
import trace_cache_pkg::*;

interface tracebuilder_instr_if (
    input logic clk_i,
    input logic rst_ni
);

  logic [SLOTS_PER_CYCLE-1:0]                   valid;
  logic [SLOTS_PER_CYCLE-1:0][INSTR_WIDTH-1:0]  inst;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0]     pc;
  logic [SLOTS_PER_CYCLE-1:0]                   is_branch;
  logic [SLOTS_PER_CYCLE-1:0]                   taken;
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0]     target;

  logic                                          serving_unaligned;
  logic [SLOTS_PER_CYCLE-1:0]                    consumed;
  logic                                          ready;

  modport producer (
    output valid, inst, pc, is_branch, taken, target, consumed, serving_unaligned,
    input  ready
  );

  modport consumer (
    input  valid, inst, pc, is_branch, taken, target, consumed, serving_unaligned,
    output ready
  );

endinterface
