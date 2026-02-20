`timescale 1ns/1ps
import trace_cache_pkg::*;

// This interface bundles together all the signals that the CVA6 frontend
// sends to the trace_builder. Instead of passing ~20 individual signals
// around, we pass one interface object.
//
// Two modports define who drives what:
//   producer ? the frontend drives instructions IN, reads ready OUT
//   consumer ? the trace_builder reads instructions IN, drives ready OUT
interface tracebuilder_instr_if (
    input logic clk_i,
    input logic rst_ni
);

  // One entry per fetch slot (CVA6 has 4 slots per cycle)
  logic [SLOTS_PER_CYCLE-1:0]               valid;      // is this slot carrying a real instruction?
  logic [SLOTS_PER_CYCLE-1:0][31:0]         inst;       // raw instruction bits (32-bit, even for compressed)
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] pc;         // PC of each instruction
  logic [SLOTS_PER_CYCLE-1:0]               is_branch;  // is this instruction a branch/jump/call/return?
  logic [SLOTS_PER_CYCLE-1:0]               taken;      // was this branch taken?
  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] target;     // branch target address

  // consumed ? driven by instr_queue: one bit per slot, high when that slot
  // is actually consumed by the instruction queue this cycle. The trace_builder
  // uses |consumed as a "new window" indicator to avoid re-processing the same
  // fetch window on stall cycles.
  logic [SLOTS_PER_CYCLE-1:0] consumed;

  // Handshake ? trace_builder drives this to signal it's ready to accept
  logic ready;

  // Frontend drives instructions, reads ready
  modport producer (
    output valid, inst, pc, is_branch, taken, target, consumed,
    input  ready
  );

  // trace_builder reads instructions, drives ready
  modport consumer (
    input  valid, inst, pc, is_branch, taken, target, consumed,
    output ready
  );

endinterface