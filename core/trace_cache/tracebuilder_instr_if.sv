`timescale 1ns/1ps

interface tracebuilder_instr_if (
  input  logic clk_i,
  input  logic rst_ni
);

  logic        valid;
  logic        ready;
  logic [31:0] pc;
  logic [31:0] inst;
  logic        is_branch;
  logic        taken;
  logic [31:0] next_pc;

  modport producer (
    output valid, pc, inst, is_branch, taken, next_pc,
    input  ready
  );

  modport consumer (
    input  valid, pc, inst, is_branch, taken, next_pc,
    output ready
  );

endinterface
