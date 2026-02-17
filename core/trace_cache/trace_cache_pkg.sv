`timescale 1ns/1ps
package trace_cache_pkg;

  localparam int unsigned SLOTS_PER_CYCLE = 4;

  localparam int unsigned TRACE_LEN = 4;
  localparam int unsigned MAX_INSTRUCTIONS = TRACE_LEN;

  localparam int unsigned MAX_BRANCHES = 3;

  localparam int unsigned TRACE_ADDRW = 6;
  localparam int unsigned GHR_WIDTH   = 8;

  localparam int unsigned PC_WIDTH = 64;

  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;

  localparam int unsigned TRACE_WIDTH =
    1 +
    PC_WIDTH +
    (MAX_BRANCHES - 1) +
    2 +
    PC_WIDTH +
    (CHUNKS_PER_TRACE * 16) +
    CHUNKS_PER_TRACE;

  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;

  typedef struct packed {
    logic valid;

    logic [PC_WIDTH-1:0] base_pc;

    logic [MAX_BRANCHES-2:0] branch_flags;

    logic [1:0] num_branches;

    logic [PC_WIDTH-1:0] target_addr;

    logic [CHUNKS_PER_TRACE-1:0][15:0] chunks;
    logic [CHUNKS_PER_TRACE-1:0] valid_chunks;
  } trace_data_t;

endpackage : trace_cache_pkg