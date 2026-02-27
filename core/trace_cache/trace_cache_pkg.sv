`timescale 1ns/1ps

package trace_cache_pkg;

  localparam int unsigned SLOTS_PER_CYCLE = 4;

  // Max instructions per trace
  localparam int unsigned TRACE_LEN = 4;

  // SRAM address width -> just for simulation checkin
  localparam int unsigned TRACE_ADDRW = 16;

  // Global history register width
  localparam int unsigned GHR_WIDTH = 8;

  // CVA6 uses 64-bit addresses
  localparam int unsigned PC_WIDTH = 64;

  // Each instruction is stored in 16-bit chunks.
  // 32-bit inst = 2 chunks, compressed = 1 chunk.
  // Worst case (all compressed): TRACE_LEN * 2 chunks.
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;

  // Bits to count up to CHUNKS_PER_TRACE branches
  localparam int unsigned BR_CNT_WIDTH = $clog2(CHUNKS_PER_TRACE + 1);

  localparam int unsigned TRACE_WIDTH =
    1 +                        // valid
    PC_WIDTH +                 // base_pc
    CHUNKS_PER_TRACE +         // branch_flags
    BR_CNT_WIDTH +             // num_branches
    PC_WIDTH +                 // target_addr
    (CHUNKS_PER_TRACE * 16) +  // instruction chunks
    CHUNKS_PER_TRACE;          // valid_chunks

  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;

  // One trace entry stored in SRAM.
  // branch_flags records T/NT outcome for every branch in the trace,
  // in order. Used both as the tag (for lookup matching) and as replay data.
  typedef struct packed {
    logic                              valid;
    logic [PC_WIDTH-1:0]               base_pc;
    logic [CHUNKS_PER_TRACE-1:0]       branch_flags;   // T/NT per branch, across all windows
    logic [BR_CNT_WIDTH-1:0]           num_branches;
    logic [PC_WIDTH-1:0]               target_addr;    // fetch target after trace ends
    logic [CHUNKS_PER_TRACE-1:0][15:0] chunks;
    logic [CHUNKS_PER_TRACE-1:0]       valid_chunks;   // 1 = start of instruction
  } trace_data_t;

  // Direct-mapped SRAM index: XOR-folded PC hash + branch path
  // Folds upper PC bits into the index to break systematic aliasing
  // between code regions that share lower address bits.
  function automatic logic [TRACE_ADDRW-1:0] tc_index(
    input logic [PC_WIDTH-1:0]         pc,
    input logic [CHUNKS_PER_TRACE-1:0] path_id
  );
    logic [TRACE_ADDRW-1:0] path_ext;
    path_ext = TRACE_ADDRW'(path_id);
    tc_index = pc[TRACE_ADDRW+1:2]
             ^ pc[2*TRACE_ADDRW+1:TRACE_ADDRW+2]
             ^ path_ext;
  endfunction

endpackage : trace_cache_pkg
