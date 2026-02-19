`timescale 1ns/1ps

package trace_cache_pkg;

  // ---------------------------------------------------------
  localparam int unsigned SLOTS_PER_CYCLE = 4;

  // ---------------------------------------------------------
  // how many instructions we store per trace
  // Set to 4 to match CVA6's fetch window width. The whole point
  // of the trace cache is to stitch together non-consecutive
  // instructions across taken branches
  // ---------------------------------------------------------
  localparam int unsigned TRACE_LEN = 4;

  // ---------------------------------------------------------
  // We use bits of base_pc to index directly into this SRAM
  // (direct-mapped cache style).
  // ---------------------------------------------------------
  localparam int unsigned TRACE_ADDRW = 6;

  // ---------------------------------------------------------
  // global history register for branch prediction.
  // ---------------------------------------------------------
  localparam int unsigned GHR_WIDTH = 8;

  // ---------------------------------------------------------
  // 64-bit addresses to match CVA6's XLEN=64.
  // ---------------------------------------------------------
  localparam int unsigned PC_WIDTH = 64;

  // ---------------------------------------------------------
  // ach instruction is stored as 16-bit chunks.
  // A normal (32-bit) instruction takes 2 chunks.
  // A compressed (16-bit, RVC) instruction takes 1 chunk.
  // Worst case: all instructions are compressed ? 2*TRACE_LEN chunks total.
  // ---------------------------------------------------------
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;

  // ---------------------------------------------------------
  // total bits needed to store one trace entry.
  //   1 bit         : valid flag
  //   PC_WIDTH      : base PC (start of trace)
  //   CHUNKS_PER_TRACE : branch_flags (one bit per possible instruction,
  //                   worst case all are branches ? compressed worst case)
  //   CHUNKS_PER_TRACE bits : num_branches (encoded, enough to count up to CHUNKS_PER_TRACE)
  //                   actually stored as $clog2(CHUNKS_PER_TRACE+1) bits ? see struct
  //   PC_WIDTH      : target address (where to fetch after trace ends)
  //   CHUNKS_PER_TRACE*16  : instruction chunks
  //   CHUNKS_PER_TRACE     : valid_chunks flags
  // ---------------------------------------------------------
  localparam int unsigned BR_CNT_WIDTH = $clog2(CHUNKS_PER_TRACE + 1); // 4 bits for 0..8

  localparam int unsigned TRACE_WIDTH =
    1 +                        // valid
    PC_WIDTH +                 // base_pc
    CHUNKS_PER_TRACE +         // branch_flags (one per possible instruction slot)
    BR_CNT_WIDTH +             // num_branches
    PC_WIDTH +                 // target_addr
    (CHUNKS_PER_TRACE * 16) +  // instruction chunks (16 bits each)
    CHUNKS_PER_TRACE;          // valid_chunks (one flag per chunk)

  // Byte enable width for SRAM writes
  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;

  // ---------------------------------------------------------
  // the actual trace entry stored in SRAM.
  //
  // TAG fields (used for lookup matching):
  //   valid        : is this entry occupied?
  //   base_pc      : PC of the first instruction in the trace
  //   branch_flags : taken/not-taken outcome for each branch in the trace,
  //                  in order. branch_flags[0] = first branch, etc.
  //   num_branches : how many branches are actually in this trace,
  //                  so we only compare the relevant bits of branch_flags
  //
  // DATA fields (used after a hit to reconstruct the fetch stream):
  //   target_addr  : where to fetch after this trace ends
  //                  (target of the last taken branch)
  //   valid_chunks : which chunks are the start of an instruction
  //                  (0 = second half of a 32-bit inst, 1 = start of new inst)
  // ---------------------------------------------------------
  typedef struct packed {
    logic                            valid;
    logic [PC_WIDTH-1:0]             base_pc;
    logic [CHUNKS_PER_TRACE-1:0]     branch_flags;
    logic [BR_CNT_WIDTH-1:0]         num_branches;
    logic [PC_WIDTH-1:0]             target_addr;
    logic [CHUNKS_PER_TRACE-1:0][15:0] chunks;
    logic [CHUNKS_PER_TRACE-1:0]     valid_chunks;
  } trace_data_t;

endpackage : trace_cache_pkg