`timescale 1ns/1ps

package trace_cache_pkg;

  // Match frontend: one fetch window = up to 4 slots per cycle
  localparam int unsigned SLOTS_PER_CYCLE = 4;
  localparam int unsigned TRACE_LEN = 4;
  localparam int unsigned INSTR_WIDTH = 32;
  localparam int unsigned TRACE_LEN_WIDTH = $clog2(TRACE_LEN + 1);

  localparam int unsigned NUM_WAYS = 2;
  localparam int unsigned TRACE_ADDRW = 8;  // 2 ways x 256 sets
  localparam int unsigned GHR_WIDTH = 8;

  localparam int unsigned PC_WIDTH = 64;  // CVA6 is 64-bit

  // Instructions stored as 16-bit chunks so we can mix 32-bit and RVC (1 or 2 chunks per instr).
  // valid_chunks[i]=1 means start of an instruction; next chunk may be high half of 32-bit.
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;
  localparam int unsigned BR_CNT_WIDTH = $clog2(CHUNKS_PER_TRACE + 1);

  // New: keep every taken target in trace order.
  // For now, max taken CFs per trace = TRACE_LEN.
  localparam int unsigned MAX_TAKEN = TRACE_LEN;
  localparam int unsigned TAKEN_CNT_WIDTH = $clog2(MAX_TAKEN + 1);

  // PCs are not stored per instruction; on hit we derive them from base_pc + instr + branch_flags.
  // We now also store the ordered list of taken targets.
  localparam int unsigned TRACE_WIDTH =
    1 + PC_WIDTH
    + CHUNKS_PER_TRACE + BR_CNT_WIDTH
    + CHUNKS_PER_TRACE + BR_CNT_WIDTH
    + TAKEN_CNT_WIDTH + (MAX_TAKEN * PC_WIDTH)
    + PC_WIDTH
    + (CHUNKS_PER_TRACE * 16) + CHUNKS_PER_TRACE;

  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;

  typedef struct packed {
    logic                               valid;
    logic [PC_WIDTH-1:0]                base_pc;
    logic [CHUNKS_PER_TRACE-1:0]        lookup_branch_flags;
    logic [BR_CNT_WIDTH-1:0]            lookup_num_branches;
    logic [CHUNKS_PER_TRACE-1:0]        branch_flags;
    logic [BR_CNT_WIDTH-1:0]            num_branches;

    // New: ordered targets of taken control-flow instructions in this trace
    logic [TAKEN_CNT_WIDTH-1:0]         num_taken;
    logic [MAX_TAKEN-1:0][PC_WIDTH-1:0] taken_targets;

    // Final next PC after the trace ends
    logic [PC_WIDTH-1:0]                target_addr;

    logic [CHUNKS_PER_TRACE-1:0][15:0]  chunks;
    logic [CHUNKS_PER_TRACE-1:0]        valid_chunks;
  } trace_data_t;

  localparam int unsigned TC_INDEX_FLAG_BITS = 2;

  function automatic logic [PC_WIDTH-1:0] pc_align_16(input logic [PC_WIDTH-1:0] pc);
    pc_align_16 = pc & {{(PC_WIDTH-4){1'b1}}, 4'b0};
  endfunction

  function automatic logic [TRACE_ADDRW-1:0] tc_index(
    input logic [PC_WIDTH-1:0]         pc,
    input logic [CHUNKS_PER_TRACE-1:0] flags
  );
    tc_index = pc[TRACE_ADDRW+3:4]
             ^ pc[2*TRACE_ADDRW+3:TRACE_ADDRW+4]
             ^ pc[3*TRACE_ADDRW+3:2*TRACE_ADDRW+4]
             ^ TRACE_ADDRW'(flags[TC_INDEX_FLAG_BITS-1:0]);
  endfunction

endpackage : trace_cache_pkg