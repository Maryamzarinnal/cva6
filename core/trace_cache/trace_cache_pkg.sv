`timescale 1ns/1ps

package trace_cache_pkg;

  // ========================================================================
  // Configuration Parameters
  // ========================================================================
  
  localparam int unsigned SLOTS_PER_CYCLE = 4;   // Instructions per fetch cycle
  localparam int unsigned TRACE_LEN       = 4;   // Max instructions per trace
  localparam int unsigned MAX_BRANCHES    = 2;   // Max taken branches per trace
  localparam int unsigned TRACE_ADDRW     = 10;  // Trace SRAM address bits 
  localparam int unsigned GHR_WIDTH       = 8;   // Global history register width
  
  // Address configuration
  localparam int unsigned PC_WIDTH        = 64;
  
  // Store instruction chunks as 16-bit pieces to handle compressed instructions
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;  // 8 chunks (each instruction = 2 chunks max)
  
  // Trace data width calculation
  localparam int unsigned TRACE_WIDTH = 
    PC_WIDTH +                           // base PC
    (MAX_BRANCHES - 1) +                 // branch flags (m-1 bits, last branch doesn't need flag)
    2 +                                  //  num_branches (count 0-2)
    PC_WIDTH +                           // fall-through address
    PC_WIDTH +                           //  target address  
    (CHUNKS_PER_TRACE * 16) +            //  instruction chunks (8 × 16 bits)
    CHUNKS_PER_TRACE;                    //  valid bits (instruction start markers)
    
  
  // Byte enable width for SRAM writes
  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;  // 42 bytes

  // ========================================================================
  // Type Definitions
  // ========================================================================
  
  // Complete trace storage format
  typedef struct packed {
    logic [PC_WIDTH-1:0]           base_pc;          // Starting PC of trace
    logic [MAX_BRANCHES-2:0]       branch_flags;     
    logic [1:0]                    num_branches;     // How many branches in trace (0-2)
    logic [PC_WIDTH-1:0]           fall_through_addr; // Next PC if last branch not taken
    logic [PC_WIDTH-1:0]           target_addr;      // Next PC if last branch taken
    logic [CHUNKS_PER_TRACE-1:0][15:0] chunks;       // Instruction data 
    logic [CHUNKS_PER_TRACE-1:0]   valid;            // Which chunks are instruction starts
  } trace_data_t;  

endpackage : trace_cache_pkg