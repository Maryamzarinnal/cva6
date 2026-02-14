`timescale 1ns/1ps

package trace_cache_pkg;

  // ========================================================================
  // Configuration Parameters 
  // ========================================================================
  
  // Fetch bandwidth: instructions per cycle from instruction cache
  localparam int unsigned SLOTS_PER_CYCLE = 4;
  
  // Maximum instructions per trace
  localparam int unsigned TRACE_LEN = 16;  
  
  // Maximum taken branches per trace
  localparam int unsigned MAX_BRANCHES = 3;  
  
  // Trace cache configuration
  localparam int unsigned TRACE_ADDRW = 6;   
  localparam int unsigned GHR_WIDTH   = 8;   
  
  // Address configuration
  localparam int unsigned PC_WIDTH = 64;
  
  // Store instruction chunks as 16-bit pieces to handle compressed instructions
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;  
  
  // ========================================================================
  // Trace Data Width Calculation 
  // ========================================================================
  //   - tag (starting PC)
  //   - branch_flags: (m-1) bits, where m = num_branches
  //   - branch_mask: log2(B+1) bits for num_branches + 1 bit for ends_in_branch
  //   - fall_through_addr: next PC if last branch not taken
  //   - target_addr: next PC if last branch taken  
  //   - instructions: N instructions
  
  localparam int unsigned TRACE_WIDTH = 
    1 +                                  // valid bit
    PC_WIDTH +                           // base PC (tag)
    (MAX_BRANCHES - 1) +                 // branch_flags: (B-1) = 2 bits
    2 +                                  // num_branches: log2(B+1) = 2 bits
    PC_WIDTH +                           // fall_through_addr
    PC_WIDTH +                           // target_addr
    (CHUNKS_PER_TRACE * 16) +            // instruction chunks 
    CHUNKS_PER_TRACE;                    // valid_chunks bits 
  
  // Byte enable width for SRAM writes
  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;

  // ========================================================================
  // Type Definitions 
  // ========================================================================
  
  typedef struct packed {
    // ---- Control Information (for lookup and next fetch) ----
    
    // "valid bit: indicates this is a valid trace"
    logic valid;
    
    // "tag: identifies the starting address of the trace"
    logic [PC_WIDTH-1:0] base_pc;
    
    // "branch flags: there is a single bit for each branch within the trace
    // to indicate the path followed after the branch (taken/not taken).
    // The m-th branch of the trace does not need a flag since no instructions
    // follow it, hence there are only (m-1) bits instead of m bits."
    logic [MAX_BRANCHES-2:0] branch_flags;  
    
    // "branch mask: state is needed to indicate (1) the number of branches
    // in the trace and (2) whether or not the trace ends in a branch."
    logic [1:0] num_branches;  
    
    // "trace fall-through address: next fetch address if the last branch
    // in the trace is predicted not taken"
    logic [PC_WIDTH-1:0] fall_through_addr;
    
    // "trace target address: next fetch address if the last branch in the
    // trace is predicted taken"
    logic [PC_WIDTH-1:0] target_addr;
    
    // ---- Instruction Storage ----
    logic [CHUNKS_PER_TRACE-1:0][15:0] chunks;  // Instruction data
    logic [CHUNKS_PER_TRACE-1:0] valid_chunks;  // Which chunks are instruction starts
  } trace_data_t;

endpackage : trace_cache_pkg