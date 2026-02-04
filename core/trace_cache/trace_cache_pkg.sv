`timescale 1ns/1ps

package trace_cache_pkg;

  // ========================================================================
  // Configuration Parameters
  // ========================================================================
  
  // Fetch configuration
  localparam int unsigned SLOTS_PER_CYCLE = 4;   // Max instructions per cycle 
  
  // Trace dimensions
  localparam int unsigned TRACE_LEN       = 4;   // Instructions per trace
  localparam int unsigned MAX_BRANCHES    = 2;   // Max taken branches per trace
  
  // Memory configuration
  localparam int unsigned TRACE_ADDRW     = 10;  // Trace SRAM address width
  
  // Branch correlation
  localparam int unsigned GHR_WIDTH       = 8;   // Global History Register width
  
  // Address width 
  localparam int unsigned PC_WIDTH        = 64;  

  // ========================================================================
  // Type Definitions
  // ========================================================================
  
  typedef struct packed {
    logic [PC_WIDTH-1:0] pc;        // 64-bit program counter
    logic [31:0]         inst;      // 32-bit instruction 
    logic                is_branch;
    logic                taken;
  } trace_entry_t;

endpackage : trace_cache_pkg