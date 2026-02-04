`timescale 1ns/1ps

package trace_cache_pkg;

  // ========================================================================
  // Configuration Parameters
  // ========================================================================
  
  // Fetch configuration
  localparam int unsigned SLOTS_PER_CYCLE = 4;   // Max instructions per cycle (RVC + superscalar)
  
  // Trace dimensions
  localparam int unsigned TRACE_LEN       = 4;   // Instructions per trace
  localparam int unsigned MAX_BRANCHES    = 2;   // Max taken branches per trace
  
  // Memory configuration
  localparam int unsigned TRACE_ADDRW     = 10;  // Trace SRAM address width (2^10 = 1024 entries)
  
  // Branch correlation
  localparam int unsigned GHR_WIDTH       = 8;   // Global History Register width
  
  // Address widths
  // CVA6 uses 64-bit addresses (XLEN=64), but we store only lower 32-bit PCs in traces.
  // Full 64-bit PC is kept in tag_sram for accurate matching.
  // TODO (Riccardo): Discuss if 32-bit PC storage is acceptable for target workloads.
  localparam int unsigned PC_WIDTH_FULL   = 64;  // Full PC width (matches CVA6 XLEN)
  localparam int unsigned PC_WIDTH_TRACE  = 32;  // PC width stored in traces (lower 32 bits)
  
  // Trace data width calculations
  // Each entry: 32-bit PC + 32-bit inst + 1-bit is_branch + 1-bit taken = 66 bits
  // Total: 4 entries × 66 bits = 264 bits
  localparam int unsigned ENTRY_WIDTH     = PC_WIDTH_TRACE + 32 + 2;    // 66 bits per entry
  localparam int unsigned TRACE_WIDTH     = TRACE_LEN * ENTRY_WIDTH;    // 264 bits total
  
  // Byte enable width for SRAM: ceiling(TRACE_WIDTH / 8)
  // Formula: (TRACE_WIDTH + 7) / 8 ensures proper rounding up
  localparam int unsigned BE_WIDTH        = (TRACE_WIDTH + 7) / 8;      // 33 bytes

  // ========================================================================
  // Type Definitions
  // ========================================================================
  
  // One instruction entry inside a trace
  typedef struct packed {
    logic [PC_WIDTH_TRACE-1:0] pc;        // 32-bit PC (lower bits only)
    logic [31:0]               inst;      // 32-bit instruction (RISC-V standard)
    logic                      is_branch; 
    logic                      taken;     
  } trace_entry_t;  // Total: 66 bits per entry

endpackage : trace_cache_pkg