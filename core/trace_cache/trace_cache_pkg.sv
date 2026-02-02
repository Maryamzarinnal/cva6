package trace_cache_pkg;

  // One instruction entry inside a trace
  typedef struct packed {
    logic [31:0] pc;
    logic [31:0] inst;
    logic        is_branch;
    logic        taken;
  } trace_entry_t;

endpackage

