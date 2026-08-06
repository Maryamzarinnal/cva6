`timescale 1ns/1ps

package trace_cache_pkg;

  // Match frontend: one fetch window = up to 4 slots per cycle.
  localparam int unsigned SLOTS_PER_CYCLE = 4;

  // V70: 1-fetch-window traces.  TRACE_LEN = INSTR_PER_FETCH = 4.
  // The entire trace fits in a single IQ push cycle ? no multi-cycle
  // feeding FSM, no phase-2 injection, no post-feed drain wait.
  // One taken branch per trace (MAX_TAKEN = 1).
  localparam int unsigned TRACE_LEN = 4;
  localparam int unsigned MAX_TRACE_INSTR = TRACE_LEN;
  localparam int unsigned INSTR_WIDTH = 32;
  localparam int unsigned TRACE_LEN_WIDTH = $clog2(TRACE_LEN + 1);

  localparam int unsigned NUM_WAYS = 2;
  localparam int unsigned TRACE_ADDRW = 10;  // 2 ways x 1024 sets
  localparam int unsigned GHR_WIDTH = 8;

  localparam int unsigned PC_WIDTH = 64;  // CVA6 is 64-bit
  localparam int unsigned TRIGGER_BRANCH_BITS = SLOTS_PER_CYCLE;
  localparam int unsigned TRIGGER_BRANCH_CNT_WIDTH = $clog2(TRIGGER_BRANCH_BITS + 1);

  // Instructions stored as 16-bit chunks (RVC = 1 chunk, RV32 = 2 chunks).
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;  // 8
  localparam int unsigned BR_CNT_WIDTH = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned SUFFIX_CHUNKS = MAX_TRACE_INSTR * 2;
  localparam int unsigned SUFFIX_LEN_WIDTH = $clog2(MAX_TRACE_INSTR + 1);

  // V90: up to 3 taken branches per trace (multi-window builder).
  localparam int unsigned MAX_TAKEN = 3;
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

  // -----------------------------------------------------------------------
  // TC V68 window-format tag/data representation.
  // Tag: identifies the trigger fetch window (base PC + branch outcomes up to
  //      and including the first taken branch, canonicalized).
  // Data: instructions from base_pc through the taken branch (inclusive),
  //       plus the taken branch target address for NPC redirect.
  // -----------------------------------------------------------------------

  typedef struct packed {
    logic                                       valid;
    logic [PC_WIDTH-1:0]                        base_pc;
    logic [TRIGGER_BRANCH_CNT_WIDTH-1:0]        num_branches;
    logic [TRIGGER_BRANCH_BITS-1:0]             branch_flags;
    logic [GHR_WIDTH-1:0]                       ghr;  // V82: path history for misprediction recovery
  } trace_tag_t;

  typedef struct packed {
    logic                                       valid;
    logic [PC_WIDTH-1:0]                        start_pc;
    logic [PC_WIDTH-1:0]                        exit_pc;
    logic [SUFFIX_LEN_WIDTH-1:0]                len;
    logic [SUFFIX_CHUNKS-1:0][15:0]             chunks;
    logic [SUFFIX_CHUNKS-1:0]                   valid_chunks;
  } trace_payload_t;

  localparam int unsigned TRACE_TAG_WIDTH = $bits(trace_tag_t);
  localparam int unsigned TRACE_PAYLOAD_WIDTH = $bits(trace_payload_t);

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

  function automatic trace_tag_t make_trace_tag(
    input logic [PC_WIDTH-1:0]                 base_pc_i,
    input logic [TRIGGER_BRANCH_CNT_WIDTH-1:0] num_branches_i,
    input logic [TRIGGER_BRANCH_BITS-1:0]      branch_flags_i,
    input logic [GHR_WIDTH-1:0]                ghr_i  // V82: path history
  );
    trace_tag_t tag;
    begin
      tag.valid        = 1'b1;
      tag.base_pc      = base_pc_i;
      tag.num_branches = num_branches_i;
      tag.branch_flags = branch_flags_i;
      tag.ghr          = ghr_i;  // V82
      make_trace_tag   = tag;
    end
  endfunction

  function automatic logic trace_tag_match(
    input trace_tag_t lookup_tag_i,
    input trace_tag_t stored_tag_i
  );
    logic flags_match;
    begin
      flags_match = 1'b1;
      for (int i = 0; i < TRIGGER_BRANCH_BITS; i++) begin
        if (i < int'(lookup_tag_i.num_branches))
          flags_match &= (lookup_tag_i.branch_flags[i] == stored_tag_i.branch_flags[i]);
      end

      trace_tag_match = lookup_tag_i.valid &&
                        stored_tag_i.valid &&
                        (lookup_tag_i.base_pc      == stored_tag_i.base_pc) &&
                        (lookup_tag_i.num_branches == stored_tag_i.num_branches) &&
                        flags_match;  // V98: GHR removed from tag match (kept in struct for capture)
    end
  endfunction

  function automatic logic [PC_WIDTH-1:0] pc_align_16(input logic [PC_WIDTH-1:0] pc);
    pc_align_16 = pc & {{(PC_WIDTH-4){1'b1}}, 4'b0};
  endfunction

  // V71: PC-only set index.  Branch flags removed from the index so the
  // SRAM can be read in parallel with the I$ request (before branch
  // predictions are available).  The 2-way associativity handles the
  // slightly higher set pressure from collapsing branch variants.
  // V77: Way-0 hash (H0) ? XOR-fold of pc[11:4]^pc[19:12]^pc[27:20].
  //
  // V97 (revert V82 in index): GHR is NOT XOR'd into the set index.
  // V98 (revert V82 in tag): GHR is NOT compared in trace_tag_match.
  //   - V82 added GHR to both the set index and the tag to enforce path
  //     sensitivity.  Both choices hurt wikisort hit rate:
  //       index: same PC under different GHR ? different sets ? empty misses
  //              (5,908 ? 62,648 empty misses, C ? C4, 10x churn)
  //       tag:   same PC under different GHR ? same set (after V97) but tag
  //              rejects the entry ? 17,498 path misses, hit rate 11% ? 0.36%
  //   - TC mispredictions have the same recovery cost as branch mispredicts.
  //     The branch_flags field already encodes the local taken/not-taken
  //     pattern per fetch window; GHR adds distant path context that aliases
  //     rarely and eliminates reuse aggressively.
  //   - The `ghr` argument is kept in both function signatures so call sites
  //     do not need to change; it is intentionally unused.
  function automatic logic [TRACE_ADDRW-1:0] tc_index(
    input logic [PC_WIDTH-1:0]  pc,
    input logic [GHR_WIDTH-1:0] ghr
  );
    logic [GHR_WIDTH-1:0] ghr_unused;
    ghr_unused = ghr;  // suppress unused-input lint
    tc_index = pc[TRACE_ADDRW+3:4]
             ^ pc[2*TRACE_ADDRW+3:TRACE_ADDRW+4]
             ^ pc[3*TRACE_ADDRW+3:2*TRACE_ADDRW+4];
  endfunction

  // V77: Way-1 hash (H1) ? shifted 3 bits down: pc[8:1]^pc[16:9]^pc[24:17].
  // V97/V98 (revert V82): see tc_index above for rationale.
  function automatic logic [TRACE_ADDRW-1:0] tc_index_w1(
    input logic [PC_WIDTH-1:0]  pc,
    input logic [GHR_WIDTH-1:0] ghr
  );
    logic [GHR_WIDTH-1:0] ghr_unused;
    ghr_unused = ghr;
    tc_index_w1 = pc[TRACE_ADDRW:1]
                ^ pc[2*TRACE_ADDRW:TRACE_ADDRW+1]
                ^ pc[3*TRACE_ADDRW:2*TRACE_ADDRW+1];
  endfunction

endpackage : trace_cache_pkg
