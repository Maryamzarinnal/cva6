`timescale 1ns/1ps

package trace_cache_pkg;

  // Match frontend: one fetch window = up to 4 slots per cycle.
  localparam int unsigned SLOTS_PER_CYCLE = 4;

  // Trace length equals the fetch window, so a whole trace goes into the IQ
  // in one push -- no multi-cycle feed FSM and no drain wait afterwards.
  localparam int unsigned TRACE_LEN = 4;
  localparam int unsigned MAX_TRACE_INSTR = TRACE_LEN;
  localparam int unsigned INSTR_WIDTH = 32;
  localparam int unsigned TRACE_LEN_WIDTH = $clog2(TRACE_LEN + 1);

  localparam int unsigned NUM_WAYS = 2;
  localparam int unsigned TRACE_ADDRW = 8;
  localparam int unsigned GHR_WIDTH = 8;

  localparam int unsigned PC_WIDTH = 64;  // CVA6 is 64-bit
  localparam int unsigned TRIGGER_BRANCH_BITS = SLOTS_PER_CYCLE;
  localparam int unsigned TRIGGER_BRANCH_CNT_WIDTH = $clog2(TRIGGER_BRANCH_BITS + 1);

  // Upper bound on branches recorded per trace, and the width of the branch
  // bookkeeping vectors.  Two per instruction is the worst case the builder
  // has to describe, so this stays at TRACE_LEN*2 even though instructions
  // themselves are now stored one per slot.
  localparam int unsigned CHUNKS_PER_TRACE = TRACE_LEN * 2;  // 8
  localparam int unsigned BR_CNT_WIDTH = $clog2(CHUNKS_PER_TRACE + 1);

  // Taken branches a trace may cross.  A third target costs 64 bits in every
  // entry and served 12 hits out of 768592 on CoreMark, so it is not worth
  // provisioning; the multi-window builder still spans two taken branches.
  localparam int unsigned MAX_TAKEN = 2;
  localparam int unsigned TAKEN_CNT_WIDTH = $clog2(MAX_TAKEN + 1);

  // PCs are not stored per instruction; on hit we derive them from base_pc + instr + branch_flags.
  // We now also store the ordered list of taken targets.
  localparam int unsigned TRACE_WIDTH =
    1 + PC_WIDTH
    + CHUNKS_PER_TRACE + BR_CNT_WIDTH
    + CHUNKS_PER_TRACE + BR_CNT_WIDTH
    + TAKEN_CNT_WIDTH + (MAX_TAKEN * PC_WIDTH)
    + PC_WIDTH
    + (TRACE_LEN * INSTR_WIDTH) + TRACE_LEN;

  localparam int unsigned BE_WIDTH = (TRACE_WIDTH + 7) / 8;

  // -----------------------------------------------------------------------
  // Window-format tag / data representation.
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
    // No ghr field: history is neither indexed nor compared (see tc_index),
    // so storing it in every tag cost 8 bits per entry for nothing.  The
    // misprediction restore in tc_ghr works off the pipeline snapshot, not
    // off the tag, and is unaffected.
  } trace_tag_t;

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

    // One slot per instruction, already aligned.  A compressed instruction is
    // stored zero-extended, so replay needs no unpacking step.  Packing these
    // as 16-bit chunks saved nothing, because the builder caps a trace at
    // TRACE_LEN instructions regardless of how few chunks they occupy, and it
    // forced a variable-stride unpack loop that cannot be synthesised.
    logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0] instrs;
    logic [TRACE_LEN-1:0]                  instr_valid;
  } trace_data_t;

  function automatic trace_tag_t make_trace_tag(
    input logic [PC_WIDTH-1:0]                 base_pc_i,
    input logic [TRIGGER_BRANCH_CNT_WIDTH-1:0] num_branches_i,
    input logic [TRIGGER_BRANCH_BITS-1:0]      branch_flags_i,
    input logic [GHR_WIDTH-1:0]                ghr_i  // unused, kept for call sites
  );
    trace_tag_t tag;
    logic [GHR_WIDTH-1:0] ghr_unused;
    begin
      ghr_unused       = ghr_i;
      tag.valid        = 1'b1;
      tag.base_pc      = base_pc_i;
      tag.num_branches = num_branches_i;
      tag.branch_flags = branch_flags_i;
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
                        flags_match;  // GHR removed from tag match (kept in struct for capture)
    end
  endfunction

  function automatic logic [PC_WIDTH-1:0] pc_align_16(input logic [PC_WIDTH-1:0] pc);
    pc_align_16 = pc & {{(PC_WIDTH-4){1'b1}}, 4'b0};
  endfunction

  // Set index is PC-only so the SRAM read can start alongside the I$ request,
  // before any branch prediction exists.  Two skewed ways absorb the extra
  // set pressure from folding the branch variants of a PC together.
  //
  // The GHR is deliberately not part of the index or the tag.  Adding it
  // sent the same PC under different histories to different sets --
  // empty misses went 5.9k -> 62.6k on wikisort -- and once that was fixed,
  // tag rejection alone still dropped the hit rate from 11% to 0.36%.
  // branch_flags already carries the local taken pattern, which is the part
  // that matters; distant history just destroys reuse.  The ghr argument
  // stays in the signature so call sites are untouched.
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

  // Way-1 hash (H1) ? shifted 3 bits down: pc[8:1]^pc[16:9]^pc[24:17].
  // Same reasoning as tc_index above.
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
