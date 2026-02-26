`timescale 1ns/1ps
import trace_cache_pkg::*;

// trace_cache_top.sv
//
// Top-level trace cache wrapper.
// Connects the fetch stream to trace_builder for recording,
// stores completed traces in a 64-entry direct-mapped SRAM,
// and handles lookup requests (hit/miss + trace data).
//
// HOW A HIT WORKS:
//   1. Frontend presents fetch address (icache_vaddr_q) as lookup_pc_i
//   2. We read SRAM[index(lookup_pc_i)] in one cycle
//   3. Next cycle: compare tag:
//        - pc_match:           stored base_pc (aligned) == lookup_pc (aligned)
//        - branch_flags_match: stored predictions == current predictions
//   4. If both match and entry is valid ? HIT ? deliver trace instructions
//      and target_addr directly, bypassing icache fetch latency
//
// WHY LOOKUP PC IS ALIGNED:
//   base_pc is stored fetch-aligned (builder masks pc[0] to 16-byte boundary).
//   lookup_pc_i (icache_vaddr_q) may be halfword-aligned due to RVC, so we
//   derive lookup_base = lookup_pc_i & ~0xF before indexing SRAM and comparing.

module trace_cache_top (
  input  logic clk_i,
  input  logic rst_ni,

  // Instruction window from the frontend (4 slots per cycle)
  input  logic [SLOTS_PER_CYCLE-1:0]               instr_valid_i,
  input  logic [SLOTS_PER_CYCLE-1:0][31:0]         instr_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] pc_i,
  input  logic [SLOTS_PER_CYCLE-1:0]               is_branch_i,
  input  logic [SLOTS_PER_CYCLE-1:0]               branch_taken_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] branch_target_i,
  // From instr_realign: 1 when current window spans blocks / is still being
  // stitched, so the trace builder should not start a new base window.
  input  logic                        serving_unaligned_i,

  input  logic                        flush_i,
  input  logic                        instr_queue_ready_i,
  input  logic [SLOTS_PER_CYCLE-1:0]  instr_queue_consumed_i,

  // Branch predictions for the current fetch window.
  // Used in TWO ways:
  //   1. At RECORD time: passed into trace_builder to store as branch_flags tag
  //   2. At LOOKUP time: compared against stored branch_flags for tag matching
  // This dual use ensures record and lookup use the same prediction semantics.
  input  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_i,

  // Lookup interface
  input  logic                lookup_valid_i,
  input  logic [PC_WIDTH-1:0] lookup_pc_i,

  // Lookup results
  output logic                       trace_hit_o,
  output logic [TRACE_LEN-1:0][31:0] trace_instructions_o,
  output logic [4:0]                 trace_length_o,
  output logic [PC_WIDTH-1:0]        trace_next_pc_o
);

  // Bundle instruction window signals into the interface for trace_builder
  tracebuilder_instr_if instr_if (
    .clk_i (clk_i),
    .rst_ni(rst_ni)
  );

  assign instr_if.valid     = instr_valid_i & {SLOTS_PER_CYCLE{instr_queue_ready_i & ~flush_i}};
  assign instr_if.pc        = pc_i;
  assign instr_if.inst      = instr_i;
  assign instr_if.is_branch = is_branch_i;
  assign instr_if.taken     = branch_taken_i;
  assign instr_if.target    = branch_target_i;
  assign instr_if.consumed  = instr_queue_consumed_i & {SLOTS_PER_CYCLE{~flush_i}};
  assign instr_if.serving_unaligned = serving_unaligned_i;

  // GHR - tracked for potential future use, not used in current tag matching
  logic [GHR_WIDTH-1:0] ghr;
  tc_ghr i_tc_ghr (
    .clk_i,
    .rst_ni,
    .flush_i,
    .branch_valid_i (|(instr_valid_i & is_branch_i)),
    .branch_taken_i (|(instr_valid_i & is_branch_i & branch_taken_i)),
    .ghr_o          (ghr)
  );

  // SRAM signals
  logic                   mem_req;
  logic                   mem_we;
  logic [TRACE_ADDRW-1:0] mem_addr;
  logic [TRACE_WIDTH-1:0] mem_wdata;
  logic [BE_WIDTH-1:0]    mem_be;
  logic [TRACE_WIDTH-1:0] mem_rdata;

  // Builder write-side signals
  logic                   mem_req_builder;
  logic                   mem_we_builder;
  logic [TRACE_ADDRW-1:0] mem_addr_builder;
  logic [TRACE_WIDTH-1:0] mem_wdata_builder;
  logic [BE_WIDTH-1:0]    mem_be_builder;

  trace_builder i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i                (instr_if),
    .ghr_i                  (ghr),
    .flush_i                (flush_i),
    // Pass predictions into builder so it stores predicted bits as tag,
    // not actual taken bits (see trace_builder.sv header for why)
    .branch_predictions_i   (branch_predictions_i),
    .trace_valid_o          (),
    .trace_data_o           (),
    .mem_req_o              (mem_req_builder),
    .mem_we_o               (mem_we_builder),
    .mem_addr_o             (mem_addr_builder),
    .mem_wdata_o            (mem_wdata_builder),
    .mem_be_o               (mem_be_builder)
  );

  // Lookup pipeline registers (SRAM has 1-cycle read latency)
  logic                        lookup_fire;
  logic                        lookup_valid_q;
  logic [PC_WIDTH-1:0]         lookup_pc_q;
  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_q;

  // Builder writes take priority over lookups (never stall the record path)
  assign lookup_fire = lookup_valid_i && !mem_req_builder;

  // Align lookup PC to 16-byte fetch window boundary.
  // icache_vaddr_q can be halfword-aligned (e.g. 0x...16) due to RVC.
  // builder always stores base_pc fetch-aligned, so we must align lookup
  // to the same boundary for both SRAM index and tag compare to match.
  logic [PC_WIDTH-1:0] lookup_base;
  assign lookup_base = lookup_pc_i & {{(PC_WIDTH-4){1'b1}}, 4'b0000};

  // Single-port SRAM arbitration: write when builder commits, else read for lookup
  always_comb begin
    mem_req   = 1'b0;
    mem_we    = 1'b0;
    mem_addr  = '0;
    mem_wdata = '0;
    mem_be    = {BE_WIDTH{1'b1}};

    if (mem_req_builder) begin
      // Builder is committing a trace - write takes priority
      mem_req   = mem_req_builder;
      mem_we    = mem_we_builder;
      mem_addr  = mem_addr_builder;
      mem_wdata = mem_wdata_builder;
      mem_be    = mem_be_builder;
    end else if (lookup_fire) begin
      // No write this cycle - do a lookup read using aligned base address
      mem_req  = 1'b1;
      mem_we   = 1'b0;
      mem_addr = lookup_base[TRACE_ADDRW+1:2];
      mem_be   = {BE_WIDTH{1'b1}};
    end
  end

  // Pipeline lookup inputs to align with 1-cycle SRAM read latency.
  // We pipeline lookup_base (aligned), not raw lookup_pc_i.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lookup_valid_q       <= 1'b0;
      lookup_pc_q          <= '0;
      branch_predictions_q <= '0;
    end else begin
      lookup_valid_q       <= lookup_fire;
      lookup_pc_q          <= lookup_base;
      branch_predictions_q <= branch_predictions_i;
    end
  end

  tc_sram #(
    .NumWords  (1 << TRACE_ADDRW),
    .DataWidth (TRACE_WIDTH),
    .NumPorts  (1),
    .Latency   (1)
  ) i_trace_sram (
    .clk_i,
    .rst_ni,
    .req_i   ({mem_req}),
    .we_i    ({mem_we}),
    .addr_i  ({mem_addr}),
    .wdata_i ({mem_wdata}),
    .be_i    ({mem_be}),
    .rdata_o ({mem_rdata})
  );

  // Cast SRAM output to struct
  trace_data_t trace_read;
  assign trace_read = mem_rdata;

  // Tag check: PC match
  // Both base_pc (stored by builder) and lookup_pc_q are fetch-aligned to
  // 16-byte boundary. base_pc is masked in builder; lookup_pc_q is lookup_base
  // pipelined above. Direct compare is correct.
  logic pc_match;
  assign pc_match = (trace_read.base_pc == lookup_pc_q);

  // Tag check: branch prediction match
  // Compare stored branch_flags (predictions at record time) against
  // current predictions. Only compare as many bits as there are branches
  // in the base window (num_branches set by builder in IDLE state).
  // ACCUM branches are NOT part of the tag.
  logic branch_flags_match;
  always_comb begin
    branch_flags_match = 1'b1;
    for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
      if (i < int'(trace_read.num_branches)) begin
        if (branch_predictions_q[i] != trace_read.branch_flags[i])
          branch_flags_match = 1'b0;
      end
    end
  end

  // HIT = valid entry + PC in same fetch window + branch predictions match
  logic trace_hit;
  assign trace_hit       = trace_read.valid && pc_match && branch_flags_match && lookup_valid_q;
  assign trace_hit_o     = trace_hit;
  assign trace_next_pc_o = trace_read.target_addr;

  // Count instructions in trace (valid_chunks[i]=1 marks an instruction start)
  logic [4:0] instr_count;
  always_comb begin
    instr_count = '0;
    for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
      if (trace_read.valid_chunks[i])
        instr_count = instr_count + 1;
    end
  end
  assign trace_length_o = trace_read.valid ? instr_count : 5'b0;

  // Reconstruct 32-bit instructions from 16-bit chunks.
  // valid_chunks[i]=1 means chunk i is the LOW half of an instruction.
  // If valid_chunks[i+1]=0, it is a 32-bit instruction (two chunks).
  // Otherwise it is a 16-bit compressed instruction (one chunk).
  always_comb begin
    int instr_idx;
    int chunk_idx;
    trace_instructions_o = '0;
    instr_idx = 0;
    chunk_idx = 0;
    while (chunk_idx < CHUNKS_PER_TRACE && instr_idx < TRACE_LEN) begin
      if (trace_read.valid_chunks[chunk_idx]) begin
        if (chunk_idx + 1 < CHUNKS_PER_TRACE && !trace_read.valid_chunks[chunk_idx + 1]) begin
          // 32-bit instruction: low half at chunk_idx, high half at chunk_idx+1
          trace_instructions_o[instr_idx] = {trace_read.chunks[chunk_idx + 1],
                                              trace_read.chunks[chunk_idx]};
          chunk_idx += 2;
        end else begin
          // 16-bit compressed instruction
          trace_instructions_o[instr_idx] = {16'b0, trace_read.chunks[chunk_idx]};
          chunk_idx += 1;
        end
        instr_idx++;
      end else begin
        chunk_idx++;
      end
    end
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (lookup_valid_q && trace_read.valid) begin
      if (!pc_match)
        $display("[TC-LOOKUP] PC MISS: stored=0x%h lookup=0x%h", trace_read.base_pc, lookup_pc_q);
      else if (!branch_flags_match)
        $display("[TC-LOOKUP] BR MISS at 0x%h: stored=%b lookup=%b num=%0d",
                 lookup_pc_q, trace_read.branch_flags, branch_predictions_q, trace_read.num_branches);
      else
        $display("[TC-LOOKUP] HIT at 0x%h", lookup_pc_q);
    end
  end

  // Useful hit rate: only counts lookups where the SRAM slot is occupied,
  // so empty-slot PC misses (sequential windows with no stored trace) are excluded.
  int unsigned tc_valid_lookups;
  int unsigned tc_useful_hits;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_valid_lookups <= 0;
      tc_useful_hits   <= 0;
    end else if (lookup_valid_q && trace_read.valid) begin
      tc_valid_lookups <= tc_valid_lookups + 1;
      if (trace_hit)
        tc_useful_hits <= tc_useful_hits + 1;
    end
  end

  final begin
    $display("[TC-USEFUL] valid_lookups=%0d hits=%0d rate=%0d%%",
             tc_valid_lookups, tc_useful_hits,
             tc_valid_lookups > 0 ? (tc_useful_hits * 100) / tc_valid_lookups : 0);
  end
`endif
endmodule