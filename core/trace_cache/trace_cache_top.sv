`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// V90 skewed-associative trace cache with multi-taken-branch support.
// V90: MAX_TAKEN raised to 3.  Builder spans multiple fetch windows to
//      record traces with up to 3 taken branches.  Simple replay policy
//      (accept length ? 3, no profitability filter).
// Each way uses a different hash function (H0 / H1) so that PCs
// colliding in one way's set likely map to different sets in the other
// way.  This breaks hot-set concentration.
// Traces are 1 fetch window (4 instructions, 1 taken branch max).
// V85: builder now accepts returns (target from RAS prediction) and
// direct calls (PC-relative target).  Frontend syncs the RAS during
// trace replay.

module trace_cache_top #(
  parameter int unsigned MaxTraceInstr = MAX_TRACE_INSTR
) (
  input  logic clk_i,
  input  logic rst_ni,

  // Instruction window from the frontend (4 slots per cycle)
  input  logic [SLOTS_PER_CYCLE-1:0]                  instr_valid_i,
  input  logic [SLOTS_PER_CYCLE-1:0][INSTR_WIDTH-1:0] instr_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0]    pc_i,
  input  logic [SLOTS_PER_CYCLE-1:0]                  is_branch_i,
  input  logic [SLOTS_PER_CYCLE-1:0]                  branch_taken_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0]    branch_target_i,
  input  logic                                        serving_unaligned_i,

  input  logic                        flush_i,
  // V99 (Option B): high the cycle a TC replay fires.  Forwarded to the
  // trace_builder so it can drop a stale TB_FILL state instead of
  // waiting forever for a target window the redirected fetch will not
  // deliver.
  input  logic                        tc_replay_fired_i,
  input  logic                        instr_queue_ready_i,
  input  logic [SLOTS_PER_CYCLE-1:0]  instr_queue_consumed_i,

  input  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_i,
  input  logic [BR_CNT_WIDTH-1:0]     lookup_num_branches_i,
  input  logic                        window_has_taken_pred_i, // V79: >=1 predicted-taken CF in this window

  // Resolved branch (from backend): correct-path outcomes to store as path tag, matching Rotenberg fill-at-retire semantics
  input  logic                         resolved_branch_valid_i,
  input  logic [PC_WIDTH-1:0]          resolved_branch_pc_i,
  input  logic                         resolved_branch_is_taken_i,
  input  logic                         resolved_branch_is_mispredict_i,
  input  logic [GHR_WIDTH-1:0]         resolved_branch_ghr_i, // V81: pipeline-propagated GHR snapshot

  // Current GHR value (for frontend to attach to predictions)
  output logic [GHR_WIDTH-1:0]         ghr_o,                 // V81: current GHR for pipeline propagation

  // Lookup interface
  input  logic                         lookup_valid_i,
  input  logic [PC_WIDTH-1:0]          lookup_pc_i,
  // V94: 1 = frontend fired this lookup as the early-unaligned variant
  // (lookup_pc_i is the address of the unaligned instruction that next
  // cycle's instr_realign will deliver as addr[0]). The result cycle is
  // therefore expected to have serving_unaligned_i==1; otherwise the
  // predicted unaligned delivery did not actually happen (flush /
  // mispredict / pipeline stall) and the lookup result is meaningless.
  input  logic                         lookup_predicts_unaligned_i,

  // Lookup results
  output logic                                        trace_hit_o,
  output logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]       trace_instructions_o,
  output logic [TRACE_LEN_WIDTH-1:0]                  trace_length_o,
  output logic [PC_WIDTH-1:0]                         trace_next_pc_o,
  output logic [CHUNKS_PER_TRACE-1:0][15:0]           trace_chunks_o,
  output logic [CHUNKS_PER_TRACE-1:0]                 trace_valid_chunks_o,
  output logic [TRACE_LEN-1:0][PC_WIDTH-1:0]          trace_pcs_o,
  output logic [CHUNKS_PER_TRACE-1:0]                 trace_branch_flags_o,
  output logic [BR_CNT_WIDTH-1:0]                     trace_num_branches_o,

  // Ordered targets of taken control-flow instructions in this trace
  output logic [MAX_TAKEN-1:0][PC_WIDTH-1:0]          trace_taken_targets_o,
  output logic [TAKEN_CNT_WIDTH-1:0]                  trace_num_taken_o,
  output logic                                        trace_used_o,

  // High when this cycle's trace_hit_o is for a lookup we actually did (we fired); use for hit/miss counting
  output logic                                        lookup_result_valid_o,

  // When lookup_result_valid_o and !trace_hit_o: exactly one of these is 1 (why we missed). Frontend counts these.
  output logic                                        miss_reason_empty_o,
  output logic                                        miss_reason_pc_o,
  output logic                                        miss_reason_path_o,

  // Miss breakdown for debug (0 in synthesis)
  output logic [31:0]                                 tc_miss_total_o,
  output logic [31:0]                                 tc_miss_empty_o,
  output logic [31:0]                                 tc_miss_pc_o,
  output logic [31:0]                                 tc_miss_path_o,
  input  logic                                        mark_used_i
);

  initial assert (MaxTraceInstr <= TRACE_LEN)
    else $fatal(1, "trace_cache_top: MaxTraceInstr (%0d) must be <= TRACE_LEN (%0d)", MaxTraceInstr, TRACE_LEN);

  tracebuilder_instr_if instr_if (
    .clk_i (clk_i),
    .rst_ni(rst_ni)
  );

  assign instr_if.valid             = instr_valid_i & {SLOTS_PER_CYCLE{instr_queue_ready_i & ~flush_i}};
  assign instr_if.pc                = pc_i;
  assign instr_if.inst              = instr_i;
  assign instr_if.is_branch         = is_branch_i;
  assign instr_if.taken             = branch_taken_i;
  assign instr_if.target            = branch_target_i;
  // IMPORTANT: consumed must be aligned with the same fetch-window domain as
  // instr_if.{pc,inst,valid}. Using instr_queue_consumed_i here mixes two
  // different timing domains and can pair a slot PC with a stale instruction.
  // Feed the builder with the accepted fetch-window mask instead.
  assign instr_if.consumed          = instr_if.valid;
  assign instr_if.serving_unaligned = serving_unaligned_i;

  // V81: Forward declarations for GHR restore (used by tc_ghr instantiation below)
  logic                  is_mispredict_event;
  logic                  ghr_restore_valid;
  logic [GHR_WIDTH-1:0]  ghr_restore_value;

  logic [GHR_WIDTH-1:0] ghr;
  tc_ghr i_tc_ghr (
    .clk_i,
    .rst_ni,
    .flush_i          (flush_i && !is_mispredict_event),  // V81: don't reset on mispredict
    .branch_valid_i   (|(instr_valid_i & is_branch_i)),
    .branch_taken_i   (|(instr_valid_i & is_branch_i & branch_taken_i)),
    .restore_valid_i  (ghr_restore_valid),   // V81: pipeline restore
    .restore_ghr_i    (ghr_restore_value),   // V81: restored GHR value
    .ghr_o            (ghr)
  );

  // V81: Expose GHR for frontend to attach to branch predictions
  assign ghr_o = ghr;

  logic                   mem_req   [NUM_WAYS];
  logic                   mem_we    [NUM_WAYS];
  logic [TRACE_ADDRW-1:0] mem_addr  [NUM_WAYS];
  logic [TRACE_WIDTH-1:0] mem_wdata [NUM_WAYS];
  logic [BE_WIDTH-1:0]    mem_be    [NUM_WAYS];
  logic [TRACE_WIDTH-1:0] mem_rdata [NUM_WAYS];
  logic                   tag_req   [NUM_WAYS];
  logic                   tag_we    [NUM_WAYS];
  logic [TRACE_ADDRW-1:0] tag_addr  [NUM_WAYS];
  trace_tag_t             tag_wdata [NUM_WAYS];
  trace_tag_t             tag_rdata [NUM_WAYS];

  logic                   mem_req_builder;
  logic                   mem_we_builder;
  logic [TRACE_ADDRW-1:0] mem_addr_builder;
  trace_tag_t             mem_tag_builder;
  logic [TRACE_WIDTH-1:0] mem_wdata_builder;
  logic [BE_WIDTH-1:0]    mem_be_builder;
  logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] mem_branch_pcs_builder;

  // -----------------------------------------------------------------------
  // V81 PIPELINE GHR RESTORE: On misprediction, the branch unit sends back
  // the GHR snapshot that was attached to the instruction when it was
  // dispatched.  This is the pre-branch GHR ? we restore it with the
  // correct outcome shifted in.  100% accurate (no hash table collisions).
  //
  // Replaces V80's 16-entry PC-indexed checkpoint table.
  // -----------------------------------------------------------------------
  assign is_mispredict_event = resolved_branch_valid_i & resolved_branch_is_mispredict_i;

  // V81: Restored GHR = pipeline snapshot with correct outcome shifted in
  assign ghr_restore_valid = is_mispredict_event;
  assign ghr_restore_value = {resolved_branch_ghr_i[GHR_WIDTH-2:0],
                               resolved_branch_is_taken_i};

  logic [TRACE_WIDTH-1:0] mem_wdata_final;
  trace_tag_t             mem_tag_final;
  assign mem_wdata_final = mem_wdata_builder;
  assign mem_tag_final   = mem_tag_builder;

  trace_builder #(
    .MAX_INSTR_PER_TRACE(MaxTraceInstr)
  ) i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i          (instr_if),
    .ghr_i            (ghr),
    .flush_i          (flush_i),
    .tc_replay_fired_i(tc_replay_fired_i),
    .trace_valid_o    (),
    .trace_data_o     (),
    .mem_tag_o        (mem_tag_builder),
    .mem_req_o        (mem_req_builder),
    .mem_we_o         (mem_we_builder),
    .mem_addr_o       (mem_addr_builder),
    .mem_wdata_o      (mem_wdata_builder),
    .mem_be_o         (mem_be_builder),
    .mem_branch_pcs_o (mem_branch_pcs_builder)
  );

  logic                        lookup_fire;
  logic                        lookup_valid_q;
  logic [PC_WIDTH-1:0]         lookup_pc_q;
  logic [TRACE_ADDRW-1:0]      lookup_set_q;
  logic [TRACE_ADDRW-1:0]      lookup_set_w1_q;  // V83: way-1 set index for FF valid lookup
  logic [GHR_WIDTH-1:0]        lookup_ghr_q;  // V82: registered GHR for tag compare

  // V94: registered "this lookup targeted the unaligned PC of the cycle that
  // is now the result cycle". Used to check that the actual delivery cycle
  // matches the prediction (serving_unaligned_i must agree).
  logic lookup_predicted_unaligned_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)            lookup_predicted_unaligned_q <= 1'b0;
    else if (lookup_fire)   lookup_predicted_unaligned_q <= lookup_predicts_unaligned_i;
    else                    lookup_predicted_unaligned_q <= 1'b0;
  end

  // Tag comparison consumes LIVE branch_predictions_i / lookup_num_branches_i
  // from the result cycle. With the V94 early-unaligned lookup, the result
  // cycle's window IS the window the lookup targeted (aligned-for-aligned,
  // unaligned-for-unaligned), so the live predictions correctly describe it.
  // No more prev-cycle registers, no more eff_* selectors.
  logic lookup_eligible;

  // -----------------------------------------------------------------------
  // V74 DEDUP GUARD: 2-phase commit pipeline.
  //
  // Phase 1 (dedup_check_fire): Builder requests commit ? capture commit
  //   data in pipeline registers, issue tag reads for all ways at the
  //   target set.  Lookups are blocked this cycle (tag ports busy).
  //
  // Phase 2 (dedup_pending_q high): Compare read-back tags against pending
  //   commit.  If any way already holds a matching tag ? suppress write
  //   (dedup_hit).  Otherwise ? proceed with SRAM write (dedup_write_fire).
  //   Suppressed commits free the SRAM ports immediately for lookups.
  //
  // Tag match criterion: valid && base_pc && num_branches && branch_flags
  // (same fields as the lookup tag comparison).  Two traces with the same
  // tag traverse the same instruction sequence ? their data payloads are
  // deterministically identical ? so a tag-only check is sufficient.
  // -----------------------------------------------------------------------

  // Pipeline registers (hold commit data across the tag-read cycle)
  logic                    dedup_pending_q;
  logic [TRACE_ADDRW-1:0]  dedup_addr_q;       // way-0 (H0) set index
  logic [TRACE_ADDRW-1:0]  dedup_addr_w1_q;    // way-1 (H1) set index (V77)
  trace_tag_t              dedup_tag_q;
  logic [TRACE_WIDTH-1:0]  dedup_data_q;
  logic [BE_WIDTH-1:0]     dedup_be_q;
  logic                    dedup_wr_way_q;

  // Dedup comparison result (combinational ? see always_comb after gen_ways)
  logic [NUM_WAYS-1:0]     dedup_way_match;
  logic                    dedup_hit;

  // V84: dedup revalidation ? suppress matched an FF-invalid entry (SRAM still has data)
  logic [NUM_WAYS-1:0]     dedup_way_ff_invalid;  // matched way was FF-invalid
  logic                    dedup_revalidate;      // suppress AND matched entry was FF-invalid
  logic                    dedup_revalidate_way;  // which way to re-validate
  logic [TRACE_ADDRW-1:0]  dedup_revalidate_addr; // set index to re-validate

  // Phase control signals (active for exactly one cycle each)
  logic                    dedup_check_fire;     // Phase 1: builder commit ? tag read
  logic                    dedup_write_fire;     // Phase 2: no match ? SRAM write
  logic                    dedup_suppress_fire;  // Phase 2: match ? skip write
  logic                    actual_write;         // = dedup_write_fire

  assign dedup_check_fire    = mem_req_builder;
  assign dedup_write_fire    = dedup_pending_q && !dedup_hit;
  assign dedup_suppress_fire = dedup_pending_q &&  dedup_hit;
  assign actual_write        = dedup_write_fire;

  assign lookup_fire           = lookup_valid_i && !dedup_check_fire && !dedup_write_fire;
  assign lookup_result_valid_o = lookup_valid_q;
  // V94: a lookup result is meaningful when:
  //   1. it actually fired (lookup_valid_q),
  //   2. the result-cycle window has >=1 predicted-taken CF (V79: every
  //      stored trace has >=1 taken branch, so a no-taken window can't hit),
  //   3. the lookup's predicted alignment matches the actual delivery
  //      (predicted-unaligned <-> serving_unaligned_i). When they disagree,
  //      a flush/mispredict cancelled the predicted unaligned cycle (or vice
  //      versa) and the SRAM result describes a window that isn't being
  //      served ? suppress to avoid polluting hit/miss counters.
  assign lookup_eligible = lookup_valid_q &&
                           window_has_taken_pred_i &&
                           (lookup_predicted_unaligned_q == serving_unaligned_i);

  // IMPORTANT: exact trace-start PC, not a 16-byte aligned block base.
  logic [PC_WIDTH-1:0] lookup_base;
  assign lookup_base = lookup_pc_i;

  // V71: branch_predictions and num_branches are used LIVE (not registered)
  // at tag-comparison time.  They arrive from the frontend's instruction
  // scan in the same cycle as the SRAM read-data, one cycle after the
  // SRAM read-request.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lookup_valid_q        <= 1'b0;
      lookup_pc_q           <= '0;
      lookup_set_q          <= '0;
      lookup_set_w1_q       <= '0;  // V83
      lookup_ghr_q          <= '0;  // V82
    end else begin
      lookup_valid_q        <= lookup_fire;
      lookup_pc_q           <= lookup_base;
      lookup_set_q          <= tc_index(lookup_base, ghr);
      lookup_set_w1_q       <= tc_index_w1(lookup_base, ghr);  // V83: way-1 set for FF valid
      lookup_ghr_q          <= ghr;  // V82: capture GHR at lookup time for tag compare
    end
  end

  // V77: per-way builder indices (combinational, from builder's base_pc)
  logic [TRACE_ADDRW-1:0] builder_idx_w0;
  logic [TRACE_ADDRW-1:0] builder_idx_w1;
  assign builder_idx_w0 = tc_index(mem_tag_builder.base_pc, mem_tag_builder.ghr);  // V82: GHR from builder tag
  assign builder_idx_w1 = tc_index_w1(mem_tag_builder.base_pc, mem_tag_builder.ghr);  // V82

  logic lru [(1 << TRACE_ADDRW)];
  logic wr_way;
  assign wr_way = ~lru[builder_idx_w0];  // LRU keyed on H0 index

  // -----------------------------------------------------------------------
  // V83 SPECULATIVE TRACE INVALIDATION
  //
  // Valid bits sit in flip-flops (not SRAM) for single-cycle clearing.
  // A speculative write log tracks recent builder writes.  On misprediction,
  // logged entries are invalidated.  On correct branch resolution, the log
  // is flushed (writes become permanent).  512 flip-flops total (2 ways x
  // 256 sets).
  // -----------------------------------------------------------------------
  localparam int SPEC_LOG_DEPTH = 8;

  logic valid_ff [NUM_WAYS][0:(1 << TRACE_ADDRW)-1];

  logic                              spec_log_valid [SPEC_LOG_DEPTH];
  logic                              spec_log_way   [SPEC_LOG_DEPTH];
  logic [TRACE_ADDRW-1:0]            spec_log_set   [SPEC_LOG_DEPTH];
  logic [$clog2(SPEC_LOG_DEPTH)-1:0] spec_log_head;

  // Helper: per-way FF valid at the dedup read addresses
  logic dedup_ff_valid [NUM_WAYS];
  // NOTE: dedup_addr_q / dedup_addr_w1_q are registered in the dedup pipeline
  // below; these assigns are valid after that pipeline register is loaded.

  // Dedup pipeline register update
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dedup_pending_q <= 1'b0;
      dedup_addr_q    <= '0;
      dedup_addr_w1_q <= '0;
      dedup_tag_q     <= '0;
      dedup_data_q    <= '0;
      dedup_be_q      <= '0;
      dedup_wr_way_q  <= 1'b0;
    end else if (flush_i) begin
      dedup_pending_q <= 1'b0;
    end else begin
      if (dedup_check_fire) begin
        dedup_pending_q <= 1'b1;
        dedup_addr_q    <= builder_idx_w0;
        dedup_addr_w1_q <= builder_idx_w1;
        dedup_tag_q     <= mem_tag_final;
        dedup_data_q    <= mem_wdata_final;
        dedup_be_q      <= mem_be_builder;
        dedup_wr_way_q  <= wr_way;
      end else begin
        dedup_pending_q <= 1'b0;
      end
    end
  end

  // V83: dedup_ff_valid ? FF valid bits at the dedup read addresses
  assign dedup_ff_valid[0] = valid_ff[0][dedup_addr_q];
  assign dedup_ff_valid[1] = valid_ff[1][dedup_addr_w1_q];

  // -----------------------------------------------------------------------
  //
  // With skewed indexing each way reads/writes at its own hash index.
  // LRU is keyed on H0 (way-0) index ? an approximation, but the main
  // benefit comes from conflict reduction, not replacement precision.
  // -----------------------------------------------------------------------
  logic final_wr_way;
  // V77: per-way write address (selected after final_wr_way is known)
  logic [TRACE_ADDRW-1:0] dedup_wr_addr;
  logic [TRACE_ADDRW-1:0] dedup_other_addr;
  always_comb begin
    final_wr_way = dedup_wr_way_q;  // default: LRU choice
    if (dedup_pending_q && !dedup_hit) begin
      // V83: use FF valid bits (not SRAM) for empty-slot detection
      if (dedup_ff_valid[dedup_wr_way_q] && !dedup_ff_valid[~dedup_wr_way_q])
        final_wr_way = ~dedup_wr_way_q;  // prefer empty slot
    end
    dedup_wr_addr    = final_wr_way ? dedup_addr_w1_q : dedup_addr_q;
    dedup_other_addr = final_wr_way ? dedup_addr_q    : dedup_addr_w1_q;
  end

  // SRAM port arbiter: dedup write > dedup check (tag read) > lookup
  always_comb begin
    for (int w = 0; w < NUM_WAYS; w++) begin
      mem_req[w]   = 1'b0;
      mem_we[w]    = 1'b0;
      mem_addr[w]  = '0;
      mem_wdata[w] = '0;
      mem_be[w]    = {BE_WIDTH{1'b1}};
      tag_req[w]   = 1'b0;
      tag_we[w]    = 1'b0;
      tag_addr[w]  = '0;
      tag_wdata[w] = '0;
    end

    if (dedup_write_fire) begin
      // Phase 2: dedup miss ? actual write to chosen way at its skewed index
      mem_req[final_wr_way]   = 1'b1;
      mem_we[final_wr_way]    = 1'b1;
      mem_addr[final_wr_way]  = dedup_wr_addr;
      mem_wdata[final_wr_way] = dedup_data_q;
      mem_be[final_wr_way]    = dedup_be_q;
      tag_req[final_wr_way]   = 1'b1;
      tag_we[final_wr_way]    = 1'b1;
      tag_addr[final_wr_way]  = dedup_wr_addr;
      tag_wdata[final_wr_way] = dedup_tag_q;
    end else if (dedup_check_fire) begin
      // Phase 1: read tags ? each way at its own skewed index (V77)
      tag_req[0]  = 1'b1;
      tag_we[0]   = 1'b0;
      tag_addr[0] = builder_idx_w0;
      tag_req[1]  = 1'b1;
      tag_we[1]   = 1'b0;
      tag_addr[1] = builder_idx_w1;
    end else if (lookup_fire) begin
      // V77: each way reads at its own skewed hash index
      mem_req[0]  = 1'b1;
      mem_we[0]   = 1'b0;
      mem_addr[0] = tc_index(lookup_base, ghr);  // V82: GHR in index
      tag_req[0]  = 1'b1;
      tag_we[0]   = 1'b0;
      tag_addr[0] = tc_index(lookup_base, ghr);
      mem_req[1]  = 1'b1;
      mem_we[1]   = 1'b0;
      mem_addr[1] = tc_index_w1(lookup_base, ghr);  // V82: GHR in index
      tag_req[1]  = 1'b1;
      tag_we[1]   = 1'b0;
      tag_addr[1] = tc_index_w1(lookup_base, ghr);
    end
  end

  for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_ways
    tag_sram i_tag_sram (
      .clk_i,
      .rst_ni,
      .req_i   (tag_req[w]),
      .we_i    (tag_we[w]),
      .addr_i  (tag_addr[w]),
      .wdata_i (tag_wdata[w]),
      .rdata_o (tag_rdata[w])
    );

    trace_data_sram #(
      .NumWords  (1 << TRACE_ADDRW),
      .DataWidth (TRACE_WIDTH),
      .NumPorts  (1),
      .Latency   (1)
    ) i_trace_sram (
      .clk_i,
      .rst_ni,
      .req_i   ({mem_req[w]}),
      .we_i    ({mem_we[w]}),
      .addr_i  ({mem_addr[w]}),
      .wdata_i ({mem_wdata[w]}),
      .be_i    ({mem_be[w]}),
      .rdata_o ({mem_rdata[w]})
    );
  end

  // -----------------------------------------------------------------------
  // V83: FF valid-bit update logic
  // -----------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int w = 0; w < NUM_WAYS; w++)
        for (int s = 0; s < (1 << TRACE_ADDRW); s++)
          valid_ff[w][s] <= 1'b0;
    end else if (flush_i && !is_mispredict_event) begin
      // Non-misprediction flush (e.g. fence.i): clear ALL valid bits
      for (int w = 0; w < NUM_WAYS; w++)
        for (int s = 0; s < (1 << TRACE_ADDRW); s++)
          valid_ff[w][s] <= 1'b0;
    end else begin
      // Misprediction: invalidate only speculative-write-log entries
      if (is_mispredict_event) begin
        for (int i = 0; i < SPEC_LOG_DEPTH; i++) begin
          if (spec_log_valid[i])
            valid_ff[spec_log_way[i]][spec_log_set[i]] <= 1'b0;
        end
      end
      // Set valid on actual write (suppress if concurrent misprediction ?
      // that write is itself speculative)
      if (actual_write && !is_mispredict_event)
        valid_ff[final_wr_way][dedup_wr_addr] <= 1'b1;
      // V84: re-validate FF bit when dedup matches an FF-invalid SRAM entry
      if (dedup_revalidate && !is_mispredict_event)
        valid_ff[dedup_revalidate_way][dedup_revalidate_addr] <= 1'b1;
    end
  end

  // V83: Speculative write log management
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < SPEC_LOG_DEPTH; i++)
        spec_log_valid[i] <= 1'b0;
      spec_log_head <= '0;
    end else if (flush_i || (resolved_branch_valid_i && !resolved_branch_is_mispredict_i)) begin
      // Misprediction flush: entries already consumed above, clear log.
      // Correct branch resolution: writes confirmed, clear log.
      // Non-misprediction flush: start fresh.
      for (int i = 0; i < SPEC_LOG_DEPTH; i++)
        spec_log_valid[i] <= 1'b0;
      spec_log_head <= '0;
    end else if (actual_write || dedup_revalidate) begin
      // V84: also log revalidated entries ? they're still speculative
      spec_log_valid[spec_log_head] <= 1'b1;
      spec_log_way[spec_log_head]   <= actual_write ? final_wr_way      : dedup_revalidate_way;
      spec_log_set[spec_log_head]   <= actual_write ? dedup_wr_addr     : dedup_revalidate_addr;
      spec_log_head <= spec_log_head + 1;  // wraps naturally at SPEC_LOG_DEPTH
    end
  end

  // -----------------------------------------------------------------------
  // V84 Dedup comparison: tag read-back from Phase 1 vs pending commit tag.
  // Uses SRAM valid (not FF valid) so that FF-invalidated entries still
  // suppress duplicate writes.  On suppress of an FF-invalid entry, the
  // FF bit is re-validated (the data is already in SRAM).
  // -----------------------------------------------------------------------
  always_comb begin
    for (int w = 0; w < NUM_WAYS; w++) begin
      logic dd_flags_ok;
      dd_flags_ok = 1'b1;
      for (int b = 0; b < TRIGGER_BRANCH_BITS; b++) begin
        if (b < int'(dedup_tag_q.num_branches))
          dd_flags_ok &= (tag_rdata[w].branch_flags[b] == dedup_tag_q.branch_flags[b]);
      end
      dedup_way_match[w] = dedup_pending_q
                         && tag_rdata[w].valid  // V84: SRAM valid (catches FF-invalid entries)
                         && (tag_rdata[w].base_pc      == dedup_tag_q.base_pc)
                         && (tag_rdata[w].num_branches  == dedup_tag_q.num_branches)
                         && dd_flags_ok;
      dedup_way_ff_invalid[w] = dedup_way_match[w] && !dedup_ff_valid[w];  // V84
    end
    dedup_hit = |dedup_way_match;
    // V84: revalidation ? the matched entry needs its FF bit restored
    dedup_revalidate = dedup_suppress_fire && |dedup_way_ff_invalid;
    dedup_revalidate_way  = dedup_way_ff_invalid[1] ? 1'b1 : 1'b0;
    dedup_revalidate_addr = dedup_way_ff_invalid[1] ? dedup_addr_w1_q : dedup_addr_q;
  end

  trace_data_t trace_read [NUM_WAYS];
  trace_tag_t  trace_tag_read [NUM_WAYS];
  logic [NUM_WAYS-1:0] way_valid;
  logic [NUM_WAYS-1:0] pc_match;
  logic [NUM_WAYS-1:0] branch_count_match;
  logic [NUM_WAYS-1:0] branch_flags_match;
  logic [NUM_WAYS-1:0] tag_hit_raw;
  logic [NUM_WAYS-1:0] way_hit;
  trace_tag_t          lookup_tag_dbg [NUM_WAYS];

  for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_tag_cmp
    assign trace_read[w] = mem_rdata[w];
    assign trace_tag_read[w] = tag_rdata[w];
    // V83: hit detection uses FF valid bits (single-cycle invalidation on mispredict)
    assign way_valid[w]  = valid_ff[w][w == 0 ? lookup_set_q : lookup_set_w1_q];
    assign pc_match[w]   = way_valid[w] && (trace_tag_read[w].base_pc == lookup_pc_q);
    // V71: comparison uses LIVE branch_predictions_i / lookup_num_branches_i
    // (provided by the frontend in the same cycle as SRAM read-data).
    assign branch_count_match[w] = way_valid[w] &&
                                   (trace_tag_read[w].num_branches ==
                                    TRIGGER_BRANCH_CNT_WIDTH'(lookup_num_branches_i));

    // V94: tag compare uses LIVE branch_predictions_i / lookup_num_branches_i.
    // The early-unaligned lookup makes the result cycle's window match the
    // lookup target (aligned for aligned, unaligned for unaligned), so the
    // live predictions correctly describe the window the SRAM tag should be
    // compared against ? no prev-cycle compensation needed.
    trace_tag_compare i_trace_tag_compare (
      .base_pc_i      (lookup_pc_q),
      .num_branches_i (TRIGGER_BRANCH_CNT_WIDTH'(lookup_num_branches_i)),
      .branch_flags_i (branch_predictions_i[TRIGGER_BRANCH_BITS-1:0]),
      .ghr_i          (lookup_ghr_q),  // V82: path history
      .stored_tag_i   (trace_tag_read[w]),
      .hit_o          (tag_hit_raw[w]),
      .lookup_tag_o   (lookup_tag_dbg[w])
    );

    always_comb begin
      branch_flags_match[w] = 1'b0;
      if (way_valid[w]) begin
        branch_flags_match[w] = 1'b1;
        for (int i = 0; i < TRIGGER_BRANCH_BITS; i++) begin
          if (i < int'(trace_tag_read[w].num_branches)) begin
            if (branch_predictions_i[i] != trace_tag_read[w].branch_flags[i])
              branch_flags_match[w] = 1'b0;
          end
        end
      end
    end
  end

  logic [NUM_WAYS-1:0] raw_way_hit;
  always_comb begin
    for (int w = 0; w < NUM_WAYS; w++) begin
      // V78: suppress lookup result when serving an unaligned instruction
      // that straddles two 16B fetch blocks.  In the straddle cycle the I$
      // returns the aligned block-base address (e.g. 0x80002880), so
      // tc_lookup_pc / lookup_pc_q = that block base.  But instr_realign
      // sets addr[0] = the unaligned instruction PC (e.g. 0x8000287e) ?
      // which is what the builder uses for accum_base_pc.  The SRAM was
      // indexed with H0/H1 of the block base, which is a different set than
      // what a stored trace at addr[0] occupies.  The result is always wrong;
      // suppress it (miss) rather than signalling a false pc_mismatch.
      raw_way_hit[w] = tag_hit_raw[w] & lookup_eligible;
      way_hit[w]     = raw_way_hit[w];  // V68: no used_q filter; frontend rehit-block handles replay guards
    end
  end

  logic raw_trace_hit;
  logic trace_hit;
  assign raw_trace_hit = |raw_way_hit;
  assign trace_hit     = |way_hit;
  assign trace_hit_o   = trace_hit;

  logic any_valid_in_set;
  logic any_pc_match_in_set;
  always_comb begin
    any_valid_in_set    = 1'b0;
    any_pc_match_in_set = 1'b0;
    for (int w = 0; w < NUM_WAYS; w++) begin
      if (way_valid[w])                any_valid_in_set    = 1'b1;
      if (way_valid[w] && pc_match[w]) any_pc_match_in_set = 1'b1;
    end
  end

  assign miss_reason_empty_o = lookup_eligible && !trace_hit && !any_valid_in_set;
  assign miss_reason_pc_o    = lookup_eligible && !trace_hit &&  any_valid_in_set && !any_pc_match_in_set;
  assign miss_reason_path_o  = lookup_eligible && !trace_hit &&  any_valid_in_set &&  any_pc_match_in_set;

  logic [$clog2(NUM_WAYS)-1:0] raw_hit_way_idx;
  logic [$clog2(NUM_WAYS)-1:0] hit_way_idx;
  always_comb begin
    raw_hit_way_idx = '0;
    for (int w = NUM_WAYS-1; w >= 0; w--) begin
      if (raw_way_hit[w]) raw_hit_way_idx = w[$clog2(NUM_WAYS)-1:0];
    end
  end

  always_comb begin
    hit_way_idx = '0;
    for (int w = NUM_WAYS-1; w >= 0; w--) begin
      if (way_hit[w]) hit_way_idx = w[$clog2(NUM_WAYS)-1:0];
    end
  end

  trace_data_t hit_trace;
  always_comb begin
    hit_trace = '0;
    if (trace_hit)
      hit_trace = trace_read[hit_way_idx];
  end

  assign trace_next_pc_o       = hit_trace.target_addr;
  assign trace_chunks_o        = hit_trace.chunks;
  assign trace_valid_chunks_o  = hit_trace.valid_chunks;
  assign trace_branch_flags_o  = hit_trace.branch_flags;
  assign trace_num_branches_o  = hit_trace.num_branches;
  assign trace_taken_targets_o = hit_trace.taken_targets;
  assign trace_num_taken_o     = hit_trace.num_taken;

  // Reconstruct instructions from 16-bit chunks.
  // For a 32-bit instruction, valid_chunks[k]=1 on the low half and valid_chunks[k+1]=0 on the high half.
  logic [TRACE_LEN_WIDTH-1:0] trace_instr_count;
  always_comb begin
    int instr_idx;
    int chunk_idx;
    logic [15:0] low16;

    trace_instructions_o = '0;
    trace_instr_count    = '0;

    instr_idx = 0;
    chunk_idx = 0;

    while ((chunk_idx < CHUNKS_PER_TRACE) && (instr_idx < TRACE_LEN)) begin
      if (!hit_trace.valid_chunks[chunk_idx]) begin
        chunk_idx++;
      end else begin
        low16 = hit_trace.chunks[chunk_idx];

        // 32-bit instruction
        if ((low16[1:0] == 2'b11) &&
            (chunk_idx + 1 < CHUNKS_PER_TRACE) &&
            (hit_trace.valid_chunks[chunk_idx + 1] === 1'b0)) begin
          trace_instructions_o[instr_idx] = {hit_trace.chunks[chunk_idx + 1], low16};
          instr_idx++;
          chunk_idx += 2;
        end
        // 16-bit compressed instruction
        else begin
          trace_instructions_o[instr_idx] = {16'b0, low16};
          instr_idx++;
          chunk_idx += 1;
        end
      end
    end

    trace_instr_count = TRACE_LEN_WIDTH'(instr_idx);
  end

  assign trace_length_o = trace_hit ? trace_instr_count : '0;

  // Reconstruct PCs from exact start PC, expanded instructions, and branch_flags.
  logic [TRACE_LEN-1:0][PC_WIDTH-1:0] computed_pcs;
  always_comb begin
    logic [PC_WIDTH-1:0]    pc;
    logic [INSTR_WIDTH-1:0] instr;
    logic                   is_rvc, is_cf, taken;
    int                     br_idx;
    int                     taken_idx;

    computed_pcs = '0;
    trace_pcs_o  = '0;

    if (!trace_hit) begin
      pc        = '0;
      br_idx    = 0;
      taken_idx = 0;
    end else begin
      pc        = hit_trace.base_pc;
      br_idx    = 0;
      taken_idx = 0;

      for (int i = 0; i < TRACE_LEN; i++) begin
        if (i >= int'(trace_instr_count)) break;

        instr  = trace_instructions_o[i];
        is_rvc = (instr[1:0] != 2'b11);

        is_cf  = (!is_rvc && (instr[6:0] == OpcodeBranch ||
                              instr[6:0] == OpcodeJal    ||
                              instr[6:0] == OpcodeJalr))
              || ( is_rvc && (
                     (((instr[15:13] == OpcodeC1J)    ||
                       (instr[15:13] == OpcodeC1Beqz) ||
                       (instr[15:13] == OpcodeC1Bnez)) &&
                      (instr[1:0]   == OpcodeC1)) ||
                     ((instr[15:13] == OpcodeC2JalrMvAdd) &&
                      (instr[6:2]   == 5'b00000) &&
                      (instr[1:0]   == OpcodeC2))
                   ));

        taken = 1'b0;
        if (is_cf) begin
          taken = (br_idx < int'(hit_trace.num_branches)) && hit_trace.branch_flags[br_idx];
          br_idx++;
        end

        computed_pcs[i] = pc;

        if (taken && (taken_idx < int'(hit_trace.num_taken))) begin
          pc = hit_trace.taken_targets[taken_idx];
          taken_idx++;
        end else begin
          pc = pc + (is_rvc ? PC_WIDTH'(2) : PC_WIDTH'(4));
        end
      end

      for (int j = 0; j < TRACE_LEN; j++)
        trace_pcs_o[j] = (j < int'(trace_instr_count)) ? computed_pcs[j] : '0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // V77: alternating init ? even sets?0, odd sets?1
      for (int s = 0; s < (1 << TRACE_ADDRW); s++)
        lru[s] <= s[0];
    end else if (flush_i) begin
      for (int s = 0; s < (1 << TRACE_ADDRW); s++)
        lru[s] <= s[0];
    end else begin
      if (trace_hit)
        lru[lookup_set_q] <= hit_way_idx[0];
      if (actual_write)
        lru[dedup_addr_q] <= final_wr_way;  // H0-keyed LRU
    end
  end

  // V70: Traces are freely reusable on every loop iteration.
  assign trace_used_o = 1'b0;

`ifdef MODEL_TECH
  initial $display("[TC-DEBUG] trace_cache_top: miss breakdown ACTIVE (MODEL_TECH defined)");

  int unsigned tc_miss_empty;
  int unsigned tc_miss_pc;
  int unsigned tc_miss_path;
  int unsigned tc_miss_total;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_miss_empty <= 0;
      tc_miss_pc    <= 0;
      tc_miss_path  <= 0;
      tc_miss_total <= 0;
    end else if (lookup_eligible && !trace_hit) begin
      tc_miss_total <= tc_miss_total + 1;
      if (!any_valid_in_set)
        tc_miss_empty <= tc_miss_empty + 1;
      else if (!any_pc_match_in_set)
        tc_miss_pc <= tc_miss_pc + 1;
      else
        tc_miss_path <= tc_miss_path + 1;
    end
  end

  assign tc_miss_total_o = tc_miss_total;
  assign tc_miss_empty_o = tc_miss_empty;
  assign tc_miss_pc_o    = tc_miss_pc;
  assign tc_miss_path_o  = tc_miss_path;
`else
  initial $display("[TC-DEBUG] trace_cache_top: miss breakdown DISABLED (MODEL_TECH not defined - add +define+MODEL_TECH to compile)");
  assign tc_miss_total_o = 32'b0;
  assign tc_miss_empty_o = 32'b0;
  assign tc_miss_pc_o    = 32'b0;
  assign tc_miss_path_o  = 32'b0;
`endif

`ifndef SYNTHESIS
  // -----------------------------------------------------------------------
  // V76 PC-MISMATCH DEBUG INSTRUMENTATION
  //
  // Goal: distinguish genuine set aliasing (different PCs hashing to same
  //       set) from potential bugs in lookup-PC or stored base_pc.
  // -----------------------------------------------------------------------
  localparam int PC_MISS_VERBOSE_LIMIT = 200;   // first N events printed
  localparam int PC_MISS_TOP_N        = 16;     // top-N tracking depth

  int unsigned pcm_event_count;                  // total pc_mismatch events
  int unsigned pcm_1valid;                       // mismatch with exactly 1 valid way
  int unsigned pcm_2valid;                       // mismatch with 2 valid ways

  // Top-N tracking via associative arrays (simulation only)
  int unsigned pcm_set_hist [int unsigned];       // set_index ? count
  int unsigned pcm_lookup_pc_hist [int unsigned];  // lookup PC[31:0] ? count
  int unsigned pcm_stored_pc_hist [int unsigned];  // stored base_pc[31:0] ? count

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pcm_event_count <= 0;
      pcm_1valid      <= 0;
      pcm_2valid      <= 0;
    end else if (lookup_eligible && !trace_hit && any_valid_in_set && !any_pc_match_in_set) begin
      // This is a pc_mismatch event
      pcm_event_count <= pcm_event_count + 1;

      // Count 1-valid vs 2-valid
      begin
        int nv;
        nv = 0;
        for (int w = 0; w < NUM_WAYS; w++)
          if (way_valid[w]) nv++;
        if (nv == 1)
          pcm_1valid <= pcm_1valid + 1;
        else
          pcm_2valid <= pcm_2valid + 1;
      end

      // Accumulate histograms
      if (pcm_set_hist.exists(32'(lookup_set_q)))
        pcm_set_hist[32'(lookup_set_q)] += 1;
      else
        pcm_set_hist[32'(lookup_set_q)] = 1;

      if (pcm_lookup_pc_hist.exists(32'(lookup_pc_q)))
        pcm_lookup_pc_hist[32'(lookup_pc_q)] += 1;
      else
        pcm_lookup_pc_hist[32'(lookup_pc_q)] = 1;

      for (int w = 0; w < NUM_WAYS; w++) begin
        if (way_valid[w]) begin
          if (pcm_stored_pc_hist.exists(32'(trace_tag_read[w].base_pc)))
            pcm_stored_pc_hist[32'(trace_tag_read[w].base_pc)] += 1;
          else
            pcm_stored_pc_hist[32'(trace_tag_read[w].base_pc)] = 1;
        end
      end

      // Verbose per-event print for first N events
      if (pcm_event_count < PC_MISS_VERBOSE_LIMIT) begin
        $display("[TC-PCMISS] #%0d t=%0t lookup_pc=0x%h set=%0d nvalid=%0d",
                 pcm_event_count, $time, lookup_pc_q, lookup_set_q,
                 (way_valid[0] ? 1 : 0) + (way_valid[1] ? 1 : 0));
        for (int w = 0; w < NUM_WAYS; w++) begin
          if (way_valid[w])
            $display("[TC-PCMISS]   way%0d: stored_base_pc=0x%h nbr=%0d bflags=%b",
                     w, trace_tag_read[w].base_pc, trace_tag_read[w].num_branches,
                     trace_tag_read[w].branch_flags);
          else
            $display("[TC-PCMISS]   way%0d: INVALID", w);
        end
        // NOTE: shadow tag arrays are declared later; use tag_rdata (live SRAM readback) instead
      end
    end
  end

  // Final summary display for PC-mismatch analysis
  function automatic void pcm_print_top_n(
    input string label,
    input int unsigned hist [int unsigned],
    input int n
  );
    // Simple top-N extraction via repeated max-find
    int unsigned keys [$];
    int unsigned vals [$];
    int unsigned best_k, best_v;

    foreach (hist[k]) begin
      keys.push_back(k);
      vals.push_back(hist[k]);
    end

    for (int i = 0; i < n && i < keys.size(); i++) begin
      best_v = 0;
      best_k = 0;
      for (int j = i; j < keys.size(); j++) begin
        if (vals[j] > best_v) begin
          best_v = vals[j];
          best_k = keys[j];
          // swap to position i
          keys[j] = keys[i]; keys[i] = best_k;
          vals[j] = vals[i]; vals[i] = best_v;
        end
      end
      $display("[TC-PCMISS] %s #%0d: key=0x%h count=%0d",
               label, i, keys[i], vals[i]);
    end
  endfunction

  // pcm_print_top_n_pc removed ? all histograms now use int unsigned keys

  `ifdef TRACE_CACHE_DEBUG_VERBOSE
  always_ff @(posedge clk_i) begin
    if (lookup_valid_q) begin
      logic any_valid;
      any_valid = 1'b0;
      for (int w = 0; w < NUM_WAYS; w++)
        if (way_valid[w]) any_valid = 1'b1;

      if (trace_hit) begin
        $display("[TC-LOOKUP] HIT at 0x%h (way %0d)", lookup_pc_q, hit_way_idx);
      end else if (any_valid) begin
        for (int w = 0; w < NUM_WAYS; w++) begin
          if (way_valid[w] && !pc_match[w])
            $display("[TC-LOOKUP] lookup PC 0x%h missed (way %0d had base_pc 0x%h)",
                     lookup_pc_q, w, trace_tag_read[w].base_pc);
          else if (way_valid[w] && pc_match[w] &&
                   (!branch_count_match[w] || !branch_flags_match[w]))
            $display("[TC-LOOKUP] BR MISS at 0x%h (way %0d): stored=%b lookup=%b stored_num=%0d lookup_num=%0d",
                     lookup_pc_q, w, trace_tag_read[w].branch_flags,
                     branch_predictions_i[TRIGGER_BRANCH_BITS-1:0],
                     trace_tag_read[w].num_branches, lookup_num_branches_i);
        end
      end
    end
  end
  `endif

  int unsigned tc_valid_lookups;
  int unsigned tc_useful_hits;
  int unsigned tc_recorded_traces;
  int unsigned tc_lookup_requests;
  int unsigned tc_lookup_blocked_by_builder;
  int unsigned tc_tag_payload_mismatch;
  int unsigned tc_unaligned_suppressed; // V78: lookups suppressed due to serving_unaligned
  int unsigned tc_no_taken_suppressed;  // V79: lookups suppressed due to no predicted-taken CF
  int unsigned tc_ghr_restores;         // V81: GHR restored from pipeline on mispredict
  int unsigned tc_ghr_resets;           // V81: GHR reset to 0 (non-mispredict flush)
  int unsigned tc_mispredicts_seen;     // V81: total mispredicts observed
  int unsigned tc_spec_invalidations;   // V83: traces invalidated on misprediction
  int unsigned tc_spec_log_clears;      // V83: spec log cleared (correct resolution)
  int unsigned tc_spec_log_overflows;   // V83: spec log head wrapped before clear
  int unsigned tc_dedup_revalidations;  // V84: dedup suppress re-validated an FF-invalid entry

  // -----------------------------------------------------------------------
  // V73 COMMIT DEBUG: Shadow tag/data arrays + churn analysis counters.
  //
  // "Trace signature" definition:
  //   sig = {base_pc, num_branches, branch_flags[0:num_branches-1], target_addr}
  // Two traces with the same signature traverse the same instruction path
  // from the same start PC through the same branch outcomes to the same
  // exit PC.  Identical signatures ? functionally identical traces.
  //
  // Counter semantics (how they distinguish real diversity from churn):
  //   commit_into_invalid   ? cold-fill into an empty/reset slot.
  //                           High early, drops once SRAM is warm.
  //   commit_replace_valid  ? eviction of a live entry.
  //                           ? total churn pressure.
  //   commit_replace_same_sig ? exact re-recording of the same trace.
  //                           Pure waste / duplicate recording bug.
  //   commit_replace_same_pc_diff_path ? same base_pc, different branch
  //                           path.  Real path diversity IF distinct paths
  //                           are both useful; churn if they keep evicting
  //                           each other.
  //   commit_replace_diff_pc ? different base_pc.  Capacity/aliasing miss
  //                           eviction; unrelated traces compete for the
  //                           same set.
  //   commit_other_way_same_sig ? the OTHER way already holds an entry
  //                           with the exact same signature.  This is a
  //                           duplicate-recording bug: two ways store the
  //                           same trace.
  //   commit_other_way_same_pc ? the OTHER way has the same base_pc but a
  //                           different path.  This means both branch
  //                           variants of a PC coexist ? real path
  //                           diversity utilising both ways.
  // -----------------------------------------------------------------------

  // Shadow tag array ? mirrors the tag SRAMs, updated on every builder
  // write, so we can read the "about-to-be-replaced" tag combinationally
  // in the same cycle the write fires.
  trace_tag_t  shadow_tag [NUM_WAYS][0:(1 << TRACE_ADDRW)-1];
  // Shadow validity + base_pc + target_addr from trace data (for signature).
  logic        shadow_data_valid [NUM_WAYS][0:(1 << TRACE_ADDRW)-1];
  logic [PC_WIDTH-1:0] shadow_data_target [NUM_WAYS][0:(1 << TRACE_ADDRW)-1];

  // Commit classification counters.
  int unsigned cmt_into_invalid;
  int unsigned cmt_replace_valid;
  int unsigned cmt_replace_same_sig;
  int unsigned cmt_replace_same_pc_diff_path;
  int unsigned cmt_replace_diff_pc;
  int unsigned cmt_other_way_same_sig;
  int unsigned cmt_other_way_same_pc;
  // Trace length histogram for committed traces.
  int unsigned cmt_len_hist [2:TRACE_LEN];
  // Committed trace histogram by number of taken control-flow instructions.
  int unsigned cmt_taken_hist [0:MAX_TAKEN];

  // Combinational classification signals (computed every cycle, consumed
  // by always_ff on actual_write).
  logic        cmt_old_valid;
  logic        cmt_same_sig;
  logic        cmt_same_pc;
  logic        cmt_oth_valid;
  logic        cmt_oth_same_sig;
  logic        cmt_oth_same_pc;
  int          cmt_new_instr_cnt;
  logic [PC_WIDTH-1:0] cmt_new_target_addr;
  logic [TAKEN_CNT_WIDTH-1:0] cmt_new_num_taken;
  logic [PC_WIDTH-1:0] cmt_new_taken_tgt0;
  logic [PC_WIDTH-1:0] cmt_new_taken_tgt1;
  logic [PC_WIDTH-1:0] cmt_new_taken_tgt2;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int w = 0; w < NUM_WAYS; w++)
        for (int s = 0; s < (1 << TRACE_ADDRW); s++) begin
          shadow_tag[w][s]         <= '0;
          shadow_data_valid[w][s]  <= 1'b0;
          shadow_data_target[w][s] <= '0;
        end
    end else if (actual_write) begin
      shadow_tag[final_wr_way][dedup_wr_addr]         <= dedup_tag_q;
      shadow_data_valid[final_wr_way][dedup_wr_addr]  <= 1'b1;
      shadow_data_target[final_wr_way][dedup_wr_addr] <= cmt_new_target_addr;
    end
  end

  always_comb begin
    // --- Replaced entry classification ---
    trace_tag_t  c_old_tag;
    logic [PC_WIDTH-1:0] c_old_target;
    trace_tag_t  c_new_tag;
    trace_data_t c_new_data;
    logic c_same_nbr, c_same_flags, c_same_target;

    c_old_tag    = shadow_tag[final_wr_way][dedup_wr_addr];
    c_old_target = shadow_data_target[final_wr_way][dedup_wr_addr];
    c_new_tag    = dedup_tag_q;
    c_new_data   = dedup_data_q;

    cmt_new_target_addr = c_new_data.target_addr;
    cmt_new_num_taken   = c_new_data.num_taken;
    cmt_new_taken_tgt0  = c_new_data.taken_targets[0];
    cmt_new_taken_tgt1  = c_new_data.taken_targets[1];
    cmt_new_taken_tgt2  = c_new_data.taken_targets[2];

    cmt_old_valid = c_old_tag.valid && shadow_data_valid[final_wr_way][dedup_wr_addr];
    cmt_same_pc   = (c_old_tag.base_pc == c_new_tag.base_pc);
    c_same_nbr    = (c_old_tag.num_branches == c_new_tag.num_branches);
    c_same_flags  = 1'b1;
    for (int b = 0; b < TRIGGER_BRANCH_BITS; b++) begin
      if (b < int'(c_old_tag.num_branches))
        c_same_flags &= (c_old_tag.branch_flags[b] == c_new_tag.branch_flags[b]);
    end
    c_same_target = (c_old_target == c_new_data.target_addr);
    cmt_same_sig  = cmt_same_pc && c_same_nbr && c_same_flags && c_same_target;

    // --- Other way classification ---
    begin
      int c_other_way;
      trace_tag_t  c_oth_tag;
      logic [PC_WIDTH-1:0] c_oth_target;
      logic c_oth_same_nbr, c_oth_same_flags, c_oth_same_target;

      c_other_way  = final_wr_way ? 0 : 1;
      c_oth_tag    = shadow_tag[c_other_way][dedup_other_addr];
      c_oth_target = shadow_data_target[c_other_way][dedup_other_addr];

      cmt_oth_valid   = c_oth_tag.valid && shadow_data_valid[c_other_way][dedup_other_addr];
      cmt_oth_same_pc = (c_oth_tag.base_pc == c_new_tag.base_pc);
      c_oth_same_nbr  = (c_oth_tag.num_branches == c_new_tag.num_branches);
      c_oth_same_flags = 1'b1;
      for (int b = 0; b < TRIGGER_BRANCH_BITS; b++) begin
        if (b < int'(c_oth_tag.num_branches))
          c_oth_same_flags &= (c_oth_tag.branch_flags[b] == c_new_tag.branch_flags[b]);
      end
      c_oth_same_target = (c_oth_target == c_new_data.target_addr);
      cmt_oth_same_sig  = cmt_oth_same_pc && c_oth_same_nbr && c_oth_same_flags && c_oth_same_target;
    end

    // --- Instruction count from chunks ---
    begin
      int c_cidx;
      cmt_new_instr_cnt = 0;
      c_cidx = 0;
      while (c_cidx < CHUNKS_PER_TRACE) begin
        if (c_new_data.valid_chunks[c_cidx]) begin
          if ((c_new_data.chunks[c_cidx][1:0] == 2'b11) && (c_cidx + 1 < CHUNKS_PER_TRACE))
            c_cidx += 2;
          else
            c_cidx += 1;
          cmt_new_instr_cnt++;
        end else
          c_cidx++;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cmt_into_invalid            <= 0;
      cmt_replace_valid           <= 0;
      cmt_replace_same_sig        <= 0;
      cmt_replace_same_pc_diff_path <= 0;
      cmt_replace_diff_pc         <= 0;
      cmt_other_way_same_sig      <= 0;
      cmt_other_way_same_pc       <= 0;
      for (int l = 2; l <= TRACE_LEN; l++)
        cmt_len_hist[l] <= 0;
      for (int t = 0; t <= MAX_TAKEN; t++)
        cmt_taken_hist[t] <= 0;
    end else if (actual_write) begin
      if (!cmt_old_valid) begin
        cmt_into_invalid <= cmt_into_invalid + 1;
      end else begin
        cmt_replace_valid <= cmt_replace_valid + 1;
        if (cmt_same_sig)
          cmt_replace_same_sig <= cmt_replace_same_sig + 1;
        else if (cmt_same_pc)
          cmt_replace_same_pc_diff_path <= cmt_replace_same_pc_diff_path + 1;
        else
          cmt_replace_diff_pc <= cmt_replace_diff_pc + 1;
      end

      if (cmt_oth_valid) begin
        if (cmt_oth_same_sig)
          cmt_other_way_same_sig <= cmt_other_way_same_sig + 1;
        else if (cmt_oth_same_pc)
          cmt_other_way_same_pc <= cmt_other_way_same_pc + 1;
      end

      if (cmt_new_instr_cnt >= 2 && cmt_new_instr_cnt <= TRACE_LEN)
        cmt_len_hist[cmt_new_instr_cnt] <= cmt_len_hist[cmt_new_instr_cnt] + 1;

      case (cmt_new_num_taken)
        TAKEN_CNT_WIDTH'(0): cmt_taken_hist[0] <= cmt_taken_hist[0] + 1;
        TAKEN_CNT_WIDTH'(1): cmt_taken_hist[1] <= cmt_taken_hist[1] + 1;
        TAKEN_CNT_WIDTH'(2): cmt_taken_hist[2] <= cmt_taken_hist[2] + 1;
        default:             cmt_taken_hist[MAX_TAKEN] <= cmt_taken_hist[MAX_TAKEN] + 1;
      endcase
    end
  end

  // --- Per-commit verbose $display (first 512 + every 50000th) ---
  int unsigned cmt_display_count;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      cmt_display_count <= 0;
    else if (actual_write) begin
      cmt_display_count <= cmt_display_count + 1;

      if (cmt_display_count < 512 || (cmt_display_count % 50000 == 0)) begin
        $display("[TC-COMMIT] #%0d t=%0t set_h0=%0d set_h1=%0d way=%0d | new: base_pc=0x%h nbr=%0d ntaken=%0d bflags=%b tgt=0x%h len=%0d taken_tgt[0]=0x%h taken_tgt[1]=0x%h taken_tgt[2]=0x%h",
                 cmt_display_count, $time,
                 dedup_addr_q, dedup_addr_w1_q, final_wr_way,
                 dedup_tag_q.base_pc[31:0],
                 dedup_tag_q.num_branches,
                 cmt_new_num_taken,
                 dedup_tag_q.branch_flags,
                 cmt_new_target_addr[31:0],
                 cmt_new_instr_cnt,
                 cmt_new_taken_tgt0[31:0],
                 cmt_new_taken_tgt1[31:0],
                 cmt_new_taken_tgt2[31:0]);

        if (cmt_old_valid)
          $display("[TC-COMMIT]   replaced: base_pc=0x%h nbr=%0d bflags=%b tgt=0x%h %s",
                   shadow_tag[final_wr_way][dedup_wr_addr].base_pc[31:0],
                   shadow_tag[final_wr_way][dedup_wr_addr].num_branches,
                   shadow_tag[final_wr_way][dedup_wr_addr].branch_flags,
                   shadow_data_target[final_wr_way][dedup_wr_addr][31:0],
                   cmt_same_pc ? "SAME_PC" : "DIFF_PC");
        else
          $display("[TC-COMMIT]   replaced: INVALID (cold fill)");
      end
    end
  end

  // -----------------------------------------------------------------------
  // V74 Dedup counters + V75 LRU override counter
  // -----------------------------------------------------------------------
  int unsigned tc_dedup_attempts;       // total builder commit requests
  int unsigned tc_dedup_skipped;        // suppressed by dedup guard
  int unsigned tc_dedup_match_chosen;   // match was in LRU-chosen (victim) way
  int unsigned tc_dedup_match_other;    // match was in the other way
  int unsigned tc_lru_invalid_override; // V75: times we overrode LRU to use empty slot
  int unsigned tc_wr_way0;             // V75: writes to way 0
  int unsigned tc_wr_way1;             // V75: writes to way 1

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_dedup_attempts      <= 0;
      tc_dedup_skipped       <= 0;
      tc_dedup_match_chosen  <= 0;
      tc_dedup_match_other   <= 0;
      tc_lru_invalid_override <= 0;
      tc_wr_way0             <= 0;
      tc_wr_way1             <= 0;
    end else begin
      if (dedup_check_fire)
        tc_dedup_attempts <= tc_dedup_attempts + 1;
      if (dedup_suppress_fire) begin
        tc_dedup_skipped <= tc_dedup_skipped + 1;
        if (dedup_way_match[dedup_wr_way_q])
          tc_dedup_match_chosen <= tc_dedup_match_chosen + 1;
        if (dedup_way_match[~dedup_wr_way_q])
          tc_dedup_match_other <= tc_dedup_match_other + 1;
      end
      if (actual_write) begin
        if (final_wr_way != dedup_wr_way_q)
          tc_lru_invalid_override <= tc_lru_invalid_override + 1;
        if (final_wr_way == 1'b0)
          tc_wr_way0 <= tc_wr_way0 + 1;
        else
          tc_wr_way1 <= tc_wr_way1 + 1;
      end
    end
  end

  // -----------------------------------------------------------------------
  // V77 Hot-PC dedup/write tracker: monitors the lifecycle of the top
  // mismatch PCs through the dedup pipeline.
  // -----------------------------------------------------------------------
  localparam logic [PC_WIDTH-1:0] HP0 = 64'h80002880;
  localparam logic [PC_WIDTH-1:0] HP1 = 64'h8000287a;
  localparam logic [PC_WIDTH-1:0] HP2 = 64'h80002888;
  localparam logic [PC_WIDTH-1:0] HP3 = 64'h8000287e;

  int unsigned hp_dedup_attempt [4];   // entered dedup pipeline
  int unsigned hp_dedup_skip    [4];   // suppressed by dedup guard
  int unsigned hp_dedup_write   [4];   // actually written to SRAM
  logic [PC_WIDTH-1:0] hp_list [4];

  initial begin
    hp_list[0] = HP0; hp_list[1] = HP1; hp_list[2] = HP2; hp_list[3] = HP3;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < 4; i++) begin
        hp_dedup_attempt[i] <= 0;
        hp_dedup_skip[i]    <= 0;
        hp_dedup_write[i]   <= 0;
      end
    end else begin
      for (int i = 0; i < 4; i++) begin
        if (dedup_check_fire && mem_tag_final.base_pc == hp_list[i])
          hp_dedup_attempt[i] <= hp_dedup_attempt[i] + 1;
        if (dedup_suppress_fire && dedup_tag_q.base_pc == hp_list[i]) begin
          hp_dedup_skip[i] <= hp_dedup_skip[i] + 1;
          if (hp_dedup_skip[i] < 5)
            $display("[TC-HOTPC] t=%0t DEDUP_SKIP base_pc=0x%h way_match=%b (HP%0d)",
                     $time, dedup_tag_q.base_pc[31:0], dedup_way_match, i);
        end
        if (actual_write && dedup_tag_q.base_pc == hp_list[i]) begin
          hp_dedup_write[i] <= hp_dedup_write[i] + 1;
          $display("[TC-HOTPC] t=%0t DEDUP_WRITE base_pc=0x%h way=%0d set_h0=%0d set_h1=%0d (HP%0d)",
                   $time, dedup_tag_q.base_pc[31:0], final_wr_way,
                   dedup_addr_q, dedup_addr_w1_q, i);
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_valid_lookups <= 0;
      tc_useful_hits   <= 0;
      tc_recorded_traces <= 0;
      tc_lookup_requests <= 0;
      tc_lookup_blocked_by_builder <= 0;
      tc_tag_payload_mismatch <= 0;
      tc_unaligned_suppressed <= 0;
      tc_no_taken_suppressed <= 0;
      tc_ghr_restores <= 0;
      tc_ghr_resets <= 0;
      tc_mispredicts_seen <= 0;
      tc_spec_invalidations <= 0;
      tc_spec_log_clears <= 0;
      tc_spec_log_overflows <= 0;
      tc_dedup_revalidations <= 0;
    end else begin
      if (actual_write)
        tc_recorded_traces <= tc_recorded_traces + 1;

      if (lookup_valid_i)
        tc_lookup_requests <= tc_lookup_requests + 1;
      if (lookup_valid_i && (dedup_check_fire || dedup_write_fire))
        tc_lookup_blocked_by_builder <= tc_lookup_blocked_by_builder + 1;

      // V94: counters reflect the alignment-mismatch suppression. With the
      // early-unaligned lookup the prediction usually matches delivery; this
      // increments only when a flush/mispredict cancels the predicted cycle
      // (predicted aligned but delivery unaligned, or vice versa).
      if (lookup_valid_q && (lookup_predicted_unaligned_q != serving_unaligned_i))
        tc_unaligned_suppressed <= tc_unaligned_suppressed + 1;
      else if (lookup_valid_q && !window_has_taken_pred_i)
        tc_no_taken_suppressed <= tc_no_taken_suppressed + 1;

      // V81: GHR pipeline restore counters
      if (is_mispredict_event) begin
        tc_mispredicts_seen <= tc_mispredicts_seen + 1;
        tc_ghr_restores <= tc_ghr_restores + 1;  // V81: always restores (pipeline propagation)
        // V83: count speculative invalidations
        begin
          int n_inval;
          n_inval = 0;
          for (int i = 0; i < SPEC_LOG_DEPTH; i++)
            if (spec_log_valid[i]) n_inval++;
          tc_spec_invalidations <= tc_spec_invalidations + n_inval;
        end
      end
      if (flush_i && !is_mispredict_event)
        tc_ghr_resets <= tc_ghr_resets + 1;

      // V83: spec log clears on correct resolution
      if (resolved_branch_valid_i && !resolved_branch_is_mispredict_i)
        tc_spec_log_clears <= tc_spec_log_clears + 1;
      // V83: spec log overflow (head wraps with valid entry still present)
      if (actual_write && spec_log_valid[spec_log_head])
        tc_spec_log_overflows <= tc_spec_log_overflows + 1;
      // V84: count dedup revalidations
      if (dedup_revalidate)
        tc_dedup_revalidations <= tc_dedup_revalidations + 1;

      if (lookup_eligible) begin
        logic any_valid;
        any_valid = 1'b0;
        for (int w = 0; w < NUM_WAYS; w++)
          if (way_valid[w]) any_valid = 1'b1;
        if (any_valid) begin
          tc_valid_lookups <= tc_valid_lookups + 1;
          if (trace_hit)
            tc_useful_hits <= tc_useful_hits + 1;
        end

        if (trace_hit && (trace_read[hit_way_idx].valid !== 1'b1))
          tc_tag_payload_mismatch <= tc_tag_payload_mismatch + 1;
      end
    end
  end

  final begin
    int unsigned tc_useful_rate;
    if (tc_valid_lookups > 0)
      tc_useful_rate = (tc_useful_hits * 100) / tc_valid_lookups;
    else
      tc_useful_rate = 0;

    $display("[TC-BUILDER] recorded_traces=%0d lookup_requests=%0d blocked_by_builder=%0d",
             tc_recorded_traces, tc_lookup_requests, tc_lookup_blocked_by_builder);
    $display("[TC-DEDUP] attempts=%0d skipped=%0d written=%0d skip_rate=%0d%%",
             tc_dedup_attempts, tc_dedup_skipped,
             tc_dedup_attempts - tc_dedup_skipped,
             tc_dedup_attempts > 0 ? (tc_dedup_skipped * 100) / tc_dedup_attempts : 0);
    $display("[TC-DEDUP] match_chosen_way=%0d match_other_way=%0d",
             tc_dedup_match_chosen, tc_dedup_match_other);

    // V77 Hot-PC lifecycle summary
    $display("[TC-HOTPC-DEDUP] ========== Hot PC Dedup Lifecycle ==========");
    for (int i = 0; i < 4; i++)
      $display("[TC-HOTPC-DEDUP] HP%0d (0x%h): attempt=%0d skip=%0d write=%0d",
               i, hp_list[i][31:0], hp_dedup_attempt[i], hp_dedup_skip[i], hp_dedup_write[i]);
    $display("[TC-HOTPC-DEDUP] ===========================================");
    $display("[TC-LRU] invalid_override=%0d wr_way0=%0d wr_way1=%0d (V77 skewed-index + prefer-invalid)",
             tc_lru_invalid_override, tc_wr_way0, tc_wr_way1);
    $display("[TC-USEFUL] valid_lookups=%0d hits=%0d rate=%0d%%",
             tc_valid_lookups, tc_useful_hits, tc_useful_rate);
    $display("[TC-USEFUL] unaligned_suppressed=%0d no_taken_suppressed=%0d (V78+V79)",
             tc_unaligned_suppressed, tc_no_taken_suppressed);
    $display("[TC-GHR] mispredicts_seen=%0d restores=%0d resets=%0d (V84 pipeline+index+tag+spec_inval+revalid)",
             tc_mispredicts_seen, tc_ghr_restores, tc_ghr_resets);
    $display("[TC-SPEC] invalidations=%0d log_clears=%0d log_overflows=%0d revalidations=%0d (V84 spec inval + dedup revalid)",
             tc_spec_invalidations, tc_spec_log_clears, tc_spec_log_overflows, tc_dedup_revalidations);
    $display("[TC-SPLIT-TAG] payload_mismatch=%0d (tag hit but payload slot was invalid)",
             tc_tag_payload_mismatch);
    `ifdef MODEL_TECH
    $display("[TC-MISS-BREAKDOWN] total_misses=%0d empty=%0d pc_mismatch=%0d path_mismatch=%0d (why lookups missed)",
             tc_miss_total, tc_miss_empty, tc_miss_pc, tc_miss_path);
    `endif

    // V73 commit churn analysis
    $display("[TC-CHURN] ========== Commit Churn Analysis ==========");
    $display("[TC-CHURN] total_commits=%0d into_invalid=%0d replace_valid=%0d",
             tc_recorded_traces, cmt_into_invalid, cmt_replace_valid);
    $display("[TC-CHURN] replace_same_sig=%0d (exact re-recording / waste)",
             cmt_replace_same_sig);
    $display("[TC-CHURN] replace_same_pc_diff_path=%0d (path diversity or ping-pong)",
             cmt_replace_same_pc_diff_path);
    $display("[TC-CHURN] replace_diff_pc=%0d (capacity/alias eviction)",
             cmt_replace_diff_pc);
    $display("[TC-CHURN] other_way_dup_sig=%0d (both ways hold same trace = BUG)",
             cmt_other_way_same_sig);
    $display("[TC-CHURN] other_way_same_pc=%0d (both ways hold same PC diff path = diversity)",
             cmt_other_way_same_pc);
    $display("[TC-CHURN] len_hist: len2=%0d len3=%0d len4=%0d",
             cmt_len_hist[2], cmt_len_hist[3], cmt_len_hist[4]);
    $display("[TC-MULTITAKEN] commit_taken_hist: tk0=%0d tk1=%0d tk2=%0d tk3p=%0d",
             cmt_taken_hist[0], cmt_taken_hist[1], cmt_taken_hist[2],
             cmt_taken_hist[MAX_TAKEN]);
    $display("[TC-CHURN] ==========================================");

    // V76 PC-mismatch analysis summary
    $display("[TC-PCMISS] ========== PC Mismatch Analysis ==========");
    $display("[TC-PCMISS] total=%0d  1_valid_way=%0d  2_valid_ways=%0d",
             pcm_event_count, pcm_1valid, pcm_2valid);
    $display("[TC-PCMISS] unique_sets=%0d unique_lookup_pcs=%0d unique_stored_pcs=%0d",
             pcm_set_hist.num(), pcm_lookup_pc_hist.num(), pcm_stored_pc_hist.num());
    $display("[TC-PCMISS] --- Top hot sets ---");
    pcm_print_top_n("hot_set", pcm_set_hist, PC_MISS_TOP_N);
    $display("[TC-PCMISS] --- Top lookup PCs ---");
    pcm_print_top_n("lookup_pc", pcm_lookup_pc_hist, PC_MISS_TOP_N);
    $display("[TC-PCMISS] --- Top stored base_PCs ---");
    pcm_print_top_n("stored_pc", pcm_stored_pc_hist, PC_MISS_TOP_N);
    $display("[TC-PCMISS] ==========================================");
  end
`endif

endmodule
