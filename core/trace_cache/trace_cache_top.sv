`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// Set-associative trace cache. Each way has its own SRAM; lookup reads all ways in parallel.
// LRU picks the way to replace on write. Lookup runs when the frontend consumes a window;
// SRAM has one cycle latency so the hit result is valid the next cycle. Builder and lookup
// share the port (builder write blocks lookup that cycle).
// MaxTraceInstr is the current recording policy (e.g. one fetch window) and must be <= TRACE_LEN.

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
  input  logic                        instr_queue_ready_i,
  input  logic [SLOTS_PER_CYCLE-1:0]  instr_queue_consumed_i,

  input  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_i,
  input  logic [BR_CNT_WIDTH-1:0]     lookup_num_branches_i,

  // Resolved branch (from backend): correct-path outcomes to store as path tag, matching Rotenberg fill-at-retire semantics
  input  logic                         resolved_branch_valid_i,
  input  logic [PC_WIDTH-1:0]          resolved_branch_pc_i,
  input  logic                         resolved_branch_is_taken_i,
  input  logic                         resolved_branch_is_mispredict_i,

  // Lookup interface
  input  logic                         lookup_valid_i,
  input  logic [PC_WIDTH-1:0]          lookup_pc_i,

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

  logic [GHR_WIDTH-1:0] ghr;
  tc_ghr i_tc_ghr (
    .clk_i,
    .rst_ni,
    .flush_i,
    .branch_valid_i (|(instr_valid_i & is_branch_i)),
    .branch_taken_i (|(instr_valid_i & is_branch_i & branch_taken_i)),
    .ghr_o          (ghr)
  );

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

  localparam int unsigned RESOLVED_ADDRW = 6;
  localparam int unsigned RESOLVED_SIZE  = 1 << RESOLVED_ADDRW;
  typedef struct packed {
    logic                 valid;
    logic [PC_WIDTH-1:10] pc_hi;
    logic                 taken;
  } resolved_entry_t;

  resolved_entry_t resolved_table_q [RESOLVED_SIZE];
  logic [RESOLVED_ADDRW-1:0] resolved_wr_idx;
  assign resolved_wr_idx = resolved_branch_pc_i[RESOLVED_ADDRW+3:4];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < RESOLVED_SIZE; i++)
        resolved_table_q[i] <= '0;
    end else if (resolved_branch_valid_i && !resolved_branch_is_mispredict_i) begin
      resolved_table_q[resolved_wr_idx].valid <= 1'b1;
      resolved_table_q[resolved_wr_idx].pc_hi <= resolved_branch_pc_i[PC_WIDTH-1:10];
      resolved_table_q[resolved_wr_idx].taken <= resolved_branch_is_taken_i;
    end
  end

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
  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_q;
  logic [BR_CNT_WIDTH-1:0]     lookup_num_branches_q;
  logic [TRACE_ADDRW-1:0]      lookup_set_q;

  assign lookup_fire           = lookup_valid_i && !mem_req_builder;
  assign lookup_result_valid_o = lookup_valid_q;

  // IMPORTANT: exact trace-start PC, not a 16-byte aligned block base.
  logic [PC_WIDTH-1:0] lookup_base;
  assign lookup_base = lookup_pc_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lookup_valid_q        <= 1'b0;
      lookup_pc_q           <= '0;
      branch_predictions_q  <= '0;
      lookup_num_branches_q <= '0;
      lookup_set_q          <= '0;
    end else begin
      lookup_valid_q        <= lookup_fire;
      lookup_pc_q           <= lookup_base;
      branch_predictions_q  <= branch_predictions_i;
      lookup_num_branches_q <= lookup_num_branches_i;
      lookup_set_q          <= tc_index(lookup_base, branch_predictions_i);
    end
  end

  logic lru [(1 << TRACE_ADDRW)];
  logic [NUM_WAYS-1:0] used_q [(1 << TRACE_ADDRW)];
  logic wr_way;
  assign wr_way = ~lru[mem_addr_builder];

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

    if (mem_req_builder) begin
      mem_req[wr_way]   = 1'b1;
      mem_we[wr_way]    = 1'b1;
      mem_addr[wr_way]  = mem_addr_builder;
      mem_wdata[wr_way] = mem_wdata_final;
      mem_be[wr_way]    = mem_be_builder;
      tag_req[wr_way]   = 1'b1;
      tag_we[wr_way]    = 1'b1;
      tag_addr[wr_way]  = mem_addr_builder;
      tag_wdata[wr_way] = mem_tag_final;
    end else if (lookup_fire) begin
      for (int w = 0; w < NUM_WAYS; w++) begin
        mem_req[w]  = 1'b1;
        mem_we[w]   = 1'b0;
        mem_addr[w] = tc_index(lookup_base, branch_predictions_i);
        tag_req[w]  = 1'b1;
        tag_we[w]   = 1'b0;
        tag_addr[w] = tc_index(lookup_base, branch_predictions_i);
      end
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

    tc_sram #(
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
    assign way_valid[w]  = (trace_tag_read[w].valid === 1'b1);
    assign pc_match[w]   = way_valid[w] && (trace_tag_read[w].base_pc == lookup_pc_q);
    assign branch_count_match[w] = way_valid[w] &&
                                   (trace_tag_read[w].num_branches ==
                                    TRIGGER_BRANCH_CNT_WIDTH'(lookup_num_branches_q));

    trace_tag_compare i_trace_tag_compare (
      .base_pc_i      (lookup_pc_q),
      .num_branches_i (TRIGGER_BRANCH_CNT_WIDTH'(lookup_num_branches_q)),
      .branch_flags_i (branch_predictions_q[TRIGGER_BRANCH_BITS-1:0]),
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
            if (branch_predictions_q[i] != trace_tag_read[w].branch_flags[i])
              branch_flags_match[w] = 1'b0;
          end
        end
      end
    end
  end

  logic [NUM_WAYS-1:0] raw_way_hit;
  always_comb begin
    for (int w = 0; w < NUM_WAYS; w++) begin
      raw_way_hit[w] = tag_hit_raw[w] & lookup_valid_q;
      way_hit[w]     = raw_way_hit[w] & ~used_q[lookup_set_q][w];
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

  assign miss_reason_empty_o = lookup_valid_q && !trace_hit && !any_valid_in_set;
  assign miss_reason_pc_o    = lookup_valid_q && !trace_hit &&  any_valid_in_set && !any_pc_match_in_set;
  assign miss_reason_path_o  = lookup_valid_q && !trace_hit &&  any_valid_in_set &&  any_pc_match_in_set;

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
      for (int s = 0; s < (1 << TRACE_ADDRW); s++) begin
        lru[s] <= 1'b0;
        used_q[s] <= '0;
      end
    end else if (flush_i) begin
      for (int s = 0; s < (1 << TRACE_ADDRW); s++) begin
        lru[s] <= 1'b0;
        used_q[s] <= '0;
      end
    end else begin
      if (trace_hit)
        lru[lookup_set_q] <= hit_way_idx[0];
      if (mem_req_builder)
        lru[mem_addr_builder] <= wr_way;
      if (mem_req_builder && mem_we_builder)
        used_q[mem_addr_builder][wr_way] <= 1'b0;
      if (mark_used_i && trace_hit)
        used_q[lookup_set_q][hit_way_idx] <= 1'b1;
    end
  end

  assign trace_used_o = raw_trace_hit && used_q[lookup_set_q][raw_hit_way_idx];

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
    end else if (lookup_valid_q && !trace_hit) begin
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
                     branch_predictions_q[TRIGGER_BRANCH_BITS-1:0],
                     trace_tag_read[w].num_branches, lookup_num_branches_q);
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

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_valid_lookups <= 0;
      tc_useful_hits   <= 0;
      tc_recorded_traces <= 0;
      tc_lookup_requests <= 0;
      tc_lookup_blocked_by_builder <= 0;
      tc_tag_payload_mismatch <= 0;
    end else begin
      if (mem_req_builder && mem_we_builder)
        tc_recorded_traces <= tc_recorded_traces + 1;

      if (lookup_valid_i)
        tc_lookup_requests <= tc_lookup_requests + 1;
      if (lookup_valid_i && mem_req_builder)
        tc_lookup_blocked_by_builder <= tc_lookup_blocked_by_builder + 1;

      if (lookup_valid_q) begin
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
    $display("[TC-USEFUL] valid_lookups=%0d hits=%0d rate=%0d%%",
             tc_valid_lookups, tc_useful_hits, tc_useful_rate);
    $display("[TC-SPLIT-TAG] payload_mismatch=%0d (tag hit but payload slot was invalid)",
             tc_tag_payload_mismatch);
    `ifdef MODEL_TECH
    $display("[TC-MISS-BREAKDOWN] total_misses=%0d empty=%0d pc_mismatch=%0d path_mismatch=%0d (why lookups missed)",
             tc_miss_total, tc_miss_empty, tc_miss_pc, tc_miss_path);
    `endif
  end
`endif

endmodule
