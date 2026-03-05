`timescale 1ns/1ps
import trace_cache_pkg::*;

// trace_cache_top.sv
//
// N-way set-associative trace cache (parametric via NUM_WAYS in pkg).
// Each way has its own SRAM; all ways are read in parallel on lookup.
// LRU replacement selects the eviction way on writes.

module trace_cache_top (
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

  // Lookup interface
  input  logic                lookup_valid_i,
  input  logic [PC_WIDTH-1:0] lookup_pc_i,

  // Lookup results
  output logic                                        trace_hit_o,
  output logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]      trace_instructions_o,
  output logic [TRACE_LEN_WIDTH-1:0]                 trace_length_o,
  output logic [PC_WIDTH-1:0]                        trace_next_pc_o
);

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
  assign instr_if.consumed          = instr_queue_consumed_i & {SLOTS_PER_CYCLE{~flush_i}};
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

  logic                   mem_req_builder;
  logic                   mem_we_builder;
  logic [TRACE_ADDRW-1:0] mem_addr_builder;
  logic [TRACE_WIDTH-1:0] mem_wdata_builder;
  logic [BE_WIDTH-1:0]    mem_be_builder;

  trace_builder i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i              (instr_if),
    .ghr_i                (ghr),
    .flush_i              (flush_i),
    .branch_predictions_i (branch_predictions_i),
    .trace_valid_o        (),
    .trace_data_o         (),
    .mem_req_o            (mem_req_builder),
    .mem_we_o             (mem_we_builder),
    .mem_addr_o           (mem_addr_builder),
    .mem_wdata_o          (mem_wdata_builder),
    .mem_be_o             (mem_be_builder)
  );

  logic                        lookup_fire;
  logic                        lookup_valid_q;
  logic [PC_WIDTH-1:0]         lookup_pc_q;
  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_q;
  logic [TRACE_ADDRW-1:0]      lookup_set_q;

  assign lookup_fire = lookup_valid_i && !mem_req_builder;

  logic [PC_WIDTH-1:0] lookup_base;
  assign lookup_base = lookup_pc_i & {{(PC_WIDTH-4){1'b1}}, 4'b0000};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lookup_valid_q       <= 1'b0;
      lookup_pc_q          <= '0;
      branch_predictions_q <= '0;
      lookup_set_q         <= '0;
    end else begin
      lookup_valid_q       <= lookup_fire;
      lookup_pc_q          <= lookup_base;
      branch_predictions_q <= branch_predictions_i;
      lookup_set_q         <= tc_index(lookup_base, branch_predictions_i);
    end
  end

  logic lru [(1 << TRACE_ADDRW)];
  logic wr_way;
  assign wr_way = ~lru[mem_addr_builder];

  always_comb begin
    for (int w = 0; w < NUM_WAYS; w++) begin
      mem_req[w]   = 1'b0;
      mem_we[w]    = 1'b0;
      mem_addr[w]  = '0;
      mem_wdata[w] = '0;
      mem_be[w]    = {BE_WIDTH{1'b1}};
    end

    if (mem_req_builder) begin
      mem_req[wr_way]   = 1'b1;
      mem_we[wr_way]    = 1'b1;
      mem_addr[wr_way]  = mem_addr_builder;
      mem_wdata[wr_way] = mem_wdata_builder;
      mem_be[wr_way]    = mem_be_builder;
    end else if (lookup_fire) begin
      for (int w = 0; w < NUM_WAYS; w++) begin
        mem_req[w]  = 1'b1;
        mem_we[w]   = 1'b0;
        mem_addr[w] = tc_index(lookup_base, branch_predictions_i);
      end
    end
  end

  for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_ways
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
  logic [NUM_WAYS-1:0] pc_match;
  logic [NUM_WAYS-1:0] branch_flags_match;
  logic [NUM_WAYS-1:0] way_hit;

  for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_tag_cmp
    assign trace_read[w] = mem_rdata[w];
    assign pc_match[w]   = (trace_read[w].base_pc == lookup_pc_q);

    always_comb begin
      branch_flags_match[w] = 1'b1;
      for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
        if (i < int'(trace_read[w].num_branches)) begin
          if (branch_predictions_q[i] != trace_read[w].branch_flags[i])
            branch_flags_match[w] = 1'b0;
        end
      end
    end

    assign way_hit[w] = trace_read[w].valid
                     && pc_match[w]
                     && branch_flags_match[w]
                     && lookup_valid_q;
  end

  logic trace_hit;
  assign trace_hit   = |way_hit;
  assign trace_hit_o = trace_hit;

  logic [$clog2(NUM_WAYS)-1:0] hit_way_idx;
  always_comb begin
    hit_way_idx = '0;
    for (int w = NUM_WAYS-1; w >= 0; w--) begin
      if (way_hit[w]) hit_way_idx = w[$clog2(NUM_WAYS)-1:0];
    end
  end

  trace_data_t hit_trace;
  assign hit_trace       = trace_read[hit_way_idx];
  assign trace_next_pc_o = hit_trace.target_addr;

  logic [TRACE_LEN_WIDTH-1:0] instr_count;
  always_comb begin
    instr_count = '0;
    for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
      if (hit_trace.valid_chunks[i])
        instr_count = instr_count + 1;
    end
  end
  assign trace_length_o = trace_hit ? instr_count : '0;

  always_comb begin
    int instr_idx;
    int chunk_idx;
    trace_instructions_o = '0;
    instr_idx = 0;
    chunk_idx = 0;
    while (chunk_idx < CHUNKS_PER_TRACE && instr_idx < TRACE_LEN) begin
      if (hit_trace.valid_chunks[chunk_idx]) begin
        if (chunk_idx + 1 < CHUNKS_PER_TRACE && !hit_trace.valid_chunks[chunk_idx + 1]) begin
          trace_instructions_o[instr_idx] = {hit_trace.chunks[chunk_idx + 1],
                                             hit_trace.chunks[chunk_idx]};
          chunk_idx += 2;
        end else begin
          trace_instructions_o[instr_idx] = {16'b0, hit_trace.chunks[chunk_idx]};
          chunk_idx += 1;
        end
        instr_idx++;
      end else begin
        chunk_idx++;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int s = 0; s < (1 << TRACE_ADDRW); s++)
        lru[s] <= 1'b0;
    end else begin
      if (trace_hit)
        lru[lookup_set_q] <= hit_way_idx[0];
      if (mem_req_builder)
        lru[mem_addr_builder] <= wr_way;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (lookup_valid_q) begin
      logic any_valid;
      any_valid = 1'b0;
      for (int w = 0; w < NUM_WAYS; w++)
        if (trace_read[w].valid) any_valid = 1'b1;

      if (trace_hit) begin
        $display("[TC-LOOKUP] HIT at 0x%h (way %0d)", lookup_pc_q, hit_way_idx);
      end else if (any_valid) begin
        for (int w = 0; w < NUM_WAYS; w++) begin
          if (trace_read[w].valid && !pc_match[w])
            $display("[TC-LOOKUP] PC MISS: stored=0x%h lookup=0x%h (way %0d)",
                     trace_read[w].base_pc, lookup_pc_q, w);
          else if (trace_read[w].valid && pc_match[w] && !branch_flags_match[w])
            $display("[TC-LOOKUP] BR MISS at 0x%h (way %0d): stored=%b lookup=%b num=%0d",
                     lookup_pc_q, w, trace_read[w].branch_flags,
                     branch_predictions_q, trace_read[w].num_branches);
        end
      end
    end
  end

  int unsigned tc_valid_lookups;
  int unsigned tc_useful_hits;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tc_valid_lookups <= 0;
      tc_useful_hits   <= 0;
    end else if (lookup_valid_q) begin
      logic any_valid;
      any_valid = 1'b0;
      for (int w = 0; w < NUM_WAYS; w++)
        if (trace_read[w].valid) any_valid = 1'b1;
      if (any_valid) begin
        tc_valid_lookups <= tc_valid_lookups + 1;
        if (trace_hit)
          tc_useful_hits <= tc_useful_hits + 1;
      end
    end
  end

  final begin
    $display("[TC-USEFUL] valid_lookups=%0d hits=%0d rate=%0d%%",
             tc_valid_lookups, tc_useful_hits,
             tc_valid_lookups > 0 ? (tc_useful_hits * 100) / tc_valid_lookups : 0);
  end
`endif
endmodule
