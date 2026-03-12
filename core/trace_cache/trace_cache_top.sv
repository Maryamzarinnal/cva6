`timescale 1ns/1ps
import trace_cache_pkg::*;
import riscv::*;

// Set-associative trace cache. Each way has its own SRAM; lookup reads all ways in parallel.
// LRU picks the way to replace on write. Lookup runs when the frontend consumes a window;
// SRAM has one cycle latency so the hit result is valid the next cycle. Builder and lookup
// share the port (builder write blocks lookup that cycle).
// MaxTraceInstr = max instructions per trace (e.g. one fetch window); must be <= TRACE_LEN.

module trace_cache_top #(
  parameter int unsigned MaxTraceInstr = TRACE_LEN
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

  // Resolved branch (from backend): correct-path outcomes to store as path tag, matching Rotenberg fill-at-retire semantics
  input  logic                resolved_branch_valid_i,
  input  logic [PC_WIDTH-1:0]  resolved_branch_pc_i,
  input  logic                resolved_branch_is_taken_i,
  input  logic                resolved_branch_is_mispredict_i,

  // Lookup interface
  input  logic                lookup_valid_i,
  input  logic [PC_WIDTH-1:0] lookup_pc_i,

  // Lookup results
  output logic                                        trace_hit_o,
  output logic [TRACE_LEN-1:0][INSTR_WIDTH-1:0]      trace_instructions_o,
  output logic [TRACE_LEN_WIDTH-1:0]                 trace_length_o,
  output logic [PC_WIDTH-1:0]                        trace_next_pc_o,
  output logic [CHUNKS_PER_TRACE-1:0][15:0]          trace_chunks_o,
  output logic [CHUNKS_PER_TRACE-1:0]                trace_valid_chunks_o,
  output logic [TRACE_LEN-1:0][PC_WIDTH-1:0]         trace_pcs_o
);

  // Stored trace length must not exceed structure size
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
  logic [CHUNKS_PER_TRACE-1:0][PC_WIDTH-1:0] mem_branch_pcs_builder;

  // Resolved-outcome table: on correct-path resolve store (pc_hi, taken); on write override branch_flags => stored path = resolved (Rotenberg)
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
      resolved_table_q[resolved_wr_idx].pc_hi  <= resolved_branch_pc_i[PC_WIDTH-1:10];
      resolved_table_q[resolved_wr_idx].taken <= resolved_branch_is_taken_i;
    end
  end

  logic [TRACE_WIDTH-1:0] mem_wdata_final;
  always_comb begin
    trace_data_t w;
    logic [RESOLVED_ADDRW-1:0] idx;
    w = trace_data_t'(mem_wdata_builder);
    for (int i = 0; i < CHUNKS_PER_TRACE; i++)
      if (i < int'(w.num_branches)) begin
        idx = mem_branch_pcs_builder[i][RESOLVED_ADDRW+3:4];
        if (resolved_table_q[idx].valid && resolved_table_q[idx].pc_hi == mem_branch_pcs_builder[i][PC_WIDTH-1:10])
          w.branch_flags[i] = resolved_table_q[idx].taken;
      end
    mem_wdata_final = w;
  end

  trace_builder #(
    .MAX_INSTR_PER_TRACE  (MaxTraceInstr)
  ) i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i              (instr_if),
    .ghr_i                (ghr),
    .flush_i              (flush_i),
    .trace_valid_o        (),
    .trace_data_o         (),
    .mem_req_o            (mem_req_builder),
    .mem_we_o             (mem_we_builder),
    .mem_addr_o           (mem_addr_builder),
    .mem_wdata_o          (mem_wdata_builder),
    .mem_be_o             (mem_be_builder),
    .mem_branch_pcs_o     (mem_branch_pcs_builder)
  );

  logic                        lookup_fire;
  logic                        lookup_valid_q;
  logic [PC_WIDTH-1:0]         lookup_pc_q;
  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_q;
  logic [TRACE_ADDRW-1:0]      lookup_set_q;

  assign lookup_fire = lookup_valid_i && !mem_req_builder;
  logic [PC_WIDTH-1:0] lookup_base;
  assign lookup_base = pc_align_16(lookup_pc_i);

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
      mem_wdata[wr_way] = mem_wdata_final;
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
  assign hit_trace            = trace_read[hit_way_idx];
  assign trace_next_pc_o      = hit_trace.target_addr;
  assign trace_chunks_o       = trace_hit ? hit_trace.chunks : '0;
  assign trace_valid_chunks_o = trace_hit ? hit_trace.valid_chunks : '0;

  logic [TRACE_LEN_WIDTH-1:0] instr_count;
  always_comb begin
    instr_count = '0;
    for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
      if (hit_trace.valid_chunks[i] && instr_count < TRACE_LEN_WIDTH'(TRACE_LEN))
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

  // Reconstruct PCs from base_pc, expanded instructions, and branch_flags. Fall-through +2/+4;
  // taken branches use pc+imm (B/J/RVC); JALR at end uses stored target_addr. Uses riscv:: opcodes.
  logic [TRACE_LEN-1:0][PC_WIDTH-1:0] computed_pcs;
  always_comb begin
    logic [PC_WIDTH-1:0]     pc;
    logic [INSTR_WIDTH-1:0]  instr;
    logic [PC_WIDTH-1:0]     imm_signed;
    logic                    is_rvc, is_cf, is_jalr, taken;
    int                      br_idx;
    for (int j = 0; j < TRACE_LEN; j++) trace_pcs_o[j] = '0;
    if (!trace_hit) begin
      pc = '0;
      br_idx = 0;
    end else begin
    pc    = hit_trace.base_pc;
    br_idx = 0;
    for (int i = 0; i < TRACE_LEN; i++) begin
      if (i >= int'(instr_count)) break;
      instr   = trace_instructions_o[i];
      is_rvc  = (instr[1:0] != 2'b11);
      is_cf   = (!is_rvc && (instr[6:0] == OpcodeBranch || instr[6:0] == OpcodeJal || instr[6:0] == OpcodeJalr))
                || (is_rvc && (instr[15:13] == OpcodeC1J || instr[15:13] == OpcodeC1Beqz || instr[15:13] == OpcodeC1Bnez));
      is_jalr = (!is_rvc && (instr[6:0] == OpcodeJalr))
                || (is_rvc && instr[15:13] == OpcodeC2JalrMvAdd && instr[6:2] == 5'b00000 && instr[1:0] == OpcodeC2 && instr[12]);
      taken   = is_cf && (br_idx < int'(hit_trace.num_branches)) && hit_trace.branch_flags[br_idx];
      if (is_cf) br_idx++;
      computed_pcs[i] = pc;
      if (taken && is_jalr && (i == int'(instr_count) - 1))
        pc = hit_trace.target_addr;
      else if (taken && is_cf) begin
        if (!is_rvc && instr[6:0] == OpcodeBranch)   // B-type (sb_imm)
          imm_signed = {{(PC_WIDTH-13){instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
        else if (!is_rvc && instr[6:0] == OpcodeJal) // JAL (uj_imm)
          imm_signed = {{(PC_WIDTH-21){instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
        else if (is_rvc && instr[15:13] == OpcodeC1J) // C.jal (two encodings: instr[14] selects format, same as instr_scan rvc_imm_o)
          imm_signed = instr[14] ? {{(PC_WIDTH-9){instr[12]}}, instr[6:5], instr[2], instr[11:10], instr[4:3], 1'b0}
                        : {{(PC_WIDTH-12){instr[12]}}, instr[8], instr[10:9], instr[6], instr[7], instr[2], instr[11], instr[5:3], 1'b0};
        else  // RVC beqz/bnez
          imm_signed = {{(PC_WIDTH-9){instr[12]}}, instr[6:5], instr[2], instr[11:10], instr[4:3], 1'b0};
        pc = pc + imm_signed;
      end else
        pc = pc + (is_rvc ? PC_WIDTH'(2) : PC_WIDTH'(4));
    end
    for (int j = 0; j < TRACE_LEN; j++)
      trace_pcs_o[j] = (j < int'(instr_count)) ? computed_pcs[j] : '0;
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
  // +define+TRACE_CACHE_DEBUG_VERBOSE for per-lookup prints
  `ifdef TRACE_CACHE_DEBUG_VERBOSE
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
            $display("[TC-LOOKUP] lookup PC 0x%h missed (way %0d had base_pc 0x%h)",
                     lookup_pc_q, w, trace_read[w].base_pc);
          else if (trace_read[w].valid && pc_match[w] && !branch_flags_match[w])
            $display("[TC-LOOKUP] BR MISS at 0x%h (way %0d): stored=%b lookup=%b num=%0d",
                     lookup_pc_q, w, trace_read[w].branch_flags,
                     branch_predictions_q, trace_read[w].num_branches);
        end
      end
    end
  end
  `endif

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
