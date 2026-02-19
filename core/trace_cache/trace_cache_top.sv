`timescale 1ns/1ps
import trace_cache_pkg::*;

// the lookup path (read side) is currently disabled in frontend.sv
module trace_cache_top (
  input  logic clk_i,
  input  logic rst_ni,

  // Raw instruction window from the frontend 
  input  logic [SLOTS_PER_CYCLE-1:0]               instr_valid_i,
  input  logic [SLOTS_PER_CYCLE-1:0][31:0]         instr_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] pc_i,
  input  logic [SLOTS_PER_CYCLE-1:0]               is_branch_i,
  input  logic [SLOTS_PER_CYCLE-1:0]               branch_taken_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] branch_target_i,

  input  logic flush_i,             // pipeline flush
  input  logic instr_queue_ready_i, // only record when the instr queue is ready

  // Current branch predictions from BHT 
  input  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_i,

  // Lookup interface provide a PC, get back hit/miss + trace
  input  logic                   lookup_valid_i,
  input  logic [PC_WIDTH-1:0]    lookup_pc_i,

  // Lookup results
  output logic                              trace_hit_o,
  output logic [TRACE_LEN-1:0][31:0]        trace_instructions_o,
  output logic [4:0]                        trace_length_o,   // number of instructions in trace
  output logic [PC_WIDTH-1:0]               trace_next_pc_o   // where to fetch after trace
);

  // ---------------------------------------------------------
  // signals into a clean interface for trace_builder
  // ---------------------------------------------------------
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

  // ---------------------------------------------------------
  // GHR 
  // ---------------------------------------------------------
  logic [GHR_WIDTH-1:0] ghr;

  tc_ghr i_tc_ghr (
    .clk_i,
    .rst_ni,
    .flush_i,
    .branch_valid_i (|(instr_valid_i & is_branch_i)),
    .branch_taken_i (|(instr_valid_i & is_branch_i & branch_taken_i)),
    .ghr_o          (ghr)
  );

  // ---------------------------------------------------------
  // SRAM signals
  // ---------------------------------------------------------
  logic                   mem_req;
  logic                   mem_we;
  logic [TRACE_ADDRW-1:0] mem_addr;
  logic [TRACE_WIDTH-1:0] mem_wdata;
  logic [BE_WIDTH-1:0]    mem_be;
  logic [TRACE_WIDTH-1:0] mem_rdata;

  // ---------------------------------------------------------
  // trace_builder 
  // ---------------------------------------------------------
  trace_builder i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i         (instr_if),
    .ghr_i           (ghr),
    .flush_i         (flush_i),

    .trace_valid_o   (),   // not used at top level for now
    .trace_data_o    (),   // not used at top level for now

    .mem_req_o       (mem_req),
    .mem_we_o        (mem_we),
    .mem_addr_o      (mem_addr),
    .mem_wdata_o     (mem_wdata),
    .mem_be_o        (mem_be)
  );

  // ---------------------------------------------------------
  // SRAM 
  // Latency = 1 cycle
  // ---------------------------------------------------------
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

  // ---------------------------------------------------------
  // Lookup / read path
  // After 1 cycle latency the SRAM returns the entry, then we
  // check base_pc and branch_flags to confirm it's a real hit.
  // ---------------------------------------------------------

  // Cast raw SRAM output to our struct 
  trace_data_t trace_read;
  assign trace_read = mem_rdata;

  // base_pc match
  logic pc_match;
  assign pc_match = (trace_read.base_pc == lookup_pc_i);

  // branch flags match
  logic branch_flags_match;
  always_comb begin
    branch_flags_match = 1'b1;
    for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
      if (i < int'(trace_read.num_branches)) begin
        if (branch_predictions_i[i] != trace_read.branch_flags[i])
          branch_flags_match = 1'b0;
      end
    end
  end

  // Hit = entry is valid + PC matches + branch history matches
  logic trace_hit;
  assign trace_hit = trace_read.valid && pc_match && branch_flags_match && lookup_valid_i;

  assign trace_hit_o     = trace_hit;
  assign trace_next_pc_o = trace_read.target_addr;

  // ---------------------------------------------------------
  // Instruction count 
  // ---------------------------------------------------------
  logic [4:0] instr_count;
  always_comb begin
    instr_count = '0;
    for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
      if (trace_read.valid_chunks[i])
        instr_count = instr_count + 1;
    end
  end
  assign trace_length_o = trace_read.valid ? instr_count : 5'b0;

  // ---------------------------------------------------------
  // Instruction reconstruction: unpack 16-bit chunks back into
  // 32-bit instructions for the frontend to use on a hit.
  // ---------------------------------------------------------
  always_comb begin
    int instr_idx;
    int chunk_idx;
    trace_instructions_o = '0;
    instr_idx = 0;
    chunk_idx = 0;
    while (chunk_idx < CHUNKS_PER_TRACE && instr_idx < TRACE_LEN) begin
      if (trace_read.valid_chunks[chunk_idx]) begin
        // Check if next chunk is the upper half of a 32-bit instruction
        if (chunk_idx + 1 < CHUNKS_PER_TRACE && !trace_read.valid_chunks[chunk_idx + 1]) begin
          // 32-bit instruction: combine lower and upper halves
          trace_instructions_o[instr_idx] = {trace_read.chunks[chunk_idx + 1],
                                              trace_read.chunks[chunk_idx]};
          chunk_idx += 2;
        end else begin
          // Compressed 16-bit instruction: zero-extend to 32 bits
          trace_instructions_o[instr_idx] = {16'b0, trace_read.chunks[chunk_idx]};
          chunk_idx += 1;
        end
        instr_idx++;
      end else begin
        // This chunk is the upper half of a previous 32-bit inst, skip it
        chunk_idx++;
      end
    end
  end

endmodule