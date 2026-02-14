`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_cache_top (
  input  logic clk_i,
  input  logic rst_ni,

  // Multi-slot instruction input from frontend
  input  logic [SLOTS_PER_CYCLE-1:0]             instr_valid_i,
  input  logic [SLOTS_PER_CYCLE-1:0][31:0]       instr_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] pc_i,
  input  logic [SLOTS_PER_CYCLE-1:0]             is_branch_i,
  input  logic [SLOTS_PER_CYCLE-1:0]             branch_taken_i,
  input  logic [SLOTS_PER_CYCLE-1:0][PC_WIDTH-1:0] branch_target_i,

  input  logic flush_i,
  input  logic instr_queue_ready_i,
  
  // Branch predictions for lookup
  input  logic [MAX_BRANCHES-1:0]                branch_predictions_i,
  
  // Lookup interface (for active mode)
  input  logic                                    lookup_valid_i,
  input  logic [PC_WIDTH-1:0]                    lookup_pc_i,
  
  // Output interface (for active mode)
  output logic                                    trace_hit_o,
  output logic [MAX_INSTRUCTIONS-1:0][31:0]     trace_instructions_o,
  output logic [4:0]                             trace_length_o,
  output logic [PC_WIDTH-1:0]                    trace_next_pc_o
);

  // ========================================================================
  // Instruction Interface
  // ========================================================================
  
  tracebuilder_instr_if instr_if (
    .clk_i (clk_i),
    .rst_ni(rst_ni)
  );

  // Connect frontend inputs to interface
  assign instr_if.valid  = instr_valid_i & {SLOTS_PER_CYCLE{instr_queue_ready_i & ~flush_i}};
  assign instr_if.pc     = pc_i;
  assign instr_if.inst   = instr_i;
  assign instr_if.is_branch = is_branch_i;
  assign instr_if.taken  = branch_taken_i;
  assign instr_if.target = branch_target_i;

  // ========================================================================
  // Global History Register
  // ========================================================================
  
  logic [GHR_WIDTH-1:0] ghr;

  tc_ghr i_tc_ghr (
    .clk_i,
    .rst_ni,
    .flush_i,
    .branch_valid_i  (|(instr_valid_i & is_branch_i)),
    .branch_taken_i  (|(instr_valid_i & is_branch_i & branch_taken_i)),
    .ghr_o           (ghr)
  );

  // ========================================================================
  // Trace Builder
  // ========================================================================
  // The line-fill buffer logic services trace cache misses.
  // The basic blocks are latched one at a time into the line-fill buffer
  // ========================================================================
  
  logic                   mem_req_write;
  logic                   mem_we;
  logic [TRACE_ADDRW-1:0] mem_addr_write;
  logic [TRACE_WIDTH-1:0] mem_wdata;
  logic [BE_WIDTH-1:0]    mem_be;

  trace_builder i_trace_builder (
    .clk_i,
    .rst_ni,
    .instr_i         (instr_if),
    .ghr_i           (ghr),
    .flush_i         (flush_i),

    .trace_valid_o   (),
    .trace_data_o    (),

    .mem_req_o       (mem_req_write),
    .mem_we_o        (mem_we),
    .mem_addr_o      (mem_addr_write),
    .mem_wdata_o     (mem_wdata),
    .mem_be_o        (mem_be),

    .tag_valid_o     (),  
    .tag_start_pc_o  (),
    .tag_start_ghr_o (),
    .tag_trace_len_o (),
    .tag_sram_addr_o ()
  );

  // ========================================================================
  // Trace Cache SRAM - Single SRAM Architecture
  // ========================================================================
  //
  // Single SRAM stores complete trace_data_t including:
  //   - valid bit
  //   - tag (base_pc)
  //   - branch_flags (for matching predictions)
  //   - num_branches, fall_through_addr, target_addr
  //   - all instructions
  //
  // ========================================================================
  
  logic                   mem_req_read;
  logic [TRACE_ADDRW-1:0] mem_addr_read;
  logic [TRACE_WIDTH-1:0] mem_rdata;
  
  // Write has priority over read
  logic                   sram_req;
  logic                   sram_we;
  logic [TRACE_ADDRW-1:0] sram_addr;
  
  assign sram_req  = mem_req_write | mem_req_read;
  assign sram_we   = mem_req_write;  // Write has priority
  assign sram_addr = mem_req_write ? mem_addr_write : mem_addr_read;
  
  tc_sram #(
    .NumWords  (1 << TRACE_ADDRW),  
    .DataWidth (TRACE_WIDTH),       // Complete trace data
    .NumPorts  (1),                 // Single port
    .Latency   (1)                  // 1 cycle read latency
  ) i_trace_sram (
    .clk_i,
    .rst_ni,
    .req_i   ({sram_req}),
    .we_i    ({sram_we}),
    .addr_i  ({sram_addr}),
    .wdata_i ({mem_wdata}),
    .be_i    ({mem_be}),
    .rdata_o ({mem_rdata})
  );

  // ========================================================================
  // Trace Cache Lookup Logic
  // ========================================================================
  // The fetch address is used together with the multiple branch
  // predictions to determine if the trace read from the trace cache matches
  // the predicted sequence of basic blocks. Specifically, a trace cache hit
  // requires that (1) the fetch address match the tag and (2) the branch
  // predictions match the branch flags.
  // ========================================================================
  
  // Lookup signals
  logic [PC_WIDTH-1:0]    lookup_pc;
  logic                   lookup_valid;
  
  // Connect lookup inputs
  assign lookup_pc = lookup_pc_i;
  assign lookup_valid = lookup_valid_i;
  
  // Unpack read data into trace structure
  trace_data_t trace_read;
  assign trace_read = mem_rdata;
  
  // Generate lookup address (simple direct-mapped for now)
  // Could be enhanced with path associativity: pc XOR branch_outcomes
  assign mem_addr_read = lookup_pc[TRACE_ADDRW+1:2];  // Word-aligned PC bits
  assign mem_req_read = lookup_valid && !mem_req_write;
  
  // Hit detection 
  logic trace_hit;
  logic pc_match;
  logic branch_flags_match;
  
  // (1) PC must match tag
  assign pc_match = (trace_read.base_pc == lookup_pc);
  
  // (2) Branch predictions must match branch_flags
    branch_flags_match = 1'b1;
    
    if (trace_read.num_branches > 1) begin
      // Compare first (num_branches - 1) predictions against branch_flags
      for (int i = 0; i < MAX_BRANCHES-1; i++) begin
        if (i < (trace_read.num_branches - 1)) begin
          if (branch_predictions_i[i] != trace_read.branch_flags[i]) begin
            branch_flags_match = 1'b0;
          end
        end
      end
    end
    // If num_branches == 0 or 1, no flags to check, so match = true
  end
  
  // Final hit signal
  assign trace_hit = trace_read.valid && pc_match && branch_flags_match;
  
  // ========================================================================
  // Output Assignments
  // ========================================================================
  
  assign trace_hit_o = trace_hit;
  assign trace_length_o = trace_read.valid ? {1'b0, trace_read.num_branches} : 5'b0;  // Temp: just output num_branches
  assign trace_next_pc_o = trace_read.target_addr;
  
  // Reconstruct full instructions from chunks
  // TODO: This needs proper reconstruction logic for compressed instructions
  // For now, just pass through first MAX_INSTRUCTIONS worth of data
  always_comb begin
    trace_instructions_o = '0;
    automatic int instr_idx = 0;
    automatic int chunk_idx = 0;
    
    while (chunk_idx < CHUNKS_PER_TRACE && instr_idx < MAX_INSTRUCTIONS) begin
      if (trace_read.valid_chunks[chunk_idx]) begin
        // Check if 32-bit or 16-bit instruction
        if (chunk_idx + 1 < CHUNKS_PER_TRACE && !trace_read.valid_chunks[chunk_idx + 1]) begin
          // 32-bit instruction (2 chunks)
          trace_instructions_o[instr_idx] = {trace_read.chunks[chunk_idx + 1], trace_read.chunks[chunk_idx]};
          chunk_idx += 2;
        end else begin
          // 16-bit compressed instruction (1 chunk)
          trace_instructions_o[instr_idx] = {16'b0, trace_read.chunks[chunk_idx]};
          chunk_idx += 1;
        end
        instr_idx++;
      end else begin
        chunk_idx++;
      end
    end
  end
  
  // ========================================================================
  // Debug Outputs
  // ========================================================================
  
  always_ff @(posedge clk_i) begin
    if (trace_hit) begin
      $display("[TC-%0t] HIT: PC=%h, num_br=%0d, next_pc=%h",
               $time, lookup_pc, trace_read.num_branches,
               trace_read.target_addr);
    end
    if (mem_req_write) begin
      $display("[TC-%0t] FILL: addr=%0d", $time, mem_addr_write);
    end
  end

endmodule