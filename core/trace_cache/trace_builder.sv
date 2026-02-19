`timescale 1ns/1ps
import trace_cache_pkg::*;

//
// The FSM has three states:
//   IDLE   ? watching the fetch stream, waiting for a taken branch
//   ACCUM  ? actively building a trace across fetch windows
//   COMMIT ? writing the completed trace to SRAM
module trace_builder (
    input  logic clk_i,
    input  logic rst_ni,

    tracebuilder_instr_if.consumer instr_i,

    input  logic [GHR_WIDTH-1:0] ghr_i,   // global history register snapshot
    input  logic                 flush_i,  

    output logic                   trace_valid_o,  // high for one cycle when a trace is ready
    output logic [TRACE_WIDTH-1:0] trace_data_o,   // the completed trace data

    // SRAM write interface
    output logic                   mem_req_o,
    output logic                   mem_we_o,
    output logic [TRACE_ADDRW-1:0] mem_addr_o,
    output logic [TRACE_WIDTH-1:0] mem_wdata_o,
    output logic [BE_WIDTH-1:0]    mem_be_o
);

  // How many bits we need to count chunks and branches
  localparam int unsigned CHUNK_PTR_W = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned BR_CNT_W    = BR_CNT_WIDTH;

  // FSM state encoding
  typedef enum logic [1:0] {
    IDLE,    // waiting for a taken branch to start a trace
    ACCUM,   // accumulating instructions into the trace
    COMMIT   // writing the trace to SRAM
  } state_t;

  state_t state_q, state_d;

  // The trace we're currently building
  trace_data_t trace_q, trace_d;

  // How many 16-bit chunks we've filled so far
  logic [CHUNK_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;

  // How many branches (taken + not-taken) we've seen in this trace
  logic [BR_CNT_W-1:0] br_cnt_q, br_cnt_d;

  // this becomes target_addr at commit
  logic [PC_WIDTH-1:0] last_branch_target_q, last_branch_target_d;

  // derived from base_pc (direct mapped)
  logic [TRACE_ADDRW-1:0] sram_wr_addr_q, sram_wr_addr_d;

  // One-cycle pulse to trigger the SRAM write
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q,  commit_data_d;

  // GHR value at the start of the trace 
  logic [GHR_WIDTH-1:0] trace_start_ghr_q, trace_start_ghr_d;

  // We're always ready to receive instructions
  assign instr_i.ready = 1'b1;

  // driven from the commit registers
  assign mem_req_o   = commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = sram_wr_addr_q;
  assign mem_wdata_o = commit_data_q;
  assign mem_be_o    = {BE_WIDTH{1'b1}};

  // Expose the last committed trace for observation
  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

  // ---------------------------------------------------------
  // Sequential block 
  // ---------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q              <= IDLE;
      trace_q              <= '0;
      chunk_ptr_q          <= '0;
      br_cnt_q             <= '0;
      last_branch_target_q <= '0;
      sram_wr_addr_q       <= '0;
      commit_valid_q       <= 1'b0;
      commit_data_q        <= '0;
      trace_start_ghr_q    <= '0;
    end else begin
      state_q              <= state_d;
      trace_q              <= trace_d;
      chunk_ptr_q          <= chunk_ptr_d;
      br_cnt_q             <= br_cnt_d;
      last_branch_target_q <= last_branch_target_d;
      sram_wr_addr_q       <= sram_wr_addr_d;
      commit_valid_q       <= commit_valid_d;
      commit_data_q        <= commit_data_d;
      trace_start_ghr_q    <= trace_start_ghr_d;
    end
  end

  // ---------------------------------------------------------
  // Combinational block 
  // ---------------------------------------------------------
  always_comb begin
    // Local working variables
    logic [CHUNK_PTR_W-1:0] temp_chunk_ptr;
    logic [BR_CNT_W-1:0]    temp_br_cnt;
    logic                   is_compressed;
    logic                   has_space;
    logic                   found_taken;
    logic                   branch_is_last;
    logic [CHUNK_PTR_W-1:0] branch_slot;
    logic                   hit_taken_in_accum;

    // Default
    state_d              = state_q;
    trace_d              = trace_q;
    chunk_ptr_d          = chunk_ptr_q;
    br_cnt_d             = br_cnt_q;
    last_branch_target_d = last_branch_target_q;
    sram_wr_addr_d       = sram_wr_addr_q;
    commit_valid_d       = 1'b0;
    commit_data_d        = commit_data_q;
    trace_start_ghr_d    = trace_start_ghr_q;

    // Initialize locals
    temp_chunk_ptr    = '0;
    temp_br_cnt       = '0;
    is_compressed     = 1'b0;
    has_space         = 1'b0;
    found_taken       = 1'b0;
    branch_is_last    = 1'b0;
    branch_slot       = '0;
    hit_taken_in_accum = 1'b0;

    // Flush overrides everything 
    if (flush_i) begin
      state_d     = IDLE;
      chunk_ptr_d = '0;
      br_cnt_d    = '0;
      trace_d     = '0;

    end else begin

      case (state_q)

        // -------------------------------------------------
        // IDLE: watching the fetch stream.
        // -------------------------------------------------
        IDLE: begin

          // Scan the window for the first taken branch
          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i] && !found_taken) begin
              found_taken = 1'b1;
              branch_slot = CHUNK_PTR_W'(i);

              // Check if this taken branch is the last valid slot
              branch_is_last = 1'b1;
              for (int j = i + 1; j < SLOTS_PER_CYCLE; j++) begin
                if (instr_i.valid[j]) branch_is_last = 1'b0;
              end
            end
          end

          // Only start a trace if we found a useful taken branch
          if (found_taken && !branch_is_last) begin

            // Snapshot the GHR at trace start for future reference
            trace_start_ghr_d = ghr_i;
            temp_chunk_ptr    = '0;
            temp_br_cnt       = '0;
            chunk_ptr_d       = '0;
            br_cnt_d          = '0;
            trace_d           = '0;

            // Record instructions from slot 0 up to and including the taken branch.
            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              if (instr_i.valid[i] && (CHUNK_PTR_W'(i) <= branch_slot)) begin
                is_compressed = (instr_i.inst[i][1:0] != 2'b11);

                // First instruction in the window is always the base PC
                if (i == 0)
                  trace_d.base_pc = instr_i.pc[i];

                // Pack the instruction into 16-bit chunks.
                // 32-bit: two chunks, valid_chunks[n]=1, valid_chunks[n+1]=0
                // 16-bit compressed: one chunk, valid_chunks[n]=1
                if (!is_compressed) begin
                  trace_d.chunks[temp_chunk_ptr]         = instr_i.inst[i][15:0];
                  trace_d.chunks[temp_chunk_ptr+1]       = instr_i.inst[i][31:16];
                  trace_d.valid_chunks[temp_chunk_ptr]   = 1'b1;
                  trace_d.valid_chunks[temp_chunk_ptr+1] = 1'b0;
                  temp_chunk_ptr = temp_chunk_ptr + 2;
                end else begin
                  trace_d.chunks[temp_chunk_ptr]       = instr_i.inst[i][15:0];
                  trace_d.valid_chunks[temp_chunk_ptr] = 1'b1;
                  temp_chunk_ptr = temp_chunk_ptr + 1;
                end

                // Record every branch (taken or not) 
                if (instr_i.is_branch[i]) begin
                  if (temp_br_cnt < CHUNKS_PER_TRACE) begin
                    trace_d.branch_flags[temp_br_cnt] = instr_i.taken[i];
                    if (instr_i.taken[i])
                      last_branch_target_d = instr_i.target[i];
                  end
                  temp_br_cnt = temp_br_cnt + 1;
                end

              end
            end

            chunk_ptr_d = temp_chunk_ptr;
            br_cnt_d    = temp_br_cnt;

            // Compute the SRAM write address from base_pc 
            // We use bits [TRACE_ADDRW+1:2] to skip the 4-byte alignment bits.
            sram_wr_addr_d = instr_i.pc[0][TRACE_ADDRW+1:2];

            state_d = ACCUM;
          end
        end

        // -------------------------------------------------
        // ACCUM
        // -------------------------------------------------
        ACCUM: begin
          temp_chunk_ptr     = chunk_ptr_q;
          temp_br_cnt        = br_cnt_q;
          hit_taken_in_accum = 1'b0;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            is_compressed = (instr_i.inst[i][1:0] != 2'b11);

            // Check if there's room for this instruction
            has_space = is_compressed ?
                        (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                        (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

            if (instr_i.valid[i] && has_space && !hit_taken_in_accum) begin

              // Store the instruction as chunks
              if (!is_compressed) begin
                trace_d.chunks[temp_chunk_ptr]         = instr_i.inst[i][15:0];
                trace_d.chunks[temp_chunk_ptr+1]       = instr_i.inst[i][31:16];
                trace_d.valid_chunks[temp_chunk_ptr]   = 1'b1;
                trace_d.valid_chunks[temp_chunk_ptr+1] = 1'b0;
                temp_chunk_ptr = temp_chunk_ptr + 2;
              end else begin
                trace_d.chunks[temp_chunk_ptr]       = instr_i.inst[i][15:0];
                trace_d.valid_chunks[temp_chunk_ptr] = 1'b1;
                temp_chunk_ptr = temp_chunk_ptr + 1;
              end

              // Record branch outcome for tag matching
              if (instr_i.is_branch[i]) begin
                if (temp_br_cnt < CHUNKS_PER_TRACE) begin
                  trace_d.branch_flags[temp_br_cnt] = instr_i.taken[i];
                  if (instr_i.taken[i])
                    last_branch_target_d = instr_i.target[i];
                end
                temp_br_cnt = temp_br_cnt + 1;

                // If this branch is taken, stop consuming this window here.
                if (instr_i.taken[i])
                  hit_taken_in_accum = 1'b1;
              end
            end
          end

          chunk_ptr_d = temp_chunk_ptr;
          br_cnt_d    = temp_br_cnt;

          // If we hit a taken branch and still have space stay in ACCUM 
          // Otherwise commit whatever we have
          if (hit_taken_in_accum && (temp_chunk_ptr < CHUNKS_PER_TRACE))
            state_d = ACCUM;
          else
            state_d = COMMIT;
        end

        // -------------------------------------------------
        // COMMIT
        // -------------------------------------------------
        COMMIT: begin
          trace_d.valid        = 1'b1;
          trace_d.target_addr  = last_branch_target_q;
          trace_d.num_branches = BR_CNT_W'(br_cnt_q);

          commit_valid_d = 1'b1;
          commit_data_d  = trace_d;

          // Reset everything for the next trace
          state_d     = IDLE;
          chunk_ptr_d = '0;
          br_cnt_d    = '0;
          trace_d     = '0;
        end

      endcase
    end
  end

  // ---------------------------------------------------------
  // Fires one cycle after COMMIT when commit_valid_q is high.
  // We read from commit_data_q
  // ---------------------------------------------------------
`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (commit_valid_q) begin
      trace_data_t dbg;
      dbg = trace_data_t'(commit_data_q);
      $display("[TC-BUILDER] ---- TRACE COMMITTED ----");
      $display("[TC-BUILDER]   SRAM addr  = %0d", sram_wr_addr_q);
      $display("[TC-BUILDER]   base_pc    = 0x%h", dbg.base_pc);
      $display("[TC-BUILDER]   target     = 0x%h", dbg.target_addr);
      $display("[TC-BUILDER]   #branches  = %0d", dbg.num_branches);
      $display("[TC-BUILDER]   br_flags   = %b", dbg.branch_flags);
      $display("[TC-BUILDER]   chunks used= %0d", chunk_ptr_q);
      for (int i = 0; i < CHUNKS_PER_TRACE; i++) begin
        if (dbg.valid_chunks[i])
          $display("[TC-BUILDER]   chunk[%0d]  = 0x%h (instr start)", i, dbg.chunks[i]);
        else if (i > 0 && dbg.valid_chunks[i-1])
          $display("[TC-BUILDER]   chunk[%0d]  = 0x%h (instr high half)", i, dbg.chunks[i]);
      end
      $display("[TC-BUILDER] ---------------------------");
    end
  end
`endif

endmodule