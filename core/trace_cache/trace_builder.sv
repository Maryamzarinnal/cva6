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

  localparam int unsigned CHUNK_PTR_W = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned BR_CNT_W    = BR_CNT_WIDTH;

  typedef enum logic [1:0] {
    IDLE,
    ACCUM,
    COMMIT
  } state_t;

  state_t state_q, state_d;
  trace_data_t trace_q, trace_d;
  logic [CHUNK_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;
  logic [BR_CNT_W-1:0] br_cnt_q, br_cnt_d;
  logic [PC_WIDTH-1:0] last_branch_target_q, last_branch_target_d;
  logic [TRACE_ADDRW-1:0] sram_wr_addr_q, sram_wr_addr_d;
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q,  commit_data_d;
  logic [GHR_WIDTH-1:0] trace_start_ghr_q, trace_start_ghr_d;
  // snapshot of chunk_ptr taken at commit time, so the debug print is correct
  // (chunk_ptr_q itself is reset to 0 in the same COMMIT cycle)
  logic [CHUNK_PTR_W-1:0] commit_chunk_ptr_q, commit_chunk_ptr_d;

  assign instr_i.ready = 1'b1;
  assign mem_req_o   = commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = sram_wr_addr_q;
  assign mem_wdata_o = commit_data_q;
  assign mem_be_o    = {BE_WIDTH{1'b1}};
  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

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
      commit_chunk_ptr_q   <= '0;
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
      commit_chunk_ptr_q   <= commit_chunk_ptr_d;
    end
  end

  always_comb begin
    logic [CHUNK_PTR_W-1:0] temp_chunk_ptr;
    logic [BR_CNT_W-1:0]    temp_br_cnt;
    logic                   is_compressed;
    logic                   has_space;
    logic                   found_taken;
    logic                   branch_is_last;
    logic [CHUNK_PTR_W-1:0] branch_slot;
    logic                   hit_taken_in_accum;
    logic                   trace_full;

    state_d              = state_q;
    trace_d              = trace_q;
    chunk_ptr_d          = chunk_ptr_q;
    br_cnt_d             = br_cnt_q;
    last_branch_target_d = last_branch_target_q;
    sram_wr_addr_d       = sram_wr_addr_q;
    commit_valid_d       = 1'b0;
    commit_data_d        = commit_data_q;
    trace_start_ghr_d    = trace_start_ghr_q;
    commit_chunk_ptr_d   = commit_chunk_ptr_q;

    temp_chunk_ptr     = '0;
    temp_br_cnt        = '0;
    is_compressed      = 1'b0;
    has_space          = 1'b0;
    found_taken        = 1'b0;
    branch_is_last     = 1'b0;
    branch_slot        = '0;
    hit_taken_in_accum = 1'b0;
    trace_full         = 1'b0;

    if (flush_i) begin
      state_d     = IDLE;
      chunk_ptr_d = '0;
      br_cnt_d    = '0;
      trace_d     = '0;

    end else begin

      case (state_q)

        // -------------------------------------------------
        // IDLE: scan each new fetch window for a taken branch.
        // When we see one, record instructions up to and
        // including the taken branch, then move to ACCUM to
        // continue filling the trace from the next window.
        //
        // Gate on |instr_i.consumed so we only act on fresh
        // windows ? stale windows (same data held while the
        // pipeline is stalled) are ignored.
        // -------------------------------------------------
        IDLE: begin
          if (|instr_i.consumed) begin
            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i] && !found_taken) begin
                found_taken    = 1'b1;
                branch_slot    = CHUNK_PTR_W'(i);
                // check if the taken branch is the very last valid slot
                branch_is_last = 1'b1;
                for (int j = i + 1; j < SLOTS_PER_CYCLE; j++) begin
                  if (instr_i.valid[j]) branch_is_last = 1'b0;
                end
              end
            end

            if (found_taken) begin
              // Record all instructions from slot 0 up to and including the
              // taken branch. Whether the branch is last or not we still start
              // the trace ? we always need at least this window recorded.
              trace_start_ghr_d = ghr_i;
              temp_chunk_ptr    = '0;
              temp_br_cnt       = '0;
              chunk_ptr_d       = '0;
              br_cnt_d          = '0;
              trace_d           = '0;

              for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
                if (instr_i.valid[i] && (CHUNK_PTR_W'(i) <= branch_slot)) begin
                  is_compressed = (instr_i.inst[i][1:0] != 2'b11);

                  if (i == 0)
                    trace_d.base_pc = instr_i.pc[i];

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

              chunk_ptr_d    = temp_chunk_ptr;
              br_cnt_d       = temp_br_cnt;
              sram_wr_addr_d = instr_i.pc[0][TRACE_ADDRW+1:2];

              // Only go to ACCUM if there are valid instructions after the
              // taken branch in this same window. If the taken branch is the
              // last slot there is nothing to accumulate yet ? stay IDLE and
              // wait for the next window (the branch target) to start a trace.
              if (!branch_is_last)
                state_d = ACCUM;
            end
          end
        end

        // -------------------------------------------------
        // ACCUM: keep filling the trace across fetch windows.
        // Only process genuinely new windows (|instr_i.consumed).
        //
        // Exit conditions:
        //   - Another taken branch found    ? commit (trace end = taken branch)
        //   - Trace full (no chunk space)   ? commit
        //   Otherwise stay in ACCUM.
        // -------------------------------------------------
        ACCUM: begin
          if (|instr_i.consumed) begin
            temp_chunk_ptr     = chunk_ptr_q;
            temp_br_cnt        = br_cnt_q;
            hit_taken_in_accum = 1'b0;
            trace_full         = 1'b0;

            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              is_compressed = (instr_i.inst[i][1:0] != 2'b11);

              has_space = is_compressed ?
                          (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                          (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

              if (instr_i.valid[i] && has_space && !hit_taken_in_accum) begin

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

                if (instr_i.is_branch[i]) begin
                  if (temp_br_cnt < CHUNKS_PER_TRACE) begin
                    trace_d.branch_flags[temp_br_cnt] = instr_i.taken[i];
                    if (instr_i.taken[i])
                      last_branch_target_d = instr_i.target[i];
                  end
                  temp_br_cnt = temp_br_cnt + 1;

                  // Taken branch ends the trace ? stop adding instructions
                  if (instr_i.taken[i])
                    hit_taken_in_accum = 1'b1;
                end

              end else if (instr_i.valid[i] && !has_space) begin
                // There is a valid instruction but no room ? trace is full
                trace_full = 1'b1;
              end
            end

            chunk_ptr_d = temp_chunk_ptr;
            br_cnt_d    = temp_br_cnt;

            // Commit only when the trace is full (no chunk space left).
            // hit_taken_in_accum is used only as a loop guard above to stop
            // adding instructions within this window after a taken branch ?
            // it does NOT end the trace. The pipeline will naturally continue
            // fetching from the branch target on the next window.
            if (trace_full) begin
              commit_chunk_ptr_d   = temp_chunk_ptr;
              trace_d.valid        = 1'b1;
              trace_d.target_addr  = last_branch_target_d;
              trace_d.num_branches = BR_CNT_W'(temp_br_cnt);
              commit_valid_d       = 1'b1;
              commit_data_d        = trace_d;
              state_d              = IDLE;
              chunk_ptr_d          = '0;
              br_cnt_d             = '0;
              trace_d              = '0;
            end
            // else: stay in ACCUM, wait for next window
          end
        end

        // -------------------------------------------------
        // COMMIT state is no longer used ? commit is now done
        // inline in IDLE (branch_is_last) and ACCUM.
        // Kept as a safe catch-all that redirects to IDLE.
        // -------------------------------------------------
        COMMIT: begin
          state_d = IDLE;
        end

      endcase
    end
  end

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
      $display("[TC-BUILDER]   chunks used= %0d", commit_chunk_ptr_q);
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