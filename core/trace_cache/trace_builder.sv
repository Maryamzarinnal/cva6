`timescale 1ns/1ps
import trace_cache_pkg::*;
//
//
// A "trace" in this design is: the sequence of instructions in the fetch
// window that contains a taken branch, from slot 0 up to and including
// that taken branch. It is tagged by:
//   - base_pc:      the fetch-aligned address of that window
//   - branch_flags: the PREDICTED outcomes of branches in that window
//   - num_branches: how many branches are in the base window (tag width)
//
// Why predictions and not actual outcomes?
//   At lookup time the frontend hasn't executed the instructions yet.
//   It only has BHT/BTB/RAS outputs. So branch_flags must store what
//   the predictor said, not what actually happened. If actual != predicted,
//   the backend will flush anyway, so a wrong-path trace hit is harmless
//   (it gets flushed), but a miss due to prediction mismatch wastes cycles.
//
// States:
//   IDLE   - waiting for a taken branch in the current fetch window
//   ACCUM  - collecting instructions from following windows to fill the trace
//   COMMIT - kept as safe fallback

module trace_builder (
    input  logic clk_i,
    input  logic rst_ni,

    // Instruction window from frontend (4 slots)
    tracebuilder_instr_if.consumer instr_i,

    // Global history register (not used in tag matching currently)
    input  logic [GHR_WIDTH-1:0] ghr_i,

    // abort any in-progress trace
    input  logic flush_i,

    // Branch PREDICTIONS for the current fetch window.
    // We store these (not actual taken bits) as the trace tag so that
    // at lookup time the tag comparison can succeed using predictor outputs.
    input  logic [CHUNKS_PER_TRACE-1:0] branch_predictions_i,

    // Committed trace output (goes to SRAM)
    output logic                   trace_valid_o,
    output logic [TRACE_WIDTH-1:0] trace_data_o,

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
  logic [BR_CNT_W-1:0]    br_cnt_q, br_cnt_d;
  logic [PC_WIDTH-1:0]    last_branch_target_q, last_branch_target_d;
  logic [TRACE_ADDRW-1:0] sram_wr_addr_q, sram_wr_addr_d;
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q,  commit_data_d;
  logic [GHR_WIDTH-1:0]   trace_start_ghr_q, trace_start_ghr_d;
  logic [CHUNK_PTR_W-1:0] commit_chunk_ptr_q, commit_chunk_ptr_d;
  logic [PC_WIDTH-1:0]    last_instr_pc_q, last_instr_pc_d;
  logic                   last_instr_compressed_q, last_instr_compressed_d;
  // Was the last appended instruction a taken control flow? Used for target_addr at commit.
  logic                   last_instr_was_taken_q, last_instr_was_taken_d;
  // Total taken branches recorded across the entire trace (base + ACCUM windows).
  // Commit when this reaches MAX_TAKEN (3), limiting trace to 3 control flow redirections.
  localparam int unsigned MAX_TAKEN = 3;

  // Per-set duplicate filter: remembers the last committed (pc, flags) for each
  // set index. Catches the interleaving pattern (A?B?A?B) that a single-entry
  // filter misses, eliminating thousands of redundant SRAM writes.
  logic                        dup_valid [(1 << TRACE_ADDRW)];
  logic [PC_WIDTH-1:0]         dup_pc    [(1 << TRACE_ADDRW)];
  logic [CHUNKS_PER_TRACE-1:0] dup_flags [(1 << TRACE_ADDRW)];
  logic [1:0] taken_cnt_q, taken_cnt_d;

  assign instr_i.ready = 1'b1;
  assign mem_req_o     = commit_valid_q;
  assign mem_we_o      = commit_valid_q;
  assign mem_addr_o    = sram_wr_addr_q;
  assign mem_wdata_o   = commit_data_q;
  assign mem_be_o      = {BE_WIDTH{1'b1}};
  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q                  <= IDLE;
      trace_q                  <= '0;
      chunk_ptr_q              <= '0;
      br_cnt_q                 <= '0;
      last_branch_target_q     <= '0;
      sram_wr_addr_q           <= '0;
      commit_valid_q           <= 1'b0;
      commit_data_q            <= '0;
      trace_start_ghr_q        <= '0;
      commit_chunk_ptr_q       <= '0;
      last_instr_pc_q          <= '0;
      last_instr_compressed_q  <= 1'b0;
      last_instr_was_taken_q   <= 1'b0;
      taken_cnt_q              <= '0;

    end else begin
      state_q                  <= state_d;
      trace_q                  <= trace_d;
      chunk_ptr_q              <= chunk_ptr_d;
      br_cnt_q                 <= br_cnt_d;
      last_branch_target_q     <= last_branch_target_d;
      sram_wr_addr_q           <= sram_wr_addr_d;
      commit_valid_q           <= commit_valid_d;
      commit_data_q            <= commit_data_d;
      trace_start_ghr_q        <= trace_start_ghr_d;
      commit_chunk_ptr_q       <= commit_chunk_ptr_d;
      last_instr_pc_q          <= last_instr_pc_d;
      last_instr_compressed_q  <= last_instr_compressed_d;
      last_instr_was_taken_q   <= last_instr_was_taken_d;
      taken_cnt_q              <= taken_cnt_d;

    end
  end

  always_comb begin
    // reset each combinational evaluation
    logic [CHUNK_PTR_W-1:0] temp_chunk_ptr;
    logic [BR_CNT_W-1:0]    temp_br_cnt;
    logic                   is_compressed;
    logic                   has_space;
    logic                   found_taken;
    logic                   branch_is_last;
    logic [CHUNK_PTR_W-1:0] branch_slot;
    logic                   hit_taken;
    logic                   trace_full;

    // Default
    state_d                  = state_q;
    trace_d                  = trace_q;
    chunk_ptr_d              = chunk_ptr_q;
    br_cnt_d                 = br_cnt_q;
    last_branch_target_d     = last_branch_target_q;
    sram_wr_addr_d           = sram_wr_addr_q;
    commit_valid_d           = 1'b0;   // pulse only, cleared every cycle
    commit_data_d            = commit_data_q;
    trace_start_ghr_d        = trace_start_ghr_q;
    commit_chunk_ptr_d       = commit_chunk_ptr_q;
    last_instr_pc_d          = last_instr_pc_q;
    last_instr_compressed_d  = last_instr_compressed_q;
    last_instr_was_taken_d   = last_instr_was_taken_q;
    taken_cnt_d              = taken_cnt_q;


    temp_chunk_ptr   = '0;
    temp_br_cnt      = '0;
    is_compressed    = 1'b0;
    has_space        = 1'b0;
    found_taken      = 1'b0;
    branch_is_last   = 1'b0;
    branch_slot      = '0;
    hit_taken        = 1'b0;
    trace_full       = 1'b0;

    // Flush overrides everything 
    if (flush_i) begin
      state_d                 = IDLE;
      chunk_ptr_d             = '0;
      br_cnt_d                = '0;
      trace_d                 = '0;
      last_instr_pc_d         = '0;
      last_instr_compressed_d = 1'b0;
      last_instr_was_taken_d  = 1'b0;
      taken_cnt_d             = '0;

    end else begin

      case (state_q)

        // -----------------------------------------------------------------
        // IDLE: scan the current fetch window for the first taken branch
        //
        // When we find one, we start a new trace:
        //   - base_pc      = fetch-aligned address of this window (pc[0] & ~0xf)
        //                    Must match lookup_pc_i (icache_vaddr_q) which is also
        //                    fetch-aligned. Both SRAM index and tag compare use this.
        //   - branch_flags = PREDICTED outcomes from branch_predictions_i
        //   - num_branches = number of branches in this base window only
        //   - chunks       = instructions from slot 0 up to and including
        //                    the taken branch slot
        //
        // Then we move to ACCUM to keep filling from the next windows.
        // -----------------------------------------------------------------
        IDLE: begin
          // Do not start a new base window while instr_realign is still
          // stitching an unaligned instruction across cache blocks. Wait
          // until we see a fully aligned logical window.
          if (|instr_i.consumed && !instr_i.serving_unaligned) begin

            // Find the first taken branch in this window
            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i] && !found_taken) begin
                found_taken = 1'b1;
                branch_slot = CHUNK_PTR_W'(i);
              end
            end

            if (found_taken) begin
              // Start a fresh trace
              trace_start_ghr_d       = ghr_i;
              temp_chunk_ptr          = '0;
              temp_br_cnt             = '0;
              chunk_ptr_d             = '0;
              br_cnt_d                = '0;
              trace_d                 = '0;
              last_instr_pc_d         = '0;
              last_instr_compressed_d = 1'b0;
              last_instr_was_taken_d  = 1'b0;  // clear stale state from previous trace

              // base_pc = fetch-aligned address of this fetch window.
              // Mask pc[0] to 16-byte boundary (4 slots x 4 bytes).
              trace_d.base_pc = instr_i.pc[0] & {{(PC_WIDTH-4){1'b1}}, 4'b0000};

              // branch_flags = PREDICTED outcomes for this window.
              // in slot order (computed in frontend.sv tc_branch_predictions)
              // We store the whole vector; num_branches tells the lookup
              // how many bits are meaningful.
              trace_d.branch_flags = branch_predictions_i;

              // Record instructions from slot 0 up to and including the taken branch
              for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
                if (instr_i.valid[i] && (CHUNK_PTR_W'(i) <= branch_slot)) begin
                  is_compressed = (instr_i.inst[i][1:0] != 2'b11);

                  // Track last instruction added
                  last_instr_pc_d         = instr_i.pc[i];
                  last_instr_compressed_d = is_compressed;
                  // Track if this instruction is a taken CF
                  last_instr_was_taken_d  = instr_i.is_branch[i] && instr_i.taken[i];

                  if (!is_compressed) begin
                    // 32-bit instruction: occupies 2 consecutive chunks
                    trace_d.chunks[temp_chunk_ptr]         = instr_i.inst[i][15:0];
                    trace_d.chunks[temp_chunk_ptr+1]       = instr_i.inst[i][31:16];
                    trace_d.valid_chunks[temp_chunk_ptr]   = 1'b1;  // marks instruction start
                    trace_d.valid_chunks[temp_chunk_ptr+1] = 1'b0;  // high half, not a new instr
                    temp_chunk_ptr = temp_chunk_ptr + 2;
                  end else begin
                    // 16-bit compressed instruction: 1 chunk
                    trace_d.chunks[temp_chunk_ptr]       = instr_i.inst[i][15:0];
                    trace_d.valid_chunks[temp_chunk_ptr] = 1'b1;
                    temp_chunk_ptr = temp_chunk_ptr + 1;
                  end

                  if (instr_i.is_branch[i]) begin
                    // Track branch target
                    if (instr_i.taken[i])
                      last_branch_target_d = instr_i.target[i];
                    temp_br_cnt = temp_br_cnt + 1;
                  end
                end
              end

              // num_branches = branches in base window only.
              // This tells the lookup exactly how many branch_flags bits to compare.
              trace_d.num_branches = BR_CNT_W'(temp_br_cnt);

              chunk_ptr_d  = temp_chunk_ptr;
              br_cnt_d     = temp_br_cnt;
              taken_cnt_d  = 2'd1;   // first taken branch in the trace
              state_d      = ACCUM;
            end
          end
        end

        // -----------------------------------------------------------------
        // ACCUM: keep adding instructions from following fetch windows.
        //
        // We commit the trace when:
        //   - The chunk buffer is full (CHUNKS_PER_TRACE chunks), OR
        //   - The trace has accumulated MAX_TAKEN (3) taken branches total
        //     counting the initial taken branch from the base window
        // -----------------------------------------------------------------
        ACCUM: begin
          if (|instr_i.consumed) begin
            temp_chunk_ptr = chunk_ptr_q;
            temp_br_cnt    = br_cnt_q;
            hit_taken      = 1'b0;
            trace_full     = 1'b0;

            for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
              is_compressed = (instr_i.inst[i][1:0] != 2'b11);
              has_space = is_compressed ?
                          (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                          (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

              if (instr_i.valid[i] && has_space && !hit_taken) begin
                last_instr_pc_d         = instr_i.pc[i];
                last_instr_compressed_d = is_compressed;
                // Track if this instruction is a taken CF (for target_addr decision)
                last_instr_was_taken_d  = instr_i.is_branch[i] && instr_i.taken[i];

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
                  // ACCUM branch: track target and count
                  // branch_flags or num_branches
                  if (instr_i.taken[i]) begin
                    last_branch_target_d = instr_i.target[i];
                    taken_cnt_d = taken_cnt_q + 2'd1;
                    hit_taken = 1'b1; // stop processing this window, next from target
                  end
                  temp_br_cnt = temp_br_cnt + 1;
                end

              end else if (instr_i.valid[i] && !has_space) begin
                trace_full = 1'b1;
              end
            end

            chunk_ptr_d = temp_chunk_ptr;
            br_cnt_d    = temp_br_cnt;

            // Commit when buffer is full OR when we've chained through MAX_TAKEN
            if (trace_full || (taken_cnt_d >= MAX_TAKEN[1:0] && temp_chunk_ptr > 0)) begin
              logic [TRACE_ADDRW-1:0] candidate_addr;
              logic                   is_duplicate;

              commit_chunk_ptr_d  = temp_chunk_ptr;
              trace_d.valid       = 1'b1;

              // target_addr = where fetch continues after replaying this trace.
              // Depends on what the LAST APPENDED instruction was:
              //   - if it was a taken CF use its target (redirect)
              //   - otherwise fall-through of last instruction
              // We use last_instr_was_taken_d (not hit_taken) because hit_taken
              // reflects the current window, not necessarily the last appended instr.
              trace_d.target_addr = last_instr_was_taken_d
                                    ? last_branch_target_d
                                    : last_instr_pc_d + (last_instr_compressed_d ? 64'h2 : 64'h4);

              candidate_addr = tc_index(trace_d.base_pc, trace_d.branch_flags);

              // Per-set duplicate filter: skip commit if this set already holds
              // the same (pc, flags) trace. Eliminates interleaving re-commits.
              is_duplicate = dup_valid[candidate_addr]
                           && (trace_d.base_pc      == dup_pc[candidate_addr])
                           && (trace_d.branch_flags == dup_flags[candidate_addr]);

              if (!is_duplicate) begin
                sram_wr_addr_d       = candidate_addr;
                commit_valid_d       = 1'b1;
                commit_data_d        = trace_d;
              end

              state_d         = IDLE;
              chunk_ptr_d     = '0;
              br_cnt_d        = '0;
              taken_cnt_d     = '0;
              trace_d         = '0;
            end
          end
        end

        // Safe fallback
        COMMIT: begin
          state_d = IDLE;
        end

      endcase
    end
  end

  // Per-set duplicate filter update: register the committed trace's identity
  // so future builds to the same set are suppressed.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 0; i < (1 << TRACE_ADDRW); i++)
        dup_valid[i] <= 1'b0;
    end else if (commit_valid_d) begin
      trace_data_t dup_tmp;
      dup_tmp = trace_data_t'(commit_data_d);
      dup_valid[sram_wr_addr_d] <= 1'b1;
      dup_pc[sram_wr_addr_d]    <= dup_tmp.base_pc;
      dup_flags[sram_wr_addr_d] <= dup_tmp.branch_flags;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    if (commit_valid_q) begin
      trace_data_t dbg;
      dbg = trace_data_t'(commit_data_q);

      if (dbg.target_addr == dbg.base_pc) begin
        $display("[TC-WARN] target==base_pc base=0x%h target=0x%h",
                 dbg.base_pc, dbg.target_addr);
      end
      if (dbg.target_addr == last_instr_pc_q) begin
        $display("[TC-WARN] target==last_instr_pc base=0x%h last_pc=0x%h target=0x%h",
                 dbg.base_pc, last_instr_pc_q, dbg.target_addr);
      end

      $display("[TC-BUILDER] ---- TRACE COMMITTED (non-duplicate) ----");
      $display("[TC-BUILDER]   SRAM addr  = %0d", sram_wr_addr_q);
      $display("[TC-BUILDER]   base_pc    = 0x%h", dbg.base_pc);
      $display("[TC-BUILDER]   HASH: pc[%0d:4]=0x%h ^ pc[%0d:%0d]=0x%h ^ pc[%0d:%0d]=0x%h ^ flags[1:0]=%b => set=%0d",
               TRACE_ADDRW+3,
               dbg.base_pc[TRACE_ADDRW+3:4],
               2*TRACE_ADDRW+3, TRACE_ADDRW+4,
               dbg.base_pc[2*TRACE_ADDRW+3:TRACE_ADDRW+4],
               3*TRACE_ADDRW+3, 2*TRACE_ADDRW+4,
               dbg.base_pc[3*TRACE_ADDRW+3:2*TRACE_ADDRW+4],
               dbg.branch_flags[TC_INDEX_FLAG_BITS-1:0],
               sram_wr_addr_q);
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
