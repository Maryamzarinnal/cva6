`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_builder (
    input  logic clk_i,
    input  logic rst_ni,

    tracebuilder_instr_if.consumer instr_i,

    input  logic [GHR_WIDTH-1:0] ghr_i,
    input  logic                 flush_i,

    output logic                   trace_valid_o,
    output logic [TRACE_WIDTH-1:0] trace_data_o,

    output logic                   mem_req_o,
    output logic                   mem_we_o,
    output logic [TRACE_ADDRW-1:0] mem_addr_o,
    output logic [TRACE_WIDTH-1:0] mem_wdata_o,
    output logic [BE_WIDTH-1:0]    mem_be_o
);

  localparam int unsigned CHUNK_PTR_W = $clog2(CHUNKS_PER_TRACE + 1);
  localparam int unsigned BR_CNT_W    = $clog2(MAX_BRANCHES + 1);

  typedef enum logic [1:0] {
    IDLE, ACCUM, COMMIT
  } state_t;

  state_t state_q, state_d;
  trace_data_t trace_q, trace_d;

  logic [CHUNK_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;
  logic [BR_CNT_W-1:0]    br_cnt_q, br_cnt_d;

  logic [PC_WIDTH-1:0]    last_branch_target_q, last_branch_target_d;
  logic [TRACE_ADDRW-1:0] sram_wr_ptr_q, sram_wr_ptr_d;

  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q, commit_data_d;

  logic [GHR_WIDTH-1:0]   trace_start_ghr_q, trace_start_ghr_d;

  assign instr_i.ready = 1'b1;

  assign mem_req_o   = commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = sram_wr_ptr_q;
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
      sram_wr_ptr_q        <= '0;
      commit_valid_q       <= 1'b0;
      commit_data_q        <= '0;
      trace_start_ghr_q    <= '0;
    end else begin
      state_q              <= state_d;
      trace_q              <= trace_d;
      chunk_ptr_q          <= chunk_ptr_d;
      br_cnt_q             <= br_cnt_d;
      last_branch_target_q <= last_branch_target_d;
      sram_wr_ptr_q        <= sram_wr_ptr_d;
      commit_valid_q       <= commit_valid_d;
      commit_data_q        <= commit_data_d;
      trace_start_ghr_q    <= trace_start_ghr_d;
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

    state_d              = state_q;
    trace_d              = trace_q;
    chunk_ptr_d          = chunk_ptr_q;
    br_cnt_d             = br_cnt_q;
    last_branch_target_d = last_branch_target_q;
    sram_wr_ptr_d        = sram_wr_ptr_q;
    commit_valid_d       = 1'b0;
    commit_data_d        = commit_data_q;
    trace_start_ghr_d    = trace_start_ghr_q;

    temp_chunk_ptr = '0;
    temp_br_cnt    = '0;
    is_compressed  = 1'b0;
    has_space      = 1'b0;
    found_taken    = 1'b0;
    branch_is_last = 1'b0;
    branch_slot    = '0;

    if (flush_i) begin
      state_d     = IDLE;
      chunk_ptr_d = '0;
      br_cnt_d    = '0;
      trace_d     = '0;
    end else begin

      case (state_q)

        IDLE: begin
          chunk_ptr_d = '0;
          br_cnt_d    = '0;
          trace_d     = '0;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.is_branch[i] && instr_i.taken[i] && !found_taken) begin
              found_taken = 1'b1;
              branch_slot = CHUNK_PTR_W'(i);
              // skip if taken branch is the last valid slot in the window
              branch_is_last = 1'b1;
              for (int j = i + 1; j < SLOTS_PER_CYCLE; j++) begin
                if (instr_i.valid[j]) branch_is_last = 1'b0;
              end
            end
          end

          if (found_taken && !branch_is_last) begin
            trace_start_ghr_d = ghr_i;
            temp_chunk_ptr    = '0;
            temp_br_cnt       = '0;

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

                if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                  temp_br_cnt          = temp_br_cnt + 1;
                  last_branch_target_d = instr_i.target[i];
                  if (temp_br_cnt > 1 && temp_br_cnt <= MAX_BRANCHES)
                    trace_d.branch_flags[temp_br_cnt-2] = 1'b1;
                end
              end
            end

            chunk_ptr_d = temp_chunk_ptr;
            br_cnt_d    = temp_br_cnt;
            state_d     = ACCUM;
          end
        end

        ACCUM: begin
          // record instructions from target window
          temp_chunk_ptr = chunk_ptr_q;
          temp_br_cnt    = br_cnt_q;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            is_compressed = (instr_i.inst[i][1:0] != 2'b11);
            has_space = is_compressed ?
                        (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                        (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

            if (instr_i.valid[i] && has_space) begin
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

              if (instr_i.is_branch[i] && instr_i.taken[i] && temp_br_cnt < MAX_BRANCHES) begin
                temp_br_cnt          = temp_br_cnt + 1;
                last_branch_target_d = instr_i.target[i];
                if (temp_br_cnt > 1)
                  trace_d.branch_flags[temp_br_cnt-2] = 1'b1;
              end
            end
          end

          chunk_ptr_d = temp_chunk_ptr;
          br_cnt_d    = temp_br_cnt;
          state_d     = COMMIT;
        end

        COMMIT: begin
          commit_valid_d       = 1'b1;
          trace_d.valid        = 1'b1;
          trace_d.target_addr  = last_branch_target_q;
          trace_d.num_branches = br_cnt_q;
          commit_data_d        = trace_d;
          sram_wr_ptr_d        = sram_wr_ptr_q + 1;
          state_d              = IDLE;
          chunk_ptr_d          = '0;
          br_cnt_d             = '0;
          trace_d              = '0;
        end

      endcase
    end
  end

  always_ff @(posedge clk_i) begin
    if (state_q != IDLE || |instr_i.valid) begin
      $display("[TB-%0t] state=%s chunks=%0d/%0d br=%0d/%0d | v=%4b pc={%h,%h,%h,%h}",
               $time, state_q.name(), chunk_ptr_q, CHUNKS_PER_TRACE, br_cnt_q, MAX_BRANCHES,
               instr_i.valid,
               instr_i.pc[0][31:0], instr_i.pc[1][31:0],
               instr_i.pc[2][31:0], instr_i.pc[3][31:0]);
    end
  end

endmodule