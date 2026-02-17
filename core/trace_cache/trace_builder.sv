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

  logic [PC_WIDTH-1:0] last_branch_pc_q, last_branch_pc_d;
  logic [PC_WIDTH-1:0] last_branch_target_q, last_branch_target_d;
  logic                last_branch_taken_q, last_branch_taken_d;

  logic [TRACE_ADDRW-1:0] sram_wr_ptr_q, sram_wr_ptr_d;

  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q, commit_data_d;

  logic [PC_WIDTH-1:0]  trace_start_pc_q, trace_start_pc_d;
  logic [GHR_WIDTH-1:0] trace_start_ghr_q, trace_start_ghr_d;

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
      last_branch_pc_q     <= '0;
      last_branch_target_q <= '0;
      last_branch_taken_q  <= 1'b0;
      sram_wr_ptr_q        <= '0;
      commit_valid_q       <= 1'b0;
      commit_data_q        <= '0;
      trace_start_pc_q     <= '0;
      trace_start_ghr_q    <= '0;
    end else begin
      state_q              <= state_d;
      trace_q              <= trace_d;
      chunk_ptr_q          <= chunk_ptr_d;
      br_cnt_q             <= br_cnt_d;
      last_branch_pc_q     <= last_branch_pc_d;
      last_branch_target_q <= last_branch_target_d;
      last_branch_taken_q  <= last_branch_taken_d;
      sram_wr_ptr_q        <= sram_wr_ptr_d;
      commit_valid_q       <= commit_valid_d;
      commit_data_q        <= commit_data_d;
      trace_start_pc_q     <= trace_start_pc_d;
      trace_start_ghr_q    <= trace_start_ghr_d;
    end
  end

  always_comb begin
    logic window_closed;
    logic abort_trace;
    logic [CHUNK_PTR_W-1:0] temp_chunk_ptr;
    logic [BR_CNT_W-1:0]    temp_br_cnt;
    logic                   is_compressed;
    logic                   has_space;
    logic [PC_WIDTH-1:0]    last_pc;

    state_d              = state_q;
    trace_d              = trace_q;
    chunk_ptr_d          = chunk_ptr_q;
    br_cnt_d             = br_cnt_q;
    last_branch_pc_d     = last_branch_pc_q;
    last_branch_target_d = last_branch_target_q;
    last_branch_taken_d  = last_branch_taken_q;
    sram_wr_ptr_d        = sram_wr_ptr_q;
    commit_valid_d       = 1'b0;
    commit_data_d        = commit_data_q;
    trace_start_pc_d     = trace_start_pc_q;
    trace_start_ghr_d    = trace_start_ghr_q;

    window_closed  = 1'b0;
    abort_trace    = 1'b0;
    temp_chunk_ptr = chunk_ptr_q;
    temp_br_cnt    = br_cnt_q;
    is_compressed  = 1'b0;
    has_space      = 1'b0;
    last_pc        = '0;

    if (flush_i) begin
      state_d     = IDLE;
      chunk_ptr_d = '0;
      br_cnt_d    = '0;
    end else begin

      case (state_q)

        IDLE: begin
          chunk_ptr_d = '0;
          br_cnt_d    = '0;
          trace_d     = '0;

          last_branch_pc_d     = '0;
          last_branch_target_d = '0;
          last_branch_taken_d  = 1'b0;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.ready && chunk_ptr_d == 0) begin
              state_d = ACCUM;

              trace_start_pc_d  = instr_i.pc[i];
              trace_start_ghr_d = ghr_i;
              trace_d.base_pc   = instr_i.pc[i];

              if (instr_i.inst[i][1:0] == 2'b11) begin
                trace_d.chunks[0]       = instr_i.inst[i][15:0];
                trace_d.chunks[1]       = instr_i.inst[i][31:16];
                trace_d.valid_chunks[0] = 1'b1;
                trace_d.valid_chunks[1] = 1'b0;
                chunk_ptr_d = 2;
              end else begin
                trace_d.chunks[0]       = instr_i.inst[i][15:0];
                trace_d.valid_chunks[0] = 1'b1;
                chunk_ptr_d = 1;
              end

              if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                br_cnt_d = 1;
                last_branch_pc_d     = instr_i.pc[i];
                last_branch_target_d = instr_i.target[i];
                last_branch_taken_d  = 1'b1;
              end
            end
          end
        end

        ACCUM: begin
          window_closed  = 1'b0;
          abort_trace    = 1'b0;
          temp_chunk_ptr = chunk_ptr_q;
          temp_br_cnt    = br_cnt_q;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            is_compressed = (instr_i.inst[i][1:0] != 2'b11);

            has_space = is_compressed ?
                        (temp_chunk_ptr + 1 <= CHUNKS_PER_TRACE) :
                        (temp_chunk_ptr + 2 <= CHUNKS_PER_TRACE);

            if (instr_i.valid[i] && instr_i.ready && !window_closed && !abort_trace && has_space) begin

              if (!is_compressed) begin
                trace_d.chunks[temp_chunk_ptr]     = instr_i.inst[i][15:0];
                trace_d.chunks[temp_chunk_ptr+1]   = instr_i.inst[i][31:16];
                trace_d.valid_chunks[temp_chunk_ptr]   = 1'b1;
                trace_d.valid_chunks[temp_chunk_ptr+1] = 1'b0;
                temp_chunk_ptr = temp_chunk_ptr + 2;
              end else begin
                trace_d.chunks[temp_chunk_ptr]       = instr_i.inst[i][15:0];
                trace_d.valid_chunks[temp_chunk_ptr] = 1'b1;
                temp_chunk_ptr = temp_chunk_ptr + 1;
              end

              if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                temp_br_cnt = temp_br_cnt + 1;
                last_branch_pc_d     = instr_i.pc[i];
                last_branch_target_d = instr_i.target[i];
                last_branch_taken_d  = 1'b1;

                if (temp_br_cnt < MAX_BRANCHES) begin
                  trace_d.branch_flags[temp_br_cnt-1] = 1'b1;
                end

                window_closed = 1'b1;
              end
            end
          end

          chunk_ptr_d = temp_chunk_ptr;
          br_cnt_d    = temp_br_cnt;

          if (chunk_ptr_d >= CHUNKS_PER_TRACE || br_cnt_d >= MAX_BRANCHES || abort_trace) begin
            state_d = COMMIT;
          end
        end

        COMMIT: begin
          commit_valid_d = 1'b1;

          trace_d.valid = 1'b1;

          if (last_branch_taken_q) begin
            trace_d.target_addr       = last_branch_target_q;
            trace_d.fall_through_addr = last_branch_pc_q +
                                        ((last_branch_pc_q[1]) ? 64'd2 : 64'd4);
          end else begin
            last_pc = trace_q.base_pc + (chunk_ptr_q * 2);
            trace_d.target_addr       = last_pc;
            trace_d.fall_through_addr = last_pc;
          end

          trace_d.num_branches = br_cnt_q;
          commit_data_d        = trace_d;

          sram_wr_ptr_d = sram_wr_ptr_q + 1;

          state_d     = IDLE;
          chunk_ptr_d = '0;
          br_cnt_d    = '0;
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