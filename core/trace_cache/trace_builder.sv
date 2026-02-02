`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_builder #(
    parameter int unsigned WINDOW_SIZE  = 4,
    parameter int unsigned TRACE_LEN    = 4,
    parameter int unsigned MAX_BRANCHES = 2,
    parameter int unsigned ADDRW        = 10,
    parameter int unsigned GHR_W        = 8
)(
    input  logic clk_i,
    input  logic rst_ni,
    tracebuilder_instr_if.consumer instr_i,
    input  logic [GHR_W-1:0] ghr_i,

    output logic         trace_valid_o,
    output logic [255:0] trace_data_o,

    output logic               mem_req_o,
    output logic               mem_we_o,
    output logic [ADDRW-1:0]   mem_addr_o,
    output logic [255:0]       mem_wdata_o,
    output logic [31:0]        mem_be_o,

    output logic               tag_valid_o,
    output logic [31:0]        tag_start_pc_o,
    output logic [GHR_W-1:0]   tag_start_ghr_o,
    output logic [3:0]         tag_trace_len_o,
    output logic [ADDRW-1:0]   tag_sram_addr_o
);

  typedef enum logic [1:0] { IDLE, ACCUM, COMMIT } tb_state_e;
  tb_state_e state_q, state_d;

  trace_entry_t trace_buf [TRACE_LEN-1:0];

  localparam int unsigned WIN_IDX_W =
      (WINDOW_SIZE > 1) ? $clog2(WINDOW_SIZE) : 1;
  logic [WIN_IDX_W-1:0] win_idx_q, win_idx_d;

  localparam int unsigned TRACE_PTR_W =
      (TRACE_LEN > 1) ? $clog2(TRACE_LEN+1) : 1;
  logic [TRACE_PTR_W-1:0] chunk_ptr_q, chunk_ptr_d;

  localparam int unsigned BR_CNT_W =
      (MAX_BRANCHES > 0) ? $clog2(MAX_BRANCHES+1) : 1;
  logic [BR_CNT_W-1:0] br_cnt_q, br_cnt_d;

  logic window_done_q, window_done_d;

  logic [ADDRW-1:0] wr_ptr_q, wr_ptr_d;

  logic         commit_valid_q, commit_valid_d;
  logic [255:0] commit_data_q,  commit_data_d;

  logic [31:0]      start_pc_q,  start_pc_d;
  logic [GHR_W-1:0] start_ghr_q, start_ghr_d;

  assign instr_i.ready = 1'b1;

  assign mem_req_o   = commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = wr_ptr_q;
  assign mem_wdata_o = commit_data_q;
  assign mem_be_o    = 32'hFFFF_FFFF;

  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

  assign tag_valid_o      = commit_valid_q;
  assign tag_start_pc_o   = start_pc_q;
  assign tag_start_ghr_o  = start_ghr_q;
  assign tag_trace_len_o  = chunk_ptr_q;
  assign tag_sram_addr_o  = wr_ptr_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
          state_q        <= IDLE;
          win_idx_q      <= '0;
          chunk_ptr_q    <= '0;
          br_cnt_q       <= '0;
          window_done_q  <= 1'b0;
          wr_ptr_q       <= '0;
          commit_valid_q <= 1'b0;
          commit_data_q  <= '0;
          start_pc_q     <= '0;
          start_ghr_q    <= '0;
      end else begin
          state_q        <= state_d;
          win_idx_q      <= win_idx_d;
          chunk_ptr_q    <= chunk_ptr_d;
          br_cnt_q       <= br_cnt_d;
          window_done_q  <= window_done_d;
          wr_ptr_q       <= wr_ptr_d;
          commit_valid_q <= commit_valid_d;
          commit_data_q  <= commit_data_d;
          start_pc_q     <= start_pc_d;
          start_ghr_q    <= start_ghr_d;
      end
  end

  always_comb begin
      state_d        = state_q;
      win_idx_d      = win_idx_q;
      chunk_ptr_d    = chunk_ptr_q;
      br_cnt_d       = br_cnt_q;
      window_done_d  = window_done_q;
      wr_ptr_d       = wr_ptr_q;
      commit_valid_d = 1'b0;
      commit_data_d  = commit_data_q;
      start_pc_d     = start_pc_q;
      start_ghr_d    = start_ghr_q;

      case (state_q)

          IDLE: begin
              win_idx_d     = '0;
              chunk_ptr_d   = '0;
              br_cnt_d      = '0;
              window_done_d = 1'b0;

              if (instr_i.valid && instr_i.ready) begin
                  state_d     = ACCUM;
                  win_idx_d   = (WINDOW_SIZE > 1) ? 'd1 : 'd0;
                  chunk_ptr_d = 'd1;
                  br_cnt_d    = (instr_i.is_branch && instr_i.taken) ? 'd1 : 'd0;

                  if (instr_i.is_branch && instr_i.taken)
                      window_done_d = 1'b1;

                  trace_buf[0] = '{instr_i.pc, instr_i.inst,
                                   instr_i.is_branch, instr_i.taken};

                  start_pc_d  = instr_i.pc;
                  start_ghr_d = ghr_i;
              end
          end

          ACCUM: begin
              if (instr_i.valid && instr_i.ready) begin

                  if (!window_done_q && chunk_ptr_q < TRACE_LEN) begin
                      trace_buf[chunk_ptr_q] =
                          '{instr_i.pc, instr_i.inst,
                            instr_i.is_branch, instr_i.taken};
                  end

                  if (window_done_q) begin
                      if (win_idx_q < WINDOW_SIZE-1)
                          win_idx_d = win_idx_q + 1;
                      else begin
                          win_idx_d     = '0;
                          window_done_d = 1'b0;
                      end
                  end else begin
                      if (chunk_ptr_q < TRACE_LEN)
                          chunk_ptr_d = chunk_ptr_q + 1;

                      win_idx_d = win_idx_q + 1;

                      if (instr_i.is_branch && instr_i.taken) begin
                          br_cnt_d      = br_cnt_q + 1;
                          window_done_d = 1'b1;
                      end
                      else if (win_idx_q == WINDOW_SIZE-1) begin
                          win_idx_d     = '0;
                          window_done_d = 1'b0;
                      end
                  end

                  if (chunk_ptr_d == TRACE_LEN || br_cnt_d == MAX_BRANCHES)
                      state_d = COMMIT;
              end
          end

          COMMIT: begin
              commit_valid_d = 1'b1;
              commit_data_d = {
                  trace_buf[3].pc, trace_buf[3].inst,
                  trace_buf[2].pc, trace_buf[2].inst,
                  trace_buf[1].pc, trace_buf[1].inst,
                  trace_buf[0].pc, trace_buf[0].inst
              };
              wr_ptr_d = wr_ptr_q + 1;

              state_d        = IDLE;
              win_idx_d      = '0;
              chunk_ptr_d    = '0;
              br_cnt_d       = '0;
              window_done_d  = 1'b0;
          end

      endcase
  end

  always_ff @(posedge clk_i) begin
      $display("[%0t] state=%s win_idx=%0d chunk_ptr=%0d br_cnt=%0d window_done=%0b | valid=%b pc=%h inst=%h br=%b tk=%b",
               $time,
               state_q.name(),
               win_idx_q,
               chunk_ptr_q,
               br_cnt_q,
               window_done_q,
               instr_i.valid,
               instr_i.pc,
               instr_i.inst,
               instr_i.is_branch,
               instr_i.taken);
  end

endmodule
