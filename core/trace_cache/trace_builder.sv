`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_builder (
    input  logic clk_i,
    input  logic rst_ni,
    
    tracebuilder_instr_if.consumer instr_i,
    
    input  logic [GHR_WIDTH-1:0] ghr_i,
    input  logic                 flush_i,

    output logic         trace_valid_o,
    output logic [383:0] trace_data_o,      

    output logic                   mem_req_o,
    output logic                   mem_we_o,
    output logic [TRACE_ADDRW-1:0] mem_addr_o,
    output logic [383:0]           mem_wdata_o,  
    output logic [47:0]            mem_be_o,     

    output logic                   tag_valid_o,
    output logic [PC_WIDTH-1:0]    tag_start_pc_o,
    output logic [GHR_WIDTH-1:0]   tag_start_ghr_o,
    output logic [3:0]             tag_trace_len_o,
    output logic [TRACE_ADDRW-1:0] tag_sram_addr_o
);

  // ========================================================================
  // Constants
  // ========================================================================
  
  // Each trace entry = PC (64) + inst (32) = 96 bits
  localparam int unsigned ENTRY_WIDTH = PC_WIDTH + 32;
  // Total trace width = 4 entries × 96 bits = 384 bits
  localparam int unsigned TRACE_WIDTH = TRACE_LEN * ENTRY_WIDTH;

  // ========================================================================
  // Type Definitions
  // ========================================================================
  
  typedef enum logic [1:0] { 
    IDLE, ACCUM, COMMIT
  } state_t;

  // ========================================================================
  // State Registers
  // ========================================================================
  
  state_t state_q, state_d;
  
  trace_entry_t trace_buf [TRACE_LEN-1:0];
  
  localparam int unsigned PTR_W = $clog2(TRACE_LEN+1);
  logic [PTR_W-1:0] trace_ptr_q, trace_ptr_d;
  
  localparam int unsigned BR_CNT_W = $clog2(MAX_BRANCHES+1);
  logic [BR_CNT_W-1:0] br_cnt_q, br_cnt_d;
  
  logic [TRACE_ADDRW-1:0] sram_wr_ptr_q, sram_wr_ptr_d;
  
  logic                commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q, commit_data_d;  
  
  logic [PC_WIDTH-1:0]  trace_start_pc_q,  trace_start_pc_d;
  logic [GHR_WIDTH-1:0] trace_start_ghr_q, trace_start_ghr_d;

  // ========================================================================
  // Output Assignments
  // ========================================================================
  
  assign instr_i.ready = 1'b1;

  assign mem_req_o   = commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = sram_wr_ptr_q;
  assign mem_wdata_o = commit_data_q;
  assign mem_be_o    = {48{1'b1}}; 

  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

  assign tag_valid_o      = commit_valid_q;
  assign tag_start_pc_o   = trace_start_pc_q;
  assign tag_start_ghr_o  = trace_start_ghr_q;
  assign tag_trace_len_o  = trace_ptr_q;
  assign tag_sram_addr_o  = sram_wr_ptr_q;

  // ========================================================================
  // Sequential Logic
  // ========================================================================
  
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q            <= IDLE;
      trace_ptr_q        <= '0;
      br_cnt_q           <= '0;
      sram_wr_ptr_q      <= '0;
      commit_valid_q     <= 1'b0;
      commit_data_q      <= '0;
      trace_start_pc_q   <= '0;
      trace_start_ghr_q  <= '0;
    end else begin
      state_q            <= state_d;
      trace_ptr_q        <= trace_ptr_d;
      br_cnt_q           <= br_cnt_d;
      sram_wr_ptr_q      <= sram_wr_ptr_d;
      commit_valid_q     <= commit_valid_d;
      commit_data_q      <= commit_data_d;
      trace_start_pc_q   <= trace_start_pc_d;
      trace_start_ghr_q  <= trace_start_ghr_d;
    end
  end

  // ========================================================================
  // Combinational Logic
  // ========================================================================
  
  always_comb begin
    state_d            = state_q;
    trace_ptr_d        = trace_ptr_q;
    br_cnt_d           = br_cnt_q;
    sram_wr_ptr_d      = sram_wr_ptr_q;
    commit_valid_d     = 1'b0;
    commit_data_d      = commit_data_q;
    trace_start_pc_d   = trace_start_pc_q;
    trace_start_ghr_d  = trace_start_ghr_q;

    if (flush_i) begin
      state_d     = IDLE;
      trace_ptr_d = '0;
      br_cnt_d    = '0;
    end else begin
      
      case (state_q)

        IDLE: begin
          trace_ptr_d = '0;
          br_cnt_d    = '0;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.ready && trace_ptr_d == 0) begin
              state_d            = ACCUM;
              trace_buf[0]       = '{instr_i.pc[i], instr_i.inst[i], 
                                     instr_i.is_branch[i], instr_i.taken[i]};
              trace_ptr_d        = 1;
              trace_start_pc_d   = instr_i.pc[i];
              trace_start_ghr_d  = ghr_i;
              
              if (instr_i.is_branch[i] && instr_i.taken[i])
                br_cnt_d = 1;
            end
          end
        end

        ACCUM: begin
          logic window_closed;
          logic [PTR_W-1:0] temp_ptr;
          logic [BR_CNT_W-1:0] temp_br_cnt;
          
          window_closed = 1'b0;
          temp_ptr = trace_ptr_q;
          temp_br_cnt = br_cnt_q;

          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.ready && 
                !window_closed && temp_ptr < TRACE_LEN) begin
              
              trace_buf[temp_ptr] = '{instr_i.pc[i], instr_i.inst[i],
                                      instr_i.is_branch[i], instr_i.taken[i]};
              temp_ptr++;
              
              if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                temp_br_cnt++;
                window_closed = 1'b1;
              end
            end
          end
          
          trace_ptr_d = temp_ptr;
          br_cnt_d    = temp_br_cnt;
          
          if (trace_ptr_d >= TRACE_LEN || br_cnt_d >= MAX_BRANCHES) begin
            state_d = COMMIT;
          end
        end

        COMMIT: begin
          commit_valid_d = 1'b1;
          
          // ? Pack 4 × 96-bit entries = 384 bits
          commit_data_d = {
            trace_buf[3].pc, trace_buf[3].inst,
            trace_buf[2].pc, trace_buf[2].inst,
            trace_buf[1].pc, trace_buf[1].inst,
            trace_buf[0].pc, trace_buf[0].inst
          };
          
          sram_wr_ptr_d = sram_wr_ptr_q + 1;
          
          state_d     = IDLE;
          trace_ptr_d = '0;
          br_cnt_d    = '0;
        end

      endcase
    end
  end

  // ========================================================================
  // Debug Display
  // ========================================================================
  
  always_ff @(posedge clk_i) begin
    if (state_q != IDLE || |instr_i.valid) begin
      $display("[TB-%0t] state=%s ptr=%0d br=%0d | slots: v=%4b pc={%h,%h,%h,%h} cf=%4b tk=%4b",
               $time,
               state_q.name(),
               trace_ptr_q,
               br_cnt_q,
               instr_i.valid,
               instr_i.pc[0], instr_i.pc[1], instr_i.pc[2], instr_i.pc[3],
               instr_i.is_branch,
               instr_i.taken);
    end
  end

endmodule