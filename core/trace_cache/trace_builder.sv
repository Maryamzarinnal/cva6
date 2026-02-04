`timescale 1ns/1ps
import trace_cache_pkg::*;

module trace_builder (
    input  logic clk_i,
    input  logic rst_ni,
    
    tracebuilder_instr_if.consumer instr_i,
    
    input  logic [GHR_WIDTH-1:0] ghr_i,
    input  logic                 flush_i,

    output logic                   trace_valid_o,
    output logic [TRACE_WIDTH-1:0] trace_data_o,      // 264 bits (4 × 66-bit entries)

    output logic                   mem_req_o,
    output logic                   mem_we_o,
    output logic [TRACE_ADDRW-1:0] mem_addr_o,
    output logic [TRACE_WIDTH-1:0] mem_wdata_o,       // 264 bits
    output logic [BE_WIDTH-1:0]    mem_be_o,          // 33 bytes 

    output logic                     tag_valid_o,
    output logic [PC_WIDTH_FULL-1:0] tag_start_pc_o,   
    output logic [GHR_WIDTH-1:0]     tag_start_ghr_o,
    output logic [3:0]               tag_trace_len_o,
    output logic [TRACE_ADDRW-1:0]   tag_sram_addr_o
);

  // ========================================================================
  // Derived Constants 
  // ========================================================================
  
  // Trace pointer width: log2(TRACE_LEN + 1)
  // Needs to count from 0 to TRACE_LEN (inclusive), so +1 for the range
  // Example: TRACE_LEN=4 ? needs to count 0,1,2,3,4 ? 3 bits
  localparam int unsigned PTR_W = $clog2(TRACE_LEN + 1);
  
  // Branch counter width: log2(MAX_BRANCHES + 1)
  // Needs to count from 0 to MAX_BRANCHES (inclusive)
  // Example: MAX_BRANCHES=2 ? needs to count 0,1,2 ? 2 bits
  localparam int unsigned BR_CNT_W = $clog2(MAX_BRANCHES + 1);

  // ========================================================================
  // Type Definitions
  // ========================================================================
  
  typedef enum logic [1:0] { 
    IDLE,   // Waiting for first instruction to start new trace
    ACCUM,  // Accumulating instructions into trace buffer
    COMMIT  // Writing completed trace to SRAM
  } state_t;

  // ========================================================================
  // State Registers
  // ========================================================================
  
  state_t state_q, state_d;
  
  // Trace buffer: stores up to TRACE_LEN instruction entries
  // NOTE: Each entry is 66 bits (32-bit PC + 32-bit inst + 2 control bits)
  // TODO (Riccardo): Storing only lower 32 bits of PC. Verify this is acceptable
  //                  for target workload address ranges. Full 64-bit PC is kept
  //                  in tag for matching accuracy.
  trace_entry_t trace_buf [TRACE_LEN-1:0];
  
  // Trace pointer
  logic [PTR_W-1:0] trace_ptr_q, trace_ptr_d;
  
  // Branch counter
  logic [BR_CNT_W-1:0] br_cnt_q, br_cnt_d;
  
  // SRAM write pointer: tracks next available SRAM address
  logic [TRACE_ADDRW-1:0] sram_wr_ptr_q, sram_wr_ptr_d;
  
  // Commit pipeline registers: hold trace data being written
  logic                   commit_valid_q, commit_valid_d;
  logic [TRACE_WIDTH-1:0] commit_data_q,  commit_data_d;  
  
  // Trace metadata: stored in tag SRAM for lookup/matching
  logic [PC_WIDTH_FULL-1:0] trace_start_pc_q,  trace_start_pc_d;  
  logic [GHR_WIDTH-1:0]     trace_start_ghr_q, trace_start_ghr_d;

  // ========================================================================
  // Output Assignments
  // ========================================================================
  
  // Always ready to accept new instructions 
  assign instr_i.ready = 1'b1;

  // SRAM write interface
  assign mem_req_o   = commit_valid_q;
  assign mem_we_o    = commit_valid_q;
  assign mem_addr_o  = sram_wr_ptr_q;
  assign mem_wdata_o = commit_data_q;
  assign mem_be_o    = {BE_WIDTH{1'b1}};  

  // Trace output 
  assign trace_valid_o = commit_valid_q;
  assign trace_data_o  = commit_data_q;

  // Tag SRAM interface
  assign tag_valid_o      = commit_valid_q;
  assign tag_start_pc_o   = trace_start_pc_q;   
  assign tag_start_ghr_o  = trace_start_ghr_q;
  assign tag_trace_len_o  = trace_ptr_q;        // Actual number of instructions in trace
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
  // Combinational Logic - State Machine
  // ========================================================================
  
  always_comb begin
    // Default: hold current state
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

        // ====================================================================
        // IDLE: Wait for first valid instruction to start new trace
        // ====================================================================
        IDLE: begin
          trace_ptr_d = '0;
          br_cnt_d    = '0;
          
          // Initialize trace buffer to zero 
          // TODO (Riccardo): Confirm if zeroing is acceptable or if NOPs preferred.
          for (int j = 0; j < TRACE_LEN; j++) begin
            trace_buf[j] = '0;
          end

          // Scan all instruction slots for first valid instruction
          for (int i = 0; i < SLOTS_PER_CYCLE; i++) begin
            if (instr_i.valid[i] && instr_i.ready && trace_ptr_d == 0) begin
              state_d = ACCUM;
              
              trace_buf[0] = '{pc:        instr_i.pc[i][PC_WIDTH_TRACE-1:0],
                               inst:      instr_i.inst[i], 
                               is_branch: instr_i.is_branch[i], 
                               taken:     instr_i.taken[i]};
              
              trace_ptr_d       = 1;
              trace_start_pc_d  = instr_i.pc[i];  
              trace_start_ghr_d = ghr_i;          
              
              if (instr_i.is_branch[i] && instr_i.taken[i])
                br_cnt_d = 1;
            end
          end
        end

        // ====================================================================
        // ACCUM: Accumulate instructions into trace with intra-cycle window
        // ====================================================================
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

              trace_buf[temp_ptr] = '{pc:        instr_i.pc[i][PC_WIDTH_TRACE-1:0],
                                      inst:      instr_i.inst[i],
                                      is_branch: instr_i.is_branch[i], 
                                      taken:     instr_i.taken[i]};
              temp_ptr++;
              
              // Check for taken control flow 
              if (instr_i.is_branch[i] && instr_i.taken[i]) begin
                temp_br_cnt++;
                window_closed = 1'b1;
              end
            end
          end
          
          trace_ptr_d = temp_ptr;
          br_cnt_d    = temp_br_cnt;
          
          // Check commit conditions
          if (trace_ptr_d >= TRACE_LEN || br_cnt_d >= MAX_BRANCHES) begin
            state_d = COMMIT;
          end
        end

        // ====================================================================
        // COMMIT: Write completed trace to SRAM
        // ====================================================================
        COMMIT: begin
          commit_valid_d = 1'b1;
          
          // Pack 4 instruction entries into 264-bit SRAM word
          // TODO (Riccardo): Confirm if zero-padding unused entries is acceptable,
          //                  or if explicit valid bits per entry are needed.
          commit_data_d = {
            trace_buf[3].pc, trace_buf[3].inst, trace_buf[3].is_branch, trace_buf[3].taken,
            trace_buf[2].pc, trace_buf[2].inst, trace_buf[2].is_branch, trace_buf[2].taken,
            trace_buf[1].pc, trace_buf[1].inst, trace_buf[1].is_branch, trace_buf[1].taken,
            trace_buf[0].pc, trace_buf[0].inst, trace_buf[0].is_branch, trace_buf[0].taken
          };
          
          // Increment SRAM write pointer for next trace
          sram_wr_ptr_d = sram_wr_ptr_q + 1;
          
          // Return to IDLE state to start next trace
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
               instr_i.pc[0][31:0], instr_i.pc[1][31:0], 
               instr_i.pc[2][31:0], instr_i.pc[3][31:0],
               instr_i.is_branch,
               instr_i.taken);
    end
  end

endmodule