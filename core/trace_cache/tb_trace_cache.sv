`timescale 1ns/1ps

module tb_trace_cache;

  import trace_cache_pkg::*;

  // Clock and reset
  logic clk;
  logic rst_n;

  // Generate clock (10ns period = 100MHz)
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Generate reset
  initial begin
    rst_n = 0;
    #20 rst_n = 1;
  end

  // Instruction interface
  tracebuilder_instr_if instr_if (
    .clk_i(clk),
    .rst_ni(rst_n)
  );

  // GHR and flush signals
  logic [GHR_WIDTH-1:0] ghr;
  logic flush;

  // Outputs (not used in this test)
  logic trace_valid;
  logic [TRACE_WIDTH-1:0] trace_data;
  logic mem_req, mem_we;
  logic [TRACE_ADDRW-1:0] mem_addr;
  logic [TRACE_WIDTH-1:0] mem_wdata;
  logic [BE_WIDTH-1:0] mem_be;
  logic tag_valid;
  logic [PC_WIDTH-1:0] tag_start_pc;
  logic [GHR_WIDTH-1:0] tag_start_ghr;
  logic [3:0] tag_trace_len;
  logic [TRACE_ADDRW-1:0] tag_sram_addr;

  // Instantiate trace_builder
  trace_builder dut (
    .clk_i(clk),
    .rst_ni(rst_n),
    .instr_i(instr_if),
    .ghr_i(ghr),
    .flush_i(flush),
    .trace_valid_o(trace_valid),
    .trace_data_o(trace_data),
    .mem_req_o(mem_req),
    .mem_we_o(mem_we),
    .mem_addr_o(mem_addr),
    .mem_wdata_o(mem_wdata),
    .mem_be_o(mem_be),
    .tag_valid_o(tag_valid),
    .tag_start_pc_o(tag_start_pc),
    .tag_start_ghr_o(tag_start_ghr),
    .tag_trace_len_o(tag_trace_len),
    .tag_sram_addr_o(tag_sram_addr)
  );

  // Test stimulus
  initial begin
    // Initialize
    ghr = 8'h00;
    flush = 0;
    instr_if.valid = 4'b0000;
    instr_if.pc = '{default: 64'h0};
    instr_if.inst = '{default: 32'h0};
    instr_if.is_branch = 4'b0000;
    instr_if.taken = 4'b0000;
    instr_if.target = '{default: 64'h0};

    // Wait for reset
    @(posedge rst_n);
    @(posedge clk);

    // Test 1: Send single instruction
    $display("\n=== Test 1: Single Instruction ===");
    @(posedge clk);
    instr_if.valid[0] = 1'b1;
    instr_if.pc[0] = 64'h0000_0000_0000_1000;
    instr_if.inst[0] = 32'h00208093;  // ADD instruction
    instr_if.is_branch[0] = 1'b0;
    instr_if.taken[0] = 1'b0;

    @(posedge clk);
    instr_if.valid = 4'b0000;

    // Test 2: Send 4 instructions (fill trace)
    $display("\n=== Test 2: Four Instructions ===");
    repeat(3) @(posedge clk);
    
    instr_if.valid = 4'b1111;
    instr_if.pc[0] = 64'h0000_0000_0000_2000;
    instr_if.pc[1] = 64'h0000_0000_0000_2004;
    instr_if.pc[2] = 64'h0000_0000_0000_2008;
    instr_if.pc[3] = 64'h0000_0000_0000_200C;
    
    instr_if.inst[0] = 32'h00208093;  // ADD
    instr_if.inst[1] = 32'h00310113;  // ADD
    instr_if.inst[2] = 32'h00418193;  // ADD
    instr_if.inst[3] = 32'h00520213;  // ADD
    
    instr_if.is_branch = 4'b0000;
    instr_if.taken = 4'b0000;

    @(posedge clk);
    instr_if.valid = 4'b0000;

    // Test 3: Instruction with taken branch
    $display("\n=== Test 3: Taken Branch ===");
    repeat(3) @(posedge clk);
    
    instr_if.valid = 4'b0011;
    instr_if.pc[0] = 64'h0000_0000_0000_3000;
    instr_if.pc[1] = 64'h0000_0000_0000_3004;
    
    instr_if.inst[0] = 32'h00208093;  // ADD
    instr_if.inst[1] = 32'h00208063;  // BEQ (branch)
    
    instr_if.is_branch[0] = 1'b0;
    instr_if.is_branch[1] = 1'b1;  // Branch instruction
    instr_if.taken[1] = 1'b1;      // Taken
    instr_if.target[1] = 64'h0000_0000_0000_5000;  // Target address

    @(posedge clk);
    instr_if.valid = 4'b0000;

    // Wait and finish
    repeat(10) @(posedge clk);
    $display("\n=== Simulation Complete ===");
    $finish;
  end

  // Monitor outputs
  always @(posedge clk) begin
    if (trace_valid) begin
      $display("[%0t] TRACE COMMITTED: addr=%h", $time, mem_addr);
      $display("  Base PC: %h", tag_start_pc);
      $display("  GHR: %h", tag_start_ghr);
    end
  end

endmodule