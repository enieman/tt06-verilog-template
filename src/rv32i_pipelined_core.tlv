\m5_TLV_version 1d --inlineGen --noDirectiveComments --noline --clkAlways --bestsv --debugSigsYosys: tl-x.org
\m5
   use(m5-1.0)
   
   // #################################################################
   // #                                                               #
   // #  Starting-Point Code for MEST Course Tiny Tapeout RISC-V CPU  #
   // #                                                               #
   // #################################################################
   
   // ========
   // Settings
   // ========
   
   //-------------------------------------------------------
   // Build Target Configuration
   //
   // To build within Makerchip for the FPGA or ASIC:
   //   o Use first line of file: \m5_TLV_version 1d --inlineGen --noDirectiveComments --noline --clkAlways --bestsv --debugSigsYosys: tl-x.org
   //   o set(MAKERCHIP, 0)
   //   o var(target, FPGA)  // or ASIC
   set(MAKERCHIP, 0)
   var(my_design, tt_um_enieman)
   var(target, ASIC)  /// FPGA or ASIC
   //-------------------------------------------------------
   
   // Input debouncing--not important for the CPU which has no inputs,but the setting is here for final projects based on the CPU.
   var(debounce_inputs, 0)         /// 1: Provide synchronization and debouncing on all input signals.
                                   /// 0: Don't provide synchronization and debouncing.
                                   /// m5_neq(m5_MAKERCHIP, 1): Debounce unless in Makerchip.

   // CPU configs
   var(num_regs, 16)  // 32 for full reg file.
   var(dmem_size, 8)  // Size of DMem, a power of 2.
   
   
   // ======================
   // Computed From Settings
   // ======================
   
   // If debouncing, a user's module is within a wrapper, so it has a different name.
   var(user_module_name, m5_if(m5_debounce_inputs, my_design, m5_my_design))
   var(debounce_cnt, m5_if_eq(m5_MAKERCHIP, 1, 8'h03, 8'hff))
   
   
   // ==================
   // Sum 1 to 9 Program
   // ==================
   
   TLV_fn(riscv_sum_prog, {
      ~assemble(['
         # Add 1,2,3,...,9 (in that order).
         #
         # Regs:
         #  x10 (a0): In: 0, Out: final sum
         #  x12 (a2): 10
         #  x13 (a3): 1..10
         #  x14 (a4): Sum
         #
         # External to function:
         reset:
            ADD x10, x0, x0             # Initialize r10 (a0) to 0.
         # Function:
            ADD x14, x10, x0            # Initialize sum register a4 with 0x0
            ADDI x12, x10, 10            # Store count of 10 in register a2.
            ADD x13, x10, x0            # Initialize intermediate sum register a3 with 0
         loop:
            ADD x14, x13, x14           # Incremental addition
            ADDI x13, x13, 1            # Increment count register by 1
            BLT x13, x12, loop          # If a3 is less than a2, branch to label named <loop>
         done:
            ADD x10, x14, x0            # Store final result to register a0 so that it can be read by main program
            SW x10, 4(x0)
            LW x15, 4(x0)
            JALR x1, 0(x0)
      '])
   })
   
\SV
   // Include Tiny Tapeout Lab.
   m4_include_lib(['https:/']['/raw.githubusercontent.com/os-fpga/Virtual-FPGA-Lab/35e36bd144fddd75495d4cbc01c4fc50ac5bde6f/tlv_lib/tiny_tapeout_lib.tlv'])  
   m4_include_lib(['https://raw.githubusercontent.com/efabless/chipcraft---mest-course/main/tlv_lib/risc-v_shell_lib.tlv'])

   m4_ifelse_block(m5_MAKERCHIP, 1, ['
   // Include UART Modules
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/synchronizer.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/neg_edge_detector.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/pos_edge_detector.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/shift_register.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/byte_to_word.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/word_to_byte.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/uart_rx.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/uart_tx.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/uart_ctrl.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/uart_top.sv)
   m4_sv_include_url(https://raw.githubusercontent.com/enieman/tt06-verilog-template/main/src/reg_file.sv)
   '])
\TLV cpu()
   
   m5+riscv_gen()
   m5+riscv_sum_prog()
   m5_define_hier(IMEM, m5_NUM_INSTRS)
   |cpu
      @0 // Instruction Fetch, PC Select
         $reset = *reset;
         $pc[31:0] =
            $reset             ? 32'h0000_0000 :
            >>1$reset          ? 32'h0000_0000 :
            >>3$valid_tgt_pc   ? >>3$tgt_pc :
            >>3$valid_load     ? >>3$inc_pc :
                                 >>1$inc_pc;
         *imem_rd_en = ! ($reset || >>1$reset);
         *imem_rd_addr = $pc[*IMEM_BYTE_ADDR_WIDTH-1:2];
      
      @1 // Instruction Decode, PC Increment
         $instr[31:0] = *imem_rd_data[31:0];
         $inc_pc[31:0] = $pc + 32'h4;
         
         // Instruction Fields
         $opcode[6:0] = $instr[6:0];
         $funct3[2:0] = $instr[14:12];
         $funct7[6:0] = $instr[31:25];
         $rd[4:0] = $instr[11:7];
         $rs1[4:0] = $instr[19:15];
         $rs2[4:0] = $instr[24:20];
         $imm[31:0] =
            $is_s_instr ? {{21{$instr[31]}}, $instr[30:25], $instr[11:7]} :
            $is_b_instr ? {{20{$instr[31]}}, $instr[7], $instr[30:25], $instr[11:8], 1'b0} :
            $is_u_instr ? {$instr[31:12], {12{1'b0}}} :
            $is_j_instr ? {{12{$instr[31]}}, $instr[19:12], $instr[20], $instr[30:25], $instr[24:21], 1'b0} :
            //default to I-type format for simplicity
                          {{21{$instr[31]}}, $instr[30:20]};
         
         // Instruction Set
         $is_lui   = $opcode == 7'b0110111;
         $is_auipc = $opcode == 7'b0010111;
         $is_jal   = $opcode == 7'b1101111;
         $is_jalr  = {$funct3, $opcode} == 10'b000_1100111;
         $is_beq   = {$funct3, $opcode} == 10'b000_1100011;
         $is_bne   = {$funct3, $opcode} == 10'b001_1100011;
         $is_blt   = {$funct3, $opcode} == 10'b100_1100011;
         $is_bge   = {$funct3, $opcode} == 10'b101_1100011;
         $is_bltu  = {$funct3, $opcode} == 10'b110_1100011;
         $is_bgeu  = {$funct3, $opcode} == 10'b111_1100011;
         $is_lb    = {$funct3, $opcode} == 10'b000_0000011;
         $is_lh    = {$funct3, $opcode} == 10'b001_0000011;
         $is_lw    = {$funct3, $opcode} == 10'b010_0000011;
         $is_lbu   = {$funct3, $opcode} == 10'b100_0000011;
         $is_lhu   = {$funct3, $opcode} == 10'b101_0000011;
         $is_sb    = {$funct3, $opcode} == 10'b000_0100011;
         $is_sh    = {$funct3, $opcode} == 10'b001_0100011;
         $is_sw    = {$funct3, $opcode} == 10'b010_0100011;
         $is_addi  = {$funct3, $opcode} == 10'b000_0010011;
         $is_slti  = {$funct3, $opcode} == 10'b010_0010011;
         $is_sltiu = {$funct3, $opcode} == 10'b011_0010011;
         $is_xori  = {$funct3, $opcode} == 10'b100_0010011;
         $is_ori   = {$funct3, $opcode} == 10'b110_0010011;
         $is_andi  = {$funct3, $opcode} == 10'b111_0010011;
         $is_slli  = {$funct7, $funct3, $opcode} == 17'b0000000_001_0010011;
         $is_srli  = {$funct7, $funct3, $opcode} == 17'b0000000_101_0010011;
         $is_srai  = {$funct7, $funct3, $opcode} == 17'b0100000_101_0010011;
         $is_add   = {$funct7, $funct3, $opcode} == 17'b0000000_000_0110011;
         $is_sub   = {$funct7, $funct3, $opcode} == 17'b0100000_000_0110011;
         $is_sll   = {$funct7, $funct3, $opcode} == 17'b0000000_001_0110011;
         $is_slt   = {$funct7, $funct3, $opcode} == 17'b0000000_010_0110011;
         $is_sltu  = {$funct7, $funct3, $opcode} == 17'b0000000_011_0110011;
         $is_xor   = {$funct7, $funct3, $opcode} == 17'b0000000_100_0110011;
         $is_srl   = {$funct7, $funct3, $opcode} == 17'b0000000_101_0110011;
         $is_sra   = {$funct7, $funct3, $opcode} == 17'b0100000_101_0110011;
         $is_or    = {$funct7, $funct3, $opcode} == 17'b0000000_110_0110011;
         $is_and   = {$funct7, $funct3, $opcode} == 17'b0000000_111_0110011;
         
         // Instruction Categories
         $is_load    = $is_lb | $is_lh | $is_lw | $is_lbu | $is_lhu;
         $is_store   = $is_sb | $is_sh | $is_sw;
         $is_jump    = $is_jal  | $is_jalr;
         $is_add_op  = $is_add  | $is_addi | $is_auipc | $is_jump | $is_load | $is_store;
         $is_and_op  = $is_and  | $is_andi;
         $is_or_op   = $is_or   | $is_ori;
         $is_sll_op  = $is_sll  | $is_slli;
         $is_slt_op  = $is_slt  | $is_slti  | $is_blt  | $is_bge;
         $is_sltu_op = $is_sltu | $is_sltiu | $is_bltu | $is_bgeu;
         $is_sra_op  = $is_sra  | $is_srai;
         $is_srl_op  = $is_srl  | $is_srli;
         $is_xor_op  = $is_xor  | $is_xori;
         
         // Instruction Types
         $is_r_instr = $is_add | $is_sub | $is_sll | $is_slt | $is_sltu | $is_xor | $is_srl | $is_sra | $is_or | $is_and;
         $is_i_instr = $is_jalr | $is_load | $is_addi | $is_slti | $is_sltiu | $is_xori | $is_ori | $is_andi | $is_slli | $is_srli | $is_srai;
         $is_s_instr = $is_store;
         $is_b_instr = $is_beq | $is_bne | $is_blt | $is_bge | $is_bltu | $is_bgeu;
         $is_u_instr = $is_lui | $is_auipc;
         $is_j_instr = $is_jal;
         
         // Validity
         $rd_valid    = !$reset & ($is_r_instr | $is_i_instr | $is_u_instr | $is_j_instr);
         $rs1_valid   = !$reset & ($is_r_instr | $is_i_instr | $is_s_instr | $is_b_instr);
         $rs2_valid   = !$reset & ($is_r_instr | $is_s_instr | $is_b_instr);
      
      @2 // Operand Selection
         $rf_rd_en1 = $rs1_valid;
         $rf_rd_en2 = $rs2_valid;
         $rf_rd_index1[4:0] = $rs1;
         $rf_rd_index2[4:0] = $rs2;
         
         $src1_value[31:0] = (>>1$rf_wr_en && (>>1$rd == $rs1)) ? >>1$rf_wr_data : $rf_rd_data1;
         $src2_value[31:0] = (>>1$rf_wr_en && (>>1$rd == $rs2)) ? >>1$rf_wr_data : $rf_rd_data2;
         
         $alu_op1[31:0] =
            $is_auipc | $is_jump ? $pc :
                                   $src1_value;
         $alu_op2[31:0] =
            $is_r_instr || $is_b_instr ? $src2_value :
            $is_jump                   ? 32'h0000_0004 :
                                         $imm;
         
         $tgt_pc_op1[31:0] = $is_jalr ? $src1_value : $pc;
         $tgt_pc_op2[31:0] = $imm;
      
      @3 // Execute, Register Write
         $valid = !$reset && !(>>1$valid_taken_br || >>2$valid_taken_br || >>1$valid_load || >>2$valid_load || >>1$valid_jump || >>2$valid_jump);
         $valid_load  = $is_load  && $valid;
         $valid_store = $is_store && $valid;
         $valid_jump  = $is_jump  && $valid;
         $valid_taken_br =
            !$valid  ? 1'b0 :
            $is_beq  ? $alu_op1 == $alu_op2 :
            $is_bne  ? $alu_op1 != $alu_op2 :
            $is_bltu ? $result[0] :
            $is_bgeu ? !$result[0] :
            $is_blt  ? $result[0] :
            $is_bge  ? !$result[0] :
                       1'b0;
         $valid_tgt_pc = $valid_taken_br | $valid_jump;
         
         // Target PC Adder
         $tgt_pc[31:0] = $tgt_pc_op1 + $tgt_pc_op2;
         
         // ALU
         $sra_result[63:0] = { {32{$alu_op1[31]}}, $alu_op1} >> $alu_op2[4:0];
         $sltu_result[31:0] = $alu_op1 < $alu_op2 ? 32'h0000_0001 : 32'h0000_0000;
         $result[31:0] =
            $is_add_op  ? $alu_op1 + $alu_op2 :
            $is_and_op  ? $alu_op1 & $alu_op2 :
            $is_lui     ? {$alu_op2[31:12], 12'h000} :
            $is_or_op   ? $alu_op1 | $alu_op2 :
            $is_sll_op  ? $alu_op1 << $alu_op2[4:0] :
            $is_slt_op  ? (($alu_op1[31] == $alu_op2[31]) ? $sltu_result : ($alu_op1[31] == 1'b1 ? 32'h0000_0001 : 32'h0000_0000)) :
            $is_sltu_op ? $sltu_result :
            $is_sra_op  ? $sra_result[31:0] :
            $is_srl_op  ? $alu_op1 >> $alu_op2[4:0] :
            $is_sub     ? $alu_op1 - $alu_op2 :
            $is_xor_op  ? $alu_op1 ^ $alu_op2 :
                          32'hxxxx_xxxx;
         
         // Register Write
         $rf_wr_en = ($valid && $rd_valid && ($rd != 5'h00) && !$valid_load) || >>2$valid_load;
         $rf_wr_index[4:0] = $valid ? $rd : >>2$rd;
         $rf_wr_data[31:0] = $valid ? $result : >>2$ld_data;
      
      @4 // Data Memory Write
         *dmem_rd_en = $valid_load;
         *dmem_wr_en = $valid_store;
         *dmem_addr = $result[*DMEM_BYTE_ADDR_WIDTH-1:2];
         *dmem_wr_byte_en = 4'b1111; // Just implement LW/SW for now
         *dmem_wr_data = $src2_value;
      
      @5 // Data Memory Read
         $ld_data[31:0] = *dmem_rd_data;
      
      // Note that pipesignals assigned here can be found under /fpga_pins/fpga.
      
      
      
   
   // Assert these to end simulation (before Makerchip cycle limit).
   // Note, for Makerchip simulation these are passed in uo_out to top-level module's passed/failed signals.
   //*passed = *top.cyc_cnt > 40;
   //*passed = (|cpu/xreg[15]>>5$value == (1+2+3+4+5+6+7+8+9)) && (*top.cyc_cnt > 65);
   //*passed = |cpu/xreg[15]>>5$value == (1+2+3+4+5+6+7+8+9);
   //*failed = 1'b0;
   
   // Connect Tiny Tapeout outputs. Note that uio_ outputs are not available in the Tiny-Tapeout-3-based FPGA boards.
   // *uo_out = {6'b0, *failed, *passed};
   m5_if_neq(m5_target, FPGA, ['*uio_out = 8'b0;'])
   m5_if_neq(m5_target, FPGA, ['*uio_oe = 8'b0;'])
   
   // Macro instantiations to be uncommented when instructed for:
   //  o instruction memory
   //  o register file
   //  o data memory
   //  o CPU visualization
   |cpu
      // m4+imem(@1)    // Args: (read stage)
      m4+rf(@2, @3)  // Args: (read stage, write stage) - if equal, no register bypass is required
      // m4+dmem(@4)    // Args: (read/write stage)

   // m4+cpu_viz(@4)    // For visualisation, argument should be at least equal to the last stage of CPU logic. @4 would work for all labs.

\SV

m4_ifelse_block(m5_MAKERCHIP, 1, ['
// ================================================
// A simple Makerchip Verilog test bench driving random stimulus.
// Modify the module contents to your needs.
// ================================================

module top(input logic clk, input logic reset, input logic [31:0] cyc_cnt, output logic passed, output logic failed);
   // Tiny tapeout I/O signals.
   logic [7:0] ui_in, uo_out;
   m5_if_neq(m5_target, FPGA, ['logic [7:0] uio_in,  uio_out, uio_oe;'])
   // assign ui_in = 8'b0;
   m5_if_neq(m5_target, FPGA, ['assign uio_in = 8'b0;'])
   logic ena = 1'b0;
   logic rst_n = ! reset;
   
   // Testbench Variables
   localparam int unsigned CYCLES_PER_BIT = 8;
   bit [31:0] instructions [16];
   bit [7:0] data [16];
   
   // Instantiate the Tiny Tapeout module.
   m5_user_module_name tt(.*);
   
   // Passed/failed to control Makerchip simulation, passed from Tiny Tapeout module's uo_out pins.
   assign passed = (data[4] == 1+2+3+4+5+6+7+8+9) ? 1'b1 : 1'b0;
   assign failed = ~passed;
endmodule

'])

// Provide a wrapper module to debounce input signals if requested.
m5_if(m5_debounce_inputs, ['m5_tt_top(m5_my_design)'])
\SV



// =======================
// The Tiny Tapeout module
// =======================

module m5_user_module_name (
    input  wire [7:0] ui_in,    // Dedicated inputs - connected to the input switches
    output wire [7:0] uo_out,   // Dedicated outputs - connected to the 7 segment display
    m5_if_eq(m5_target, FPGA, ['/']['*'])   // The FPGA is based on TinyTapeout 3 which has no bidirectional I/Os (vs. TT6 for the ASIC).
    input  wire [7:0] uio_in,   // IOs: Bidirectional Input path
    output wire [7:0] uio_out,  // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,   // IOs: Bidirectional Enable path (active high: 0=input, 1=output)
    m5_if_eq(m5_target, FPGA, ['*']['/'])
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
   
   // Parameters for Memory Sizing and UART Baud
   localparam int unsigned COUNTER_WIDTH = 16;       // Width of the clock counters in the UART RX and TX modules; at 50MHz, 16 bits should allow baud as low as 763
   localparam int unsigned IMEM_BYTE_ADDR_WIDTH = 6; // 64 bytes / 16 words of I-Memory
   localparam int unsigned DMEM_BYTE_ADDR_WIDTH = 4; // 16 bytes /  4 words of D-Memory
   
   // CPU Reset
   wire reset;
   
   // User Interface
   wire rst = ! rst_n | ui_in[7]; // Provide a dedicated button input for RESET
   wire rx_in = ui_in[2];         // Should be wired to Pin 2 of the USBUART Pmod (data from host to Pmod)
   wire tx_out;
   assign uo_out[7] = rst;        // Feedback of RST button, intended to use with LED
   assign uo_out[6] = reset;      // Feedback of CPU reset, indicates if UART controller is in write mode (reset = 1) or read mode (reset = 0)
   assign uo_out[5] = ~rx_in;     // Feedback of RX line, intended to use with LED
   assign uo_out[4] = ~tx_out;    // Feedback of TX line, intended to use with LED
   assign uo_out[3] = 1'b0;       // Unused
   assign uo_out[2] = tx_out;     // Should be wired to Pin 3 of the USBUART Pmod (data from Pmod to host)
   assign uo_out[1] = 1'b0;       // Unused
   assign uo_out[0] = 1'b0;       // Unused
   
   // I-Memory Interface
   logic uart_imem_ctrl;
   logic imem_rd_en, uart_imem_wr_en;
   logic [IMEM_BYTE_ADDR_WIDTH-3:0] imem_rd_addr, uart_imem_addr;
   logic [3:0] uart_imem_byte_en;
   logic [31:0] imem_rd_data, uart_imem_wr_data;
   
   // D-Memory Interface
   logic dmem_rd_en, dmem_wr_en, uart_dmem_rd_en;
   logic [DMEM_BYTE_ADDR_WIDTH-3:0] dmem_addr, uart_dmem_addr;
   logic [3:0] dmem_wr_byte_en;
   logic [31:0] dmem_wr_data, dmem_rd_data, uart_dmem_rd_data;
   
   // UART Module
   uart_top #(
      .COUNTER_WIDTH(COUNTER_WIDTH),
      .IMEM_BYTE_ADDR_WIDTH(IMEM_BYTE_ADDR_WIDTH),
      .DMEM_BYTE_ADDR_WIDTH(DMEM_BYTE_ADDR_WIDTH))
   uart_top0 (
      .clk(clk),
      .rst(rst),
      .rx_in(rx_in),
      .tx_out(tx_out),
      .cpu_rst(reset),
      .imem_ctrl(uart_imem_ctrl),
      .imem_wr_en(uart_imem_wr_en),
      .imem_addr(uart_imem_addr),
      .imem_byte_en(uart_imem_byte_en),
      .imem_wr_data(uart_imem_wr_data),
      .dmem_ctrl(),
      .dmem_rd_en(uart_dmem_rd_en),
      .dmem_addr(uart_dmem_addr),
      .dmem_rd_data(uart_dmem_rd_data));
   
   // I-Memory
   reg_file #(
      .BYTE_ADDR_WIDTH(IMEM_BYTE_ADDR_WIDTH))
   imem0 (
      .clk(clk),
      .rst(rst),
      .rd_en0(uart_imem_ctrl ? 1'b0 : imem_rd_en),
      .rd_addr0(uart_imem_ctrl ? '0 : imem_rd_addr),
      .rd_data0(imem_rd_data),
      .rd_en1(1'b0),
      .rd_addr1('0),
      .rd_data1(),
      .wr_en(uart_imem_ctrl ? uart_imem_wr_en : 1'b0),
      .wr_addr(uart_imem_ctrl ? uart_imem_addr : '0),
      .byte_en(uart_imem_ctrl ? uart_imem_byte_en : '0),
      .wr_data(uart_imem_ctrl ? uart_imem_wr_data : '0));
   
   // D-Memory
   reg_file #(
      .BYTE_ADDR_WIDTH(DMEM_BYTE_ADDR_WIDTH))
   dmem0 (
      .clk(clk),
      .rst(rst),
      .rd_en0(dmem_rd_en),
      .rd_addr0(dmem_addr),
      .rd_data0(dmem_rd_data),
      .rd_en1(uart_dmem_rd_en),
      .rd_addr1(uart_dmem_addr),
      .rd_data1(uart_dmem_rd_data),
      .wr_en(dmem_wr_en),
      .wr_addr(dmem_addr),
      .byte_en(dmem_wr_byte_en),
      .wr_data(dmem_wr_data));
   
\TLV
   /* verilator lint_off UNOPTFLAT */
   // Connect Tiny Tapeout I/Os to Virtual FPGA Lab.
   m5+tt_connections()
   
   // Instantiate the Virtual FPGA Lab.
   m5+board(/top, /fpga, 7, $, , cpu)
   // Label the switch inputs [0..7] (1..8 on the physical switch panel) (top-to-bottom).
   m5+tt_input_labels_viz(['"UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED"'])

\SV
endmodule
