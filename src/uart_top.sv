module uart_top #(
   parameter int unsigned COUNTER_WIDTH = 24,
   parameter int unsigned IMEM_BYTE_ADDR_WIDTH = 6, // 64 bytes of storage, or 16 four-byte words
   parameter int unsigned DMEM_BYTE_ADDR_WIDTH = 6) // 64 bytes of storage, or 16 four-byte words
(
   input  logic clk,
   // User Interface
   input  logic rst,
   input  logic rx_in,
   output logic tx_out,
   // CPU Interface
   output logic cpu_rst,
   // I-Memory Interface
   output logic imem_ctrl,
   output logic imem_wr_en,
   output logic [IMEM_BYTE_ADDR_WIDTH-3:0] imem_addr,
   output logic [3:0] imem_byte_en,
   output logic [31:0] imem_wr_data,
   // D-Memory Interface
   output logic dmem_ctrl,
   output logic dmem_rd_en,
   output logic [DMEM_BYTE_ADDR_WIDTH-3:0] dmem_addr,
   input  logic [31:0] dmem_rd_data);

   logic rx_ready, tx_empty, tx_error, tx_req;
   logic [IMEM_BYTE_ADDR_WIDTH-1:0] imem_byte_addr;
   logic [DMEM_BYTE_ADDR_WIDTH-1:0] dmem_byte_addr;
   logic [7:0] rx_data, tx_data;
   logic [COUNTER_WIDTH-1:0] cycles_per_bit;

   uart_rx #(
      .COUNTER_WIDTH(COUNTER_WIDTH))
   uart_rx0 (
      .clk(clk),
      .rst(rst),
      .uart_rx_in(rx_in),
      .data(rx_data),
      .cycles_per_bit(cycles_per_bit),
      .ready(rx_ready));

   uart_tx #(
      .COUNTER_WIDTH(COUNTER_WIDTH))
   uart_tx0 (
      .clk(clk),
      .rst(rst),
      .uart_tx_out(tx_out),
      .data(tx_data),
      .req(tx_req),
      .cycles_per_bit(cycles_per_bit),
      .empty(tx_empty),
      .error(tx_error));

   uart_ctrl #(
      .IMEM_BYTE_ADDR_WIDTH(IMEM_BYTE_ADDR_WIDTH),
      .DMEM_BYTE_ADDR_WIDTH(DMEM_BYTE_ADDR_WIDTH))
   uart_ctrl0 (
      .clk(clk),
      .rst(rst),
      .rx_ready(rx_ready),
      .tx_empty(tx_empty),
      .tx_error(tx_error),
      .cpu_rst(cpu_rst),
      .tx_req(tx_req),
      .imem_ctrl(imem_ctrl),
      .imem_wr_en(imem_wr_en),
      .imem_addr(imem_byte_addr),
      .dmem_ctrl(dmem_ctrl),
      .dmem_rd_en(dmem_rd_en),
      .dmem_addr(dmem_byte_addr));

   byte_to_word #(
      .BYTE_ADDR_WIDTH(IMEM_BYTE_ADDR_WIDTH))
   byte_to_word0 (
      .byte_addr_in(imem_byte_addr),
      .byte_data_in(rx_data),
      .word_addr_out(imem_addr),
      .word_byte_en_out(imem_byte_en),
      .word_data_out(imem_wr_data));
   
   word_to_byte #(
      .BYTE_ADDR_WIDTH(DMEM_BYTE_ADDR_WIDTH))
   word_to_byte0 (
      .byte_addr_in(dmem_byte_addr),
      .byte_data_out(tx_data),
      .word_addr_out(dmem_addr),
      .word_data_in(dmem_rd_data));
   
endmodule
