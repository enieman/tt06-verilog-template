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
   input  logic imem_rd_en,
   input  logic [IMEM_BYTE_ADDR_WIDTH-3:0] imem_rd_addr,
   output logic [31:0] imem_rd_data,
   input  logic dmem_rd_en,
   input  logic dmem_wr_en,
   input  logic [DMEM_BYTE_ADDR_WIDTH-3:0] dmem_addr,
   input  logic [3:0] dmem_wr_byte_en,
   input  logic [31:0] dmem_wr_data,
   output logic [31:0] dmem_rd_data);

   logic rx_ready, tx_empty, tx_error, tx_req, imem_uart_ctrl, imem_uart_wr_en, dmem_uart_ctrl, dmem_uart_rd_en;
   logic [IMEM_BYTE_ADDR_WIDTH-1:0] imem_uart_addr;
   logic [DMEM_BYTE_ADDR_WIDTH-1:0] dmem_uart_addr;
   logic [7:0] umem_rd_data, umem_wr_data;
   logic [COUNTER_WIDTH-1:0] cycles_per_bit;

   uart_rx #(
      .COUNTER_WIDTH(COUNTER_WIDTH))
   uart_rx0 (
      .clk(clk),
      .rst(rst),
      .uart_rx_in(rx_in),
      .data(umem_wr_data),
      .cycles_per_bit(cycles_per_bit),
      .ready(rx_ready));

   uart_tx #(
      .COUNTER_WIDTH(COUNTER_WIDTH))
   uart_tx0 (
      .clk(clk),
      .rst(rst),
      .uart_tx_out(tx_out),
      .data(umem_rd_data),
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
      .imem_ctrl(imem_uart_ctrl),
      .imem_wr_en(imem_uart_wr_en),
      .imem_addr(imem_uart_addr),
      .dmem_ctrl(dmem_uart_ctrl),
      .dmem_rd_en(dmem_uart_rd_en),
      .dmem_addr(dmem_uart_addr));

   mem_rf #(
      .MEM_BYTE_ADDR_WIDTH(IMEM_BYTE_ADDR_WIDTH))
   imem0 (
      .clk(clk),
      .rst(rst),
      .umem_ctrl(imem_uart_ctrl),
      .umem_rd_en(0),
      .umem_wr_en(imem_uart_wr_en),
      .umem_addr(imem_uart_addr),
      .umem_wr_data(umem_wr_data),
      .umem_rd_data(),
      .cpu_mem_rd_en(imem_rd_en),
      .cpu_mem_wr_en(0),
      .cpu_mem_addr(imem_rd_addr),
      .cpu_mem_wr_byte_en('0),
      .cpu_mem_wr_data('0),
      .cpu_mem_rd_data(imem_rd_data));

   mem_rf #(
      .MEM_BYTE_ADDR_WIDTH(DMEM_BYTE_ADDR_WIDTH))
   dmem0 (
      .clk(clk),
      .rst(rst),
      .umem_ctrl(dmem_uart_ctrl),
      .umem_rd_en(dmem_uart_rd_en),
      .umem_wr_en(0),
      .umem_addr(dmem_uart_addr),
      .umem_wr_data('0),
      .umem_rd_data(umem_rd_data),
      .cpu_mem_rd_en(dmem_rd_en),
      .cpu_mem_wr_en(dmem_wr_en),
      .cpu_mem_addr(dmem_addr),
      .cpu_mem_wr_byte_en(dmem_wr_byte_en),
      .cpu_mem_wr_data(dmem_wr_data),
      .cpu_mem_rd_data(dmem_rd_data));
endmodule
