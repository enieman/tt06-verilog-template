module uart_ctrl #(
   parameter int unsigned IMEM_BYTE_ADDR_WIDTH = 6, // 64 bytes of storage, or 16 four-byte words
   parameter int unsigned DMEM_BYTE_ADDR_WIDTH = 6) // 64 bytes of storage, or 16 four-byte words
(
   input  logic clk,
   input  logic rst,
   input  logic rx_ready,
   input  logic tx_empty,
   input  logic tx_error,
   output logic cpu_rst,
   output logic tx_req,
   output logic imem_ctrl,
   output logic imem_wr_en,
   output logic [IMEM_BYTE_ADDR_WIDTH-1:0] imem_addr,
   output logic dmem_ctrl,
   output logic dmem_rd_en,
   output logic [DMEM_BYTE_ADDR_WIDTH-1:0] dmem_addr);

   localparam int unsigned NUM_IMEM_BYTES = 2**IMEM_BYTE_ADDR_WIDTH;
   localparam int unsigned NUM_DMEM_BYTES = 2**DMEM_BYTE_ADDR_WIDTH;

   // Declare intermediate wires
   logic rd_complete, wr_data_ready, all_imem_written;
   enum logic [1:0] {
      STATE_RESET      = 2'b00,
      STATE_DATA_WRITE = 2'b01,
      STATE_IDLE       = 2'b10,
      STATE_DATA_READ  = 2'b11
   } state;

   // Assign intermediate wires
   /* verilator lint_off WIDTHEXPAND */
   assign rd_complete = ((state == STATE_DATA_READ) && (dmem_addr == NUM_DMEM_BYTES-1) && dmem_rd_en) ? 1'b1 : 1'b0;
   assign all_imem_written = ((state == STATE_DATA_WRITE) && (imem_addr == NUM_IMEM_BYTES-1) && imem_wr_en) ? 1'b1 : 1'b0;
   /* verilator lint_on WIDTHEXPAND */
   pos_edge_detector wr_data_ready_detect (
      .clk(clk),
      .rst(rst),
      .signal_in(rx_ready),
      .edge_detected(wr_data_ready)
   );

   // State machine logic
   always_ff @(posedge clk) begin
      if (rst) state <= STATE_RESET;
      else begin
         case (state)
            STATE_RESET:      if (!rst)             state <= STATE_DATA_WRITE;
            STATE_DATA_WRITE: if (all_imem_written) state <= STATE_IDLE;
            STATE_IDLE:       if (wr_data_ready)    state <= STATE_DATA_READ;
            STATE_DATA_READ:  if (rd_complete)      state <= STATE_IDLE;
         endcase
      end // else
   end // always_ff

   // I-Memory Address Counter
   always_ff @(posedge clk) begin
      if (rst) imem_addr <= '0;
      else if (state == STATE_RESET) imem_addr <= '0;
      else if (state == STATE_IDLE)  imem_addr <= '0;
      else if (state == STATE_DATA_WRITE && imem_wr_en) imem_addr <= imem_addr + 1;
   end //always_ff

   // D-Memory Address Counter
   always_ff @(posedge clk) begin
      if (rst) dmem_addr <= '0;
      else if (state == STATE_RESET) dmem_addr <= '0;
      else if (state == STATE_IDLE)  dmem_addr <= '0;
      else if (state == STATE_DATA_READ && dmem_rd_en) dmem_addr <= dmem_addr + 1;
   end //always_ff

   // Connect outputs
   assign cpu_rst    = (state == STATE_RESET || state == STATE_DATA_WRITE || rst) ? 1'b1 : 1'b0;
   assign imem_ctrl  = cpu_rst;
   assign dmem_ctrl  = cpu_rst;
   assign dmem_rd_en = (state == STATE_DATA_READ && tx_empty) ? 1'b1 : 1'b0;
   assign imem_wr_en = (state == STATE_DATA_WRITE && wr_data_ready) ? 1'b1 : 1'b0;
   assign tx_req     = dmem_rd_en;

endmodule
