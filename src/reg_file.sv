module reg_file #(
   parameter int unsigned BYTE_ADDR_WIDTH = 6)     // 64 bytes (16 32-bit words)
(
   input  logic clk,
   input  logic rst,
   // Read Channel 0
   input  logic rd_en0,
   input  logic [BYTE_ADDR_WIDTH-3:0] rd_addr0,
   output logic [31:0] rd_data0,
   // Read Channel 1
   input  logic rd_en1,
   input  logic [BYTE_ADDR_WIDTH-3:0] rd_addr1,
   output logic [31:0] rd_data1,
   // Write Channel
   input  logic wr_en,
   input  logic [BYTE_ADDR_WIDTH-3:0] wr_addr,
   input  logic [3:0] byte_en,
   input  logic [31:0] wr_data);
   
   localparam int unsigned NUM_MEM_BYTES  = 2**BYTE_ADDR_WIDTH;
   
   logic [7:0] register [NUM_MEM_BYTES];
   
   // Register File
   always_ff @(posedge clk) begin
      if (rst)
         for (int unsigned i = 0; i < NUM_MEM_BYTES; i++) register[i] = 8'h00;
      else if (wr_en) begin
         if (byte_en[3]) register[{wr_addr, 2'b11}] <= wr_data[31:24];
         if (byte_en[2]) register[{wr_addr, 2'b10}] <= wr_data[23:16];
         if (byte_en[1]) register[{wr_addr, 2'b01}] <= wr_data[15:8];
         if (byte_en[0]) register[{wr_addr, 2'b00}] <= wr_data[7:0];
      end
   end
   
   // Read Buffers
   always_ff @(posedge clk) begin
      if (rst) begin
         rd_data0 <= '0;
         rd_data1 <= '0;
      end
      else begin
         if (rd_en0) rd_data0 <= {
            register[{rd_addr0, 2'b11}],
            register[{rd_addr0, 2'b10}],
            register[{rd_addr0, 2'b01}],
            register[{rd_addr0, 2'b00}]};
         if (rd_en1) rd_data1 <= {
            register[{rd_addr1, 2'b11}],
            register[{rd_addr1, 2'b10}],
            register[{rd_addr1, 2'b01}],
            register[{rd_addr1, 2'b00}]};
      end
   end
   
   /*
   // Read Data
   assign rd_data0 = {
      register[{rd_addr0, 2'b11}],
      register[{rd_addr0, 2'b10}],
      register[{rd_addr0, 2'b01}],
      register[{rd_addr0, 2'b00}]};
   assign rd_data1 = {
      register[{rd_addr1, 2'b11}],
      register[{rd_addr1, 2'b10}],
      register[{rd_addr1, 2'b01}],
      register[{rd_addr1, 2'b00}]};
   */

endmodule
