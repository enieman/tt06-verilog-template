module mem_rf #(
   parameter int unsigned MEM_BYTE_ADDR_WIDTH = 6) // 64 bytes of storage, or 16 four-byte words
(
   input  logic clk,
   input  logic rst,
   // UART Memory Interface
   input  logic umem_ctrl,
   input  logic umem_rd_en,
   input  logic umem_wr_en,
   input  logic [MEM_BYTE_ADDR_WIDTH-1:0] umem_addr,
   input  logic [7:0] umem_wr_data,
   output logic [7:0] umem_rd_data,
   // CPU Memory Interface
   input  logic cpu_mem_rd_en,
   input  logic cpu_mem_wr_en,
   input  logic [MEM_BYTE_ADDR_WIDTH-3:0] cpu_mem_addr, // Two fewer bits than umem since addressing 4 byte values
   input  logic [3:0] cpu_mem_wr_byte_en,
   input  logic [31:0] cpu_mem_wr_data,
   output logic [31:0] cpu_mem_rd_data);

   localparam int unsigned NUM_MEM_BYTES = 2**MEM_BYTE_ADDR_WIDTH;

   logic [7:0] reg_file [NUM_MEM_BYTES];

   always_ff @(posedge clk) begin
      if (rst) begin
         for (int unsigned i = 0; i < NUM_MEM_BYTES; i++) reg_file[i] = 8'h00;
      end // if
      else if (umem_ctrl && umem_wr_en)
         reg_file[umem_addr] <= umem_wr_data;
      else if (!umem_ctrl && cpu_mem_wr_en) begin
         if (cpu_mem_wr_byte_en[3]) reg_file[{cpu_mem_addr, 2'b11}] <= cpu_mem_wr_data[31:24];
         if (cpu_mem_wr_byte_en[2]) reg_file[{cpu_mem_addr, 2'b10}] <= cpu_mem_wr_data[23:16];
         if (cpu_mem_wr_byte_en[1]) reg_file[{cpu_mem_addr, 2'b01}] <= cpu_mem_wr_data[15:8];
         if (cpu_mem_wr_byte_en[0]) reg_file[{cpu_mem_addr, 2'b00}] <= cpu_mem_wr_data[7:0];
      end
   end // always_ff

   assign umem_rd_data = reg_file[umem_addr];
   assign cpu_mem_rd_data =
      {reg_file[{cpu_mem_addr, 2'b11}],
       reg_file[{cpu_mem_addr, 2'b10}],
       reg_file[{cpu_mem_addr, 2'b01}],
       reg_file[{cpu_mem_addr, 2'b00}]};

endmodule
