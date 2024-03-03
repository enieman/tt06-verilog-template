module byte_to_word #(
   parameter int unsigned BYTE_ADDR_WIDTH = 6) // 64 bytes access
(
   input  logic [BYTE_ADDR_WIDTH-1:0] byte_addr_in,
   input  logic [7:0] byte_data_in,
   output logic [BYTE_ADDR_WIDTH-3:0] word_addr_out,
   output logic [3:0] word_byte_en_out,
   output logic [31:0] word_data_out);
   
   always_comb begin
      word_addr_out = byte_addr_in[BYTE_ADDR_WIDTH-1:2];
      case (byte_addr_in[1:0])
         2'b00: begin
                word_byte_en_out = 4'b0001;
                word_data_out = {24'h00_0000, byte_data_in};
                end
         2'b01: begin
                word_byte_en_out = 4'b0010;
                word_data_out = {16'h0000, byte_data_in, 8'h00};
                end
         2'b10: begin
                word_byte_en_out = 4'b0100;
                word_data_out = {8'h00, byte_data_in, 16'h0000};
                end
         2'b11: begin
                word_byte_en_out = 4'b1000;
                word_data_out = {byte_data_in, 24'h00_0000};
                end
      endcase
   end
   
endmodule
