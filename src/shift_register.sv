module shift_register #(
   parameter int unsigned NUM_BITS = 8,
   parameter int unsigned RST_VALUE = 0)
(
   input  logic clk,
   input  logic rst,
   input  logic serial_in,
   input  logic shift_enable,
   input  logic [NUM_BITS-1:0] parallel_in,
   input  logic load_enable,
   output logic serial_out,
   output logic [NUM_BITS-1:0] parallel_out);

   logic [NUM_BITS-1:0] register;

   always_ff @(posedge clk) begin
      if (rst) register <= RST_VALUE[NUM_BITS-1:0];
      else if (load_enable) register <= parallel_in;
      else if (shift_enable) begin
         for (int unsigned i = 0; i < NUM_BITS-1; i++) register[i] <= register[i+1];
         register[NUM_BITS-1] <= serial_in;
      end
      else register <= register;
   end

   always_comb begin
      parallel_out = register;
      serial_out = register[0];
   end

endmodule
