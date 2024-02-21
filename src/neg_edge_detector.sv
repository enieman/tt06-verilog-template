module neg_edge_detector (
   input  logic clk,
   input  logic rst,
   input  logic signal_in,
   output logic edge_detected);

   logic register;

   always_ff @(posedge clk) begin
      if (rst) register <= 1;
      else register <= signal_in;
   end

   assign edge_detected = register & ~signal_in;

endmodule
