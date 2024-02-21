module synchronizer (
   input  logic clk,    //your local clock
   input  logic async,  //unsynchronized signal
   output logic sync); //synchronized signal

   // Create a signal buffer
   logic [1:0] buff;

   always_ff @ (posedge clk) 
      buff <= {buff[0], async};

   assign sync = buff[1];

endmodule
