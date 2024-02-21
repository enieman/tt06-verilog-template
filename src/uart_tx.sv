module uart_tx #(
   parameter int unsigned COUNTER_WIDTH = 24)
(
   input  logic clk,
   input  logic rst,
   output logic uart_tx_out,
   input  logic [7:0] data,
   input  logic req,
   input  logic [COUNTER_WIDTH-1:0] cycles_per_bit,
   output logic empty,
   output logic error);

   // Declare intermediate wires
   logic full_bit, idle_state;
   logic [COUNTER_WIDTH-1:0] counter_val;
   enum logic [3:0] {
      STATE_IDLE  = 4'h0,
      STATE_START = 4'h1,
      STATE_D0    = 4'h2,
      STATE_D1    = 4'h3,
      STATE_D2    = 4'h4,
      STATE_D3    = 4'h5,
      STATE_D4    = 4'h6,
      STATE_D5    = 4'h7,
      STATE_D6    = 4'h8,
      STATE_D7    = 4'h9,
      STATE_STOP  = 4'hA
   } state;

   assign idle_state = (state == STATE_IDLE) ? 1'b1:1'b0;

   // Connect shift register
   shift_register #(
      .NUM_BITS(9),
      .RST_VALUE(1))
   shift_reg(
      .clk(clk),
      .rst(rst),
      .serial_in(1'b1),
      .shift_enable(full_bit),
      .parallel_in({data, 1'b0}),
      .load_enable(idle_state & req),
      .serial_out(uart_tx_out),
      .parallel_out());

   // Implement timer
   assign full_bit = (counter_val >= cycles_per_bit) ? 1'b1 : 1'b0;
   always_ff @(posedge clk) begin
      if (full_bit || idle_state || rst) counter_val <= '0;
      else counter_val <= counter_val + 1;
   end

   // State machine logic
   always_ff @(posedge clk) begin
      if (rst || (state > STATE_STOP)) state <= STATE_IDLE;
      else if (idle_state & req) state <= STATE_START;
      else if (full_bit)
         case (state)
            STATE_START:    state <= STATE_D0;
            STATE_D0:       state <= STATE_D1;
            STATE_D1:       state <= STATE_D2;
            STATE_D2:       state <= STATE_D3;
            STATE_D3:       state <= STATE_D4;
            STATE_D4:       state <= STATE_D5;
            STATE_D5:       state <= STATE_D6;
            STATE_D6:       state <= STATE_D7;
            STATE_D7:       state <= STATE_STOP;
            STATE_STOP:     state <= STATE_IDLE;
            default:        state <= STATE_IDLE;
         endcase
      else state <= state;
   end

   // Connect outputs
   assign empty = idle_state;
   assign error = ~idle_state & req;

endmodule
