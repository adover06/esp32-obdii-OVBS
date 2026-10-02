// Board check: a light runs back and forth across the 8 LEDs (~8 steps/s).
// Hold the USER button to make it run 4x faster. No PLL, no extra wiring.
module blink (
  input  wire       CLK12M,
  input  wire       USER_BTN,   // low when pressed
  output reg  [7:0] LED
);
  reg [22:0] div = 23'd0;
  reg [3:0]  pos = 4'd0;        // 0..13: LEDs 0-7, then 6 down to 1
  wire       step = USER_BTN ? (div == 23'd0) : (div[20:0] == 21'd0);

  always @(posedge CLK12M) begin
    div <= div + 23'd1;
    if (step) pos <= (pos == 4'd13) ? 4'd0 : pos + 4'd1;
    LED <= 8'd1 << ((pos < 4'd8) ? pos[2:0] : 3'd6 - pos[2:0]);
  end
endmodule
