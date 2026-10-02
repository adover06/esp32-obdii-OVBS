// Built-in 360x240 test picture (RGB332), purely combinational.
//   rows   0-159: 8 colour bars (white yellow cyan green magenta red blue black)
//   rows 160-199: 8-step grey ramp
//   rows 200-239: 8x8 checkerboard
//   1-pixel white border, to see how much the TV crops
// tools/ntsc.py pattern() is the reference copy used by the checkers.
module test_pattern (
  input  wire [8:0] x,
  input  wire [7:0] y,
  output reg  [7:0] c
);
  wire [2:0] bar = (x <  9'd45)  ? 3'd0 :
                   (x <  9'd90)  ? 3'd1 :
                   (x <  9'd135) ? 3'd2 :
                   (x <  9'd180) ? 3'd3 :
                   (x <  9'd225) ? 3'd4 :
                   (x <  9'd270) ? 3'd5 :
                   (x <  9'd315) ? 3'd6 : 3'd7;

  always @* begin
    if (x == 9'd0 || x == 9'd359 || y == 8'd0 || y == 8'd239)
      c = 8'hFF;
    else if (y < 8'd160)
      case (bar)
        3'd0: c = 8'hFF;   // white
        3'd1: c = 8'hFC;   // yellow
        3'd2: c = 8'h1F;   // cyan
        3'd3: c = 8'h1C;   // green
        3'd4: c = 8'hE3;   // magenta
        3'd5: c = 8'hE0;   // red
        3'd6: c = 8'h03;   // blue
        default: c = 8'h00;
      endcase
    else if (y < 8'd200)
      c = {bar, bar, bar[2:1]};
    else
      c = (x[3] ^ y[3]) ? 8'hFF : 8'h00;
  end
endmodule
