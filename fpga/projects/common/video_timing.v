// NTSC 240p timing, one sample every 4 clocks.
//
// Clock: 57.2727 MHz = 4 x 14.31818 MHz. Every video sample lasts 4 clocks
// (sub = 0..3), which gives 14.318 MHz samples = exactly 4 per colour
// subcarrier cycle. Everything in the design runs on this one clock.
//
// h = sample in the line (0..909, 0 = start of horizontal sync)
// v = line in the frame (0..261): 0-2 equalising, 3-5 vertical sync,
//     6-8 equalising, 9-21 blank with burst, 22-261 picture (240 lines)
// ph = subcarrier phase of this sample in 90 degree steps. It never resets:
//     910 samples per line = 227.5 cycles, so the phase flips every line,
//     exactly like broadcast NTSC.
//
// All outputs describe the sample shown by `h`/`v`; they are steady for its
// 4 clocks. The encoder registers its result one sample later.
module video_timing (
  input  wire       clk,
  input  wire       rst,
  output reg  [1:0] sub,
  output reg  [9:0] h,
  output reg  [8:0] v,
  output reg  [1:0] ph,
  output wire       sample_end,   // last clock of this sample (sub == 3)
  output wire       line_start,   // first clock of sample 0 of a line
  output wire       frame_start,  // first clock of line 0
  output wire       sync,         // sample is sync tip
  output wire       burst,        // sample is in the colour burst window
  output wire       active,       // sample is picture
  output wire [8:0] px,           // picture x 0..359 (when active)
  output wire [7:0] py            // picture y 0..239 (when active)
);
  localparam SAMPLES    = 910;
  localparam LINES      = 262;
  localparam HALF       = 455;
  localparam HSYNC_LEN  = 67;     // 4.7 us
  localparam EQ_LEN     = 33;     // 2.3 us
  localparam VS_LOW     = HALF - 67;   // broad pulse, 4.7 us serration
  localparam BURST_ON   = 76;
  localparam BURST_OFF  = 76 + 36;
  localparam ACT_ON     = 151;
  localparam ACT_OFF    = 151 + 720;
  localparam FIRST_LINE = 22;

  always @(posedge clk) begin
    if (rst) begin
      sub <= 2'd0;
      h   <= 10'd0;
      v   <= 9'd0;
      ph  <= 2'd0;
    end else begin
      sub <= sub + 2'd1;
      if (sub == 2'd3) begin
        ph <= ph + 2'd1;
        if (h == SAMPLES - 1) begin
          h <= 10'd0;
          v <= (v == LINES - 1) ? 9'd0 : v + 9'd1;
        end else begin
          h <= h + 10'd1;
        end
      end
    end
  end

  assign sample_end  = (sub == 2'd3);
  assign line_start  = (sub == 2'd0) && (h == 10'd0);
  assign frame_start = line_start && (v == 9'd0);

  wire       eq_line = (v < 9'd3) || (v >= 9'd6 && v < 9'd9);
  wire       vs_line = (v >= 9'd3 && v < 9'd6);
  wire [9:0] hh      = (h >= HALF) ? h - HALF : h;     // position in the half line

  assign sync   = eq_line ? (hh < EQ_LEN) :
                  vs_line ? (hh < VS_LOW) :
                            (h < HSYNC_LEN);
  assign burst  = (v >= 9'd9) && (h >= BURST_ON) && (h < BURST_OFF);
  assign active = (v >= FIRST_LINE) && (h >= ACT_ON) && (h < ACT_OFF);

  wire [9:0] hx = h - ACT_ON;
  wire [8:0] vy = v - FIRST_LINE;
  assign px = hx[9:1];
  assign py = vy[7:0];
endmodule
