// Composite encoder: pixel colour + timing -> 6-bit DAC code.
//
// All the colour maths is done offline (tools/gen_lut.py): a 1024-entry ROM
// holds the final DAC code for every RGB332 colour at each of the 4
// subcarrier phases. Sync, blank and burst are constant levels.
//
// Pipeline inside one sample (4 clocks, see video_timing.v):
//   pix must be valid from sub==1 -> ROM output valid from sub==2 ->
//   dac register loads at the end of sub==3. So the DAC shows each sample
//   exactly one sample (70 ns) after the timing counters, for every signal
//   alike, which keeps sync, burst and picture aligned.
module ntsc_encoder #(
  parameter LUT_FILE = "ntsc_lut.hex"
) (
  input  wire       clk,
  input  wire [1:0] sub,
  input  wire [1:0] ph,
  input  wire       sync,
  input  wire       burst,
  input  wire       active,
  input  wire [7:0] pix,
  output reg  [5:0] dac
);
`include "ntsc_levels.vh"

  reg [7:0] lut [0:1023];     // 8 bits wide to match the 2-digit hex file; top 2 bits are 0
  initial $readmemh(LUT_FILE, lut);

  reg [7:0] lut_q;
  always @(posedge clk) lut_q <= lut[{pix, ph}];

  // burst = -sin: phase 90 deg is the low peak, 270 deg the high peak
  reg [5:0] code;
  always @* begin
    if (sync)                    code = LVL_SYNC;
    else if (burst && ph == 2'd1) code = LVL_BURST_LO;
    else if (burst && ph == 2'd3) code = LVL_BURST_HI;
    else if (active)             code = lut_q[5:0];
    else                         code = LVL_BLANK;
  end

  initial dac = LVL_BLANK;
  always @(posedge clk) if (sub == 2'd3) dac <= code;
endmodule
