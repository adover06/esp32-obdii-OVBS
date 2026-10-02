// Simulation stand-in for common/pll_57m.v: ideal 57.2727 MHz clock and
// the SDRAM clock delayed by the same 6.73 ns as the real PLL's c1.
// The input clock is ignored.
`timescale 1ps/1ps
module pll_57m (
  input  wire inclk,
  output reg  c0,
  output wire c1,
  output reg  locked
);
  localparam real HALF = 8727.27;   // ps (57.2727 MHz)
  initial begin
    c0 = 1'b0;
    locked = 1'b0;
    #(HALF * 20) locked = 1'b1;
  end
  always #(HALF) c0 = ~c0;
  localparam real SDRAM_SHIFT = 6730.0;   // keep equal to clk1_phase_shift in pll_57m.v
  assign #(SDRAM_SHIFT) c1 = c0;
endmodule
