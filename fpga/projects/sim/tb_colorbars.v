// Colour-bar test design: capture one frame of composite and let
// check_video.py verify it (USER button held, so the box stays at 20,20).
`timescale 1ps/1ps
module tb;
  reg clk12 = 0;
  always #41667 clk12 = ~clk12;
  wire [7:0] led;
  wire [5:0] dac;
  colorbars #(.LUT_FILE("../../../common/ntsc_lut.hex")) dut (
    .CLK12M(clk12), .USER_BTN(1'b0), .LED(led), .DAC(dac));

  integer fd, n;
  initial begin
    wait (dut.v == 9'd250 && dut.sub == 2'd0);
    fd = $fopen("bars.txt", "w");
    for (n = 0; n < 910 * (262 + 30); n = n + 1) begin
      @(posedge dut.clk); @(posedge dut.clk);
      $fwrite(fd, "%0d\n", dac);
      @(posedge dut.clk); @(posedge dut.clk);
    end
    $fclose(fd);
    $display("[TB] PASS (capture done; see check_video.py)");
    $finish;
  end
endmodule
