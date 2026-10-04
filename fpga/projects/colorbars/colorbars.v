// Video check without the SDRAM or the ESP32: the built-in test pattern
// (colour bars, grey ramp, checkerboard, white border) plus a bouncing box,
// generated live and sent straight to the DAC. If this looks right on the
// TV, the DAC wiring, levels, sync and colour are right.
//
// LEDs: 1 heartbeat, 2 PLL locked. USER button: freeze the box.
module colorbars #(
  parameter LUT_FILE = "../common/ntsc_lut.hex"
) (
  input  wire       CLK12M,
  input  wire       USER_BTN,
  output wire [7:0] LED,
  output wire [5:0] DAC
);
  wire clk, clk_unused, locked;
  pll_57m u_pll (.inclk(CLK12M), .c0(clk), .c1(clk_unused), .locked(locked));

  reg [3:0] rst_sh;
  always @(posedge clk or negedge locked)
    if (!locked) rst_sh <= 4'hF;
    else rst_sh <= {rst_sh[2:0], 1'b0};
  wire rst = rst_sh[3];

  wire [1:0] sub, ph;
  wire [9:0] h;
  wire [8:0] v;
  wire [9:0] px;
  wire [7:0] py;
  wire       sample_end, line_start, frame_start, sync, burst, active;
  video_timing u_tim (
    .clk(clk), .rst(rst), .sub(sub), .h(h), .v(v), .ph(ph),
    .sample_end(sample_end), .line_start(line_start), .frame_start(frame_start),
    .sync(sync), .burst(burst), .active(active), .px(px), .py(py)
  );

  // bouncing box: 48 x 24 pixels (720-wide pixels are half as wide, so it's square on screen)
  reg [9:0] bx;
  reg [7:0] by;
  reg       dx, dy;
  reg [5:0] frames;
  reg [2:0] btn_s;
  always @(posedge clk) begin
    btn_s <= {btn_s[1:0], USER_BTN};
    if (rst) begin
      bx <= 10'd40; by <= 8'd20; dx <= 1'b1; dy <= 1'b1; frames <= 6'd0;
    end else if (frame_start) begin
      frames <= frames + 6'd1;
      if (btn_s[2]) begin
        if (dx) begin if (bx >= 10'd666) dx <= 1'b0; bx <= bx + 10'd4; end
        else    begin if (bx <= 10'd6)   dx <= 1'b1; bx <= bx - 10'd4; end
        if (dy) begin if (by >= 8'd213) dy <= 1'b0; by <= by + 8'd1; end
        else    begin if (by <= 8'd2)   dy <= 1'b1; by <= by - 8'd1; end
      end
    end
  end

  wire [7:0] pat;
  test_pattern u_pat (.x(px), .y(py), .c(pat));
  wire in_box  = (px >= bx) && (px < bx + 10'd48) && (py >= by) && (py < by + 8'd24);
  wire box_rim = in_box && (px < bx + 10'd4 || px >= bx + 10'd44 || py < by + 8'd2 || py >= by + 8'd22);
  wire [7:0] pix = box_rim ? 8'h00 : in_box ? 8'hF8 : pat;   // orange box, black rim

  ntsc_encoder #(.LUT_FILE(LUT_FILE)) u_enc (
    .clk(clk), .sub(sub), .ph(ph), .sync(sync), .burst(burst), .active(active),
    .pix(pix), .dac(DAC)
  );

  assign LED = {6'd0, locked, frames[5]};
endmodule
