// MAX1000 composite video card for the ESP32 dashboard.
//
//   ESP32 --SPI--> parser --> write FIFO --> SDRAM (2 x 720x240 frame buffers)
//   SDRAM --line fetch--> line buffer --> NTSC encoder --> 6-bit R-2R DAC --> TV
//
// The ESP32 draws a frame in bands and sends them with 'W' commands into the
// back buffer, then sends 'S'. At the next vertical blank the buffers swap,
// so the TV only ever shows finished frames (no tearing, no flicker).
//
// Power-up self test: after the SDRAM starts, both buffers are filled with
// the test pattern and the TV shows it, read back through the SDRAM. Every
// line read is compared with the expected pattern until the ESP32's first
// frame arrives; a mismatch lights LED8 (SDRAM timing problem).
//
// LEDs: 1 heartbeat (~1 Hz)       2 SDRAM ready
//       3 self test passing        4 SPI activity
//       5 front buffer (toggles on every swap)
//       6 FIFO overflow (error)    7 line fetch late (error)    8 SDRAM data error
// USER button: clears the error LEDs.
//
// ---- Reading Verilog if you know Python ---------------------------------------
// Verilog describes HARDWARE, not steps. Everything below exists at the same
// time and runs in parallel, all the time. The main shapes:
//
//   wire [7:0] x = a + b;      A wire is like a Python property: x is ALWAYS
//                              a + b, recomputed instantly when a or b change.
//                              [7:0] = 8 bits wide (bit 7 down to bit 0).
//
//   always @(posedge clk)      Runs once per clock tick (57 million times a
//     x <= x + 1;              second). `x` is a register (reg): a variable that
//                              keeps its value between ticks. `<=` means "x
//                              becomes this AT the tick". Every `<=` in the
//                              design happens at the same instant, using the
//                              values from BEFORE the tick, like computing a
//                              whole new state dict from the old one:
//                                  new = {"x": old["x"] + 1, "y": old["x"]}
//
//   module foo (...)           Like a class. `foo u_name (...)` makes an
//                              instance (u_name = foo(...)) and the
//                              .port(signal) pairs are its wiring.
//
//   {a, b}                     Joins bits: {2'b10, 2'b01} = 4'b1001.
//   8'hA5, 9'd359, 1'b0        Sized numbers: width ' base value
//                              (h = hex, d = decimal, b = binary).
//   x[3]  x[8:1]               One bit / a slice (x >> 1 & 0xFF in Python).
//   cond ? a : b               Python's  a if cond else b
// --------------------------------------------------------------------------------
module video_card #(
  parameter LUT_FILE   = "../common/ntsc_lut.hex",
  parameter INIT_WAIT  = 14000,      // SDRAM power-up wait (simulation shortens it)
  parameter SDRAM_CL   = 2
) (
  input  wire        CLK12M,
  input  wire        USER_BTN,       // low when pressed
  output wire [7:0]  LED,

  output wire [5:0]  DAC,            // MKR J2 pins 1,2,3,4,5,8 = D6..D10, D13 (bit 0 = LSB on D6)

  input  wire        SPI_SCK,        // MKR D0
  input  wire        SPI_MOSI,       // MKR D1
  input  wire        SPI_CS_N,       // MKR D2
  output wire        SPI_MISO,       // MKR D3
  output wire        FPGA_READY,     // MKR D4: high = ready for the next frame

  output wire        SDRAM_CLK,
  output wire        SDRAM_CKE,
  output wire        SDRAM_CS_N,
  output wire        SDRAM_RAS_N,
  output wire        SDRAM_CAS_N,
  output wire        SDRAM_WE_N,
  output wire [1:0]  SDRAM_BA,
  output wire [13:0] SDRAM_A,
  output wire [1:0]  SDRAM_DQM,
  inout  wire [15:0] SDRAM_DQ
);
  // ---- clock and reset ----------------------------------------------------
  // The PLL turns the board's 12 MHz crystal into 57.27 MHz (clk), which runs
  // everything. `rst` stays high for a few ticks after the PLL locks, so every
  // block starts from a known state (like calling __init__ on all of them).
  wire clk, pll_locked;
  pll_57m u_pll (.inclk(CLK12M), .c0(clk), .c1(SDRAM_CLK), .locked(pll_locked));

  reg [3:0] rst_sh = 4'hF;
  always @(posedge clk or negedge pll_locked)
    if (!pll_locked) rst_sh <= 4'hF;
    else rst_sh <= {rst_sh[2:0], 1'b0};
  wire rst = rst_sh[3];

  // ---- video timing --------------------------------------------------------
  // Counters that say where the TV's beam is: h = position along the line,
  // v = which line. From them: is this moment sync, colour burst, or picture,
  // and if picture, which pixel (px, py). See common/video_timing.v.
  wire [1:0] sub, ph;
  wire [9:0] h;
  wire [8:0] v;
  wire [9:0] px;
  wire [7:0] py;
  wire       sample_end, line_start, frame_start, vsync_tip, burst, active;
  video_timing u_tim (
    .clk(clk), .rst(rst), .sub(sub), .h(h), .v(v), .ph(ph),
    .sample_end(sample_end), .line_start(line_start), .frame_start(frame_start),
    .sync(vsync_tip), .burst(burst), .active(active), .px(px), .py(py)
  );

  // ---- SDRAM ---------------------------------------------------------------
  // The 8 MB memory chip holds the two frames. Memory layout: SDRAM row
  // number = {buffer bit, picture line}, so line 37 of buffer 1 is row 256+37.
  // The controller (sdram_ctrl.v) does three jobs: fetch a line for the TV,
  // write pixels from the FIFO, and refresh (SDRAM forgets without it).
  wire        init_done, fetch_overrun, lb_we, wr_busy, wf_pop;
  wire [9:0]  lb_waddr;
  wire [15:0] lb_wdata;
  wire        wf_empty, wf_full, wf_overflow;
  wire [9:0]  wf_count;
  wire [26:0] wf_head;
  reg         fetch_req;
  reg  [8:0]  fetch_row;
  reg         fetch_half;
  reg         front;
  reg         swap_req;
  wire [11:0] sd_a;
  wire        clear_err;

  sdram_ctrl #(.CL(SDRAM_CL), .INIT_WAIT(INIT_WAIT)) u_sdram (
    .clk(clk), .rst(rst), .init_done(init_done),
    .fetch_req(fetch_req), .fetch_row(fetch_row), .fetch_half(fetch_half),
    .fetch_overrun(fetch_overrun), .clear_err(clear_err),
    .lb_we(lb_we), .lb_waddr(lb_waddr), .lb_wdata(lb_wdata),
    .wf_empty(wf_empty), .wf_count(wf_count), .wf_head(wf_head), .wf_pop(wf_pop),
    .wf_flush(swap_req), .wr_busy(wr_busy),
    .sd_cs_n(SDRAM_CS_N), .sd_ras_n(SDRAM_RAS_N), .sd_cas_n(SDRAM_CAS_N), .sd_we_n(SDRAM_WE_N),
    .sd_ba(SDRAM_BA), .sd_a(sd_a), .sd_dqm(SDRAM_DQM), .sd_dq(SDRAM_DQ)
  );
  assign SDRAM_A   = {2'b00, sd_a};   // A12/A13 only exist on the 256 Mbit fit
  assign SDRAM_CKE = 1'b1;

  // Fetch the next picture line at the start of each line. It has a whole
  // line (63.5 us) to arrive; it takes ~3.5 us. Picture lines are TV lines
  // 22..261, so TV line v shows picture line v - 22.
  wire [8:0] next_v    = v + 9'd1;
  wire [8:0] next_line = next_v - 9'd22;
  always @(posedge clk) begin
    fetch_req <= 1'b0;
    if (line_start && next_v >= 9'd22 && next_v <= 9'd261) begin
      fetch_req  <= 1'b1;
      fetch_row  <= {front, next_line[7:0]};
      fetch_half <= next_line[0];
    end
  end

  // ---- line buffer: 2 lines x 360 words, written by the SDRAM, read by video
  // A small on-chip RAM, like a list of 512 16-bit ints. Half 0 holds even
  // picture lines, half 1 odd ones: while the TV reads one half, the SDRAM
  // fills the other with the next line (ping-pong). Each 16-bit word holds two
  // pixels: the even-x pixel in the low byte, the odd-x pixel in the high byte.
  reg [15:0] lbuf [0:1023];
  reg [15:0] lb_q;
  always @(posedge clk) begin
    if (lb_we) lbuf[lb_waddr] <= lb_wdata;
    lb_q <= lbuf[{py[0], px[9:1]}];
  end
  wire [7:0] pix = px[0] ? lb_q[15:8] : lb_q[7:0];

  // The encoder turns the pixel colour into the voltage level for this
  // instant (sync, burst, or colour) and drives the 6 DAC pins.
  wire [5:0] dac_code;
  ntsc_encoder #(.LUT_FILE(LUT_FILE)) u_enc (
    .clk(clk), .sub(sub), .ph(ph), .sync(vsync_tip), .burst(burst), .active(active),
    .pix(pix), .dac(dac_code)
  );
  assign DAC = dac_code;

  // ---- SPI -----------------------------------------------------------------
  // spi_slave turns the ESP32's bits into bytes; cmd_parser turns bytes into
  // pixel writes (pushed into the FIFO) and swap requests.
  wire        byte_valid, byte_first, xfer_end, miso, miso_oe;
  wire [7:0]  byte_data;
  wire [63:0] status;
  spi_slave u_spi (
    .clk(clk), .rst(rst), .sck(SPI_SCK), .mosi(SPI_MOSI), .cs_n(SPI_CS_N),
    .miso(miso), .miso_oe(miso_oe), .status(status),
    .byte_valid(byte_valid), .byte_data(byte_data), .byte_first(byte_first), .xfer_end(xfer_end)
  );
  assign SPI_MISO = miso_oe ? miso : 1'bz;

  reg bist_done;
  wire        p_push, swap_cmd;
  wire [26:0] p_data;
  cmd_parser u_parse (
    .clk(clk), .rst(rst), .enable(bist_done), .back(~front),
    .byte_valid(byte_valid), .byte_data(byte_data), .byte_first(byte_first),
    .push(p_push), .push_data(p_data), .swap_cmd(swap_cmd)
  );

  // CRC and byte count of each transaction, returned in the next status read.
  // The ESP32 computes the same CRC over what it sent; if they match, every
  // byte arrived intact. Same algorithm as tools/ntsc.py crc16().
  function [15:0] crc16_byte(input [15:0] c, input [7:0] d);
    integer k;
    reg [15:0] x;
    begin
      x = c;
      for (k = 7; k >= 0; k = k - 1)
        x = (x[15] ^ d[k]) ? ({x[14:0], 1'b0} ^ 16'h1021) : {x[14:0], 1'b0};
      crc16_byte = x;
    end
  endfunction

  reg [15:0] crc_acc, cnt_acc;
  always @(posedge clk) begin
    if (byte_valid) begin
      crc_acc <= crc16_byte(byte_first ? 16'hFFFF : crc_acc, byte_data);
      cnt_acc <= byte_first ? 16'd1 : cnt_acc + 16'd1;
    end
  end

  // ---- self test: fill both buffers with the pattern -----------------------
  // At power-up, (bx, by) walks over every pixel of buffer 0, then buffer 1,
  // pushing the test-pattern colour into the write FIFO, like:
  //     for buf in (0, 1): for y in range(240): for x in range(720): push(...)
  // except one pixel per clock tick, pausing whenever the FIFO is full.
  reg       bist_run, bist_buf;
  reg [3:0] idle_cnt;   // clocks the FIFO and writer have been idle (saturates at 8)
  always @(posedge clk)
    if (rst || !wf_empty || wr_busy || bist_run) idle_cnt <= 4'd0;
    else if (!idle_cnt[3]) idle_cnt <= idle_cnt + 4'd1;
  reg [9:0] bx;
  reg [7:0] by;
  wire [7:0] bist_pix;
  test_pattern u_bpat (.x(bx), .y(by), .c(bist_pix));

  always @(posedge clk) begin
    if (rst) begin
      bist_run  <= 1'b0;
      bist_done <= 1'b0;
      bist_buf  <= 1'b0;
      bx <= 10'd0; by <= 8'd0;
    end else if (!bist_done) begin
      if (!bist_run) begin
        if (bist_buf && idle_cnt[3]) bist_done <= 1'b1;   // everything written
        else if (init_done && !bist_buf && bx == 0 && by == 0) bist_run <= 1'b1;
      end else if (!wf_full) begin
        if (bx == 10'd719) begin
          bx <= 10'd0;
          if (by == 8'd239) begin
            by <= 8'd0;
            if (bist_buf) bist_run <= 1'b0;
            bist_buf <= 1'b1;
          end else by <= by + 8'd1;
        end else bx <= bx + 10'd1;
      end
    end
  end
  wire        b_push = bist_run && !wf_full;
  wire [26:0] b_data = {bist_buf, by, bx[9:1], bx[0], bist_pix};

  // The write FIFO is a queue (like collections.deque) between the pixel
  // sources and the SDRAM: the self test feeds it first, then the ESP32.
  wfifo u_fifo (
    .clk(clk), .rst(rst),
    .push(bist_done ? p_push : b_push), .din(bist_done ? p_data : b_data),
    .full(wf_full), .overflow(wf_overflow), .clear_err(clear_err),
    .pop(wf_pop), .head(wf_head), .empty(wf_empty), .count(wf_count)
  );

  // ---- buffer swap at vertical blank --------------------------------------
  // Swap in lines 0-18: after the last picture line, before the first fetch
  // (line 21), and only once every queued pixel is in the SDRAM.
  wire       do_swap = line_start && v <= 9'd18 && swap_req && wf_empty && !wr_busy;
  reg [15:0] frame_cnt;
  reg        spi_frames;   // the ESP32 has shown a frame: self-test check over
  reg        check_en;
  always @(posedge clk) begin
    if (rst) begin
      front      <= 1'b0;
      swap_req   <= 1'b0;
      frame_cnt  <= 16'd0;
      spi_frames <= 1'b0;
      check_en   <= 1'b0;
    end else begin
      if (swap_cmd) swap_req <= 1'b1;
      if (frame_start) frame_cnt <= frame_cnt + 16'd1;
      // check from the first whole frame after the self-test pattern is written
      if (frame_start && bist_done) check_en <= 1'b1;
      if (do_swap) begin
        front      <= ~front;
        swap_req   <= 1'b0;
        spi_frames <= 1'b1;
      end
    end
  end
  wire checking = check_en && !spi_frames;

  // ---- read-back check of every fetched word against the pattern ----------
  // While the self-test picture is on screen, every word the SDRAM returns is
  // compared with what the pattern says it should be. One mismatch sets
  // verify_err (LED8) and it stays set: that would mean SDRAM timing trouble.
  reg  [7:0] chk_line;
  wire [7:0] exp_lo, exp_hi;
  test_pattern u_clo (.x({lb_waddr[8:0], 1'b0}), .y(chk_line), .c(exp_lo));
  test_pattern u_chi (.x({lb_waddr[8:0], 1'b1}), .y(chk_line), .c(exp_hi));
  always @(posedge clk) if (fetch_req) chk_line <= next_line[7:0];

  reg verify_err;
  reg [23:0] verify_words;   // words checked (debug/sim only; a 720-wide frame is 86,400)
  always @(posedge clk) begin
    if (rst) begin
      verify_err   <= 1'b0;
      verify_words <= 24'd0;
    end else begin
      if (clear_err) verify_err <= 1'b0;
      if (lb_we && checking) begin
        verify_words <= verify_words + 24'd1;
        if (lb_wdata != {exp_hi, exp_lo}) verify_err <= 1'b1;
      end
    end
  end

  // ---- status word (read over MISO) and pins -------------------------------
  // 64 bits the ESP32 can read back: A5 (a marker so it knows the FPGA is
  // there), 8 flag bits, frame counter, and the previous transaction's byte
  // count and CRC.
  wire ready = bist_done && !swap_req;
  assign FPGA_READY = ready;
  wire [7:0] flags = {init_done, checking, front, fetch_overrun, wf_overflow, verify_err, bist_done, ready};
  assign status = {8'hA5, flags, frame_cnt, cnt_acc, crc_acc};

  // user button: sync, then a press clears the sticky errors
  reg [2:0] btn_s = 3'b111;
  always @(posedge clk) btn_s <= {btn_s[1:0], USER_BTN};
  assign clear_err = btn_s[2] & ~btn_s[1];

  // SPI activity, stretched to ~70 ms
  reg [21:0] act_cnt;
  always @(posedge clk) begin
    if (rst) act_cnt <= 22'd0;
    else if (byte_valid) act_cnt <= 22'h3FFFFF;
    else if (act_cnt != 0) act_cnt <= act_cnt - 22'd1;
  end

  assign LED = {verify_err, fetch_overrun, wf_overflow, front,
                act_cnt != 0, checking && !verify_err, init_done, frame_cnt[5]};
endmodule
