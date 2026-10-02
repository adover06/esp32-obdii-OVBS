// End-to-end test of the video card, the way the hardware will be used:
//   power-up -> SDRAM init -> self-test pattern on screen
//   ESP32-style SPI: a full 360x240 frame in 10 bands at 40 MHz, swap
//   a clipped, odd-aligned partial rectangle, swap
// The composite output (DAC codes, one per 14.318 MHz sample) is written to
// out/<corner>/<name>.txt and checked by check_video.py, which finds sync
// on its own like a TV would and compares every sample with the expected one.
//
// Board delays are modelled (+define corners via the Makefile/run script):
//   T_CO  FPGA clock-to-pin + trace for SDRAM commands/data
//   T_CLK FPGA clock-to-pin + trace for the SDRAM clock
//   T_IN  SDRAM pin to FPGA capture register
`timescale 1ps/1ps
module tb;
  parameter T_CO   = 5000;
  parameter T_CLK  = 4000;
  parameter T_IN   = 2000;
  parameter T_AC   = 6000;
  parameter T_OH   = 2500;
  parameter CL     = 2;
  parameter SCK_PS = 25000;          // 40 MHz SPI
  parameter QUICK  = 0;              // 1 = skip the partial-update test
  parameter CORNER = "typ";

  reg clk12 = 0;
  always #41667 clk12 = ~clk12;

  // ---- DUT -----------------------------------------------------------------
  wire [7:0]  led;
  wire [5:0]  dac;
  reg         sck = 0, mosi = 0, cs_n = 1;
  wire        miso, ready;
  wire        sd_clk, sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n;
  wire [1:0]  sd_ba, sd_dqm;
  wire [13:0] sd_a;
  wire [15:0] dq_fpga;
  reg         btn = 1;

  video_card #(.LUT_FILE("../../../common/ntsc_lut.hex"), .SDRAM_CL(CL)) dut (
    .CLK12M(clk12), .USER_BTN(btn), .LED(led), .DAC(dac),
    .SPI_SCK(sck), .SPI_MOSI(mosi), .SPI_CS_N(cs_n), .SPI_MISO(miso), .FPGA_READY(ready),
    .SDRAM_CLK(sd_clk), .SDRAM_CKE(sd_cke), .SDRAM_CS_N(sd_cs_n), .SDRAM_RAS_N(sd_ras_n),
    .SDRAM_CAS_N(sd_cas_n), .SDRAM_WE_N(sd_we_n), .SDRAM_BA(sd_ba), .SDRAM_A(sd_a),
    .SDRAM_DQM(sd_dqm), .SDRAM_DQ(dq_fpga)
  );

  // ---- board: pin and trace delays ---------------------------------------
  wire        m_clk, m_cke, m_cs_n, m_ras_n, m_cas_n, m_we_n;
  wire [1:0]  m_ba, m_dqm;
  wire [11:0] m_a;
  wire [15:0] dq_mem;
  wire        m_drv_en;
  wire [15:0] m_drv_val;

  assign #(T_CLK) m_clk = sd_clk;
  assign #(T_CO) {m_cke, m_cs_n, m_ras_n, m_cas_n, m_we_n, m_ba, m_dqm, m_a} =
                 {sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n, sd_ba, sd_dqm, sd_a[11:0]};
  // FPGA -> SDRAM data (only while the controller drives it)
  wire        f_oe  = dut.u_sdram.dq_oe;
  wire [15:0] f_out = dut.u_sdram.dq_out;
  wire        f_oe_d;
  wire [15:0] f_out_d;
  assign #(T_CO) f_oe_d  = f_oe;
  assign #(T_CO) f_out_d = f_out;
  assign dq_mem = f_oe_d ? f_out_d : 16'bz;
  // SDRAM -> FPGA data
  wire        m_en_d;
  wire [15:0] m_val_d;
  assign #(T_IN) m_en_d  = m_drv_en;
  assign #(T_IN) m_val_d = m_drv_val;
  assign dq_fpga = m_en_d ? m_val_d : 16'bz;

  sdram_model #(.CL(CL), .T_AC(T_AC), .T_OH(T_OH)) mem (
    .clk(m_clk), .cke(m_cke), .cs_n(m_cs_n), .ras_n(m_ras_n), .cas_n(m_cas_n), .we_n(m_we_n),
    .ba(m_ba), .a(m_a), .dqm(m_dqm), .dq(dq_mem), .drv_en(m_drv_en), .drv_val(m_drv_val)
  );

  // ---- read capture margin at the FPGA's falling-edge register ------------
  // data must be stable >= 1 ns before and >= 0.5 ns after each capture edge
  integer margin_err = 0;
  time    dq_change = 0, last_cap = 0;
  reg     cap_valid = 0;
  always @(dq_fpga) begin
    dq_change = $time;
    if (cap_valid && $time - last_cap < 500) begin
      margin_err = margin_err + 1;
      if (margin_err < 5) $display("ERROR @%0t: read data hold margin < 0.5 ns", $time);
    end
  end
  always @(negedge dut.clk) begin
    cap_valid = 0;
    if (dut.u_sdram.rd_v[CL]) begin   // a read word is due at this capture edge
      last_cap  = $time;
      cap_valid = 1;
      if (^dq_fpga === 1'bx) begin
        margin_err = margin_err + 1;
        if (margin_err < 5) $display("ERROR @%0t: captured X read data", $time);
      end else if ($time - dq_change < 1000) begin
        margin_err = margin_err + 1;
        if (margin_err < 5) $display("ERROR @%0t: read data setup margin < 1 ns", $time);
      end
    end
  end

  // ---- SPI master (ESP32, mode 0) ------------------------------------------
  reg [15:0] crc;
  integer    nbytes;
  reg [63:0] status_rx;

  function [15:0] crc_step(input [15:0] c, input [7:0] d);
    integer k;
    reg [15:0] x;
    begin
      x = c;
      for (k = 7; k >= 0; k = k - 1)
        x = (x[15] ^ d[k]) ? ({x[14:0], 1'b0} ^ 16'h1021) : {x[14:0], 1'b0};
      crc_step = x;
    end
  endfunction

  task spi_begin;
    begin
      crc = 16'hFFFF; nbytes = 0;
      cs_n = 0;
      #(SCK_PS);
    end
  endtask

  task spi_end;
    begin
      #(SCK_PS / 2);
      cs_n = 1;
      #(3000000);          // 3 us between transactions (ESP32 driver overhead)
    end
  endtask

  task spi_byte(input [7:0] b);
    integer k;
    begin
      for (k = 7; k >= 0; k = k - 1) begin
        mosi = b[k];
        #(SCK_PS / 2) sck = 1;
        status_rx = {status_rx[62:0], miso};
        #(SCK_PS / 2) sck = 0;
      end
      crc = crc_step(crc, b);
      nbytes = nbytes + 1;
    end
  endtask

  // status read: 'Q' + 7 more bytes; returns the status latched at the end
  // of the previous transaction
  task spi_status;
    integer k;
    begin
      spi_begin;
      spi_byte(8'h51);
      for (k = 0; k < 7; k = k + 1) spi_byte(8'h00);
      spi_end;
    end
  endtask

  // after any transaction: read status and check its byte count and CRC
  integer link_err = 0;
  task check_last(input [15:0] exp_crc, input [15:0] exp_cnt);
    begin
      spi_status;
      if (status_rx[63:56] != 8'hA5 || status_rx[31:16] != exp_cnt || status_rx[15:0] != exp_crc) begin
        link_err = link_err + 1;
        $display("ERROR: status %h, expected count %0d crc %h", status_rx, exp_cnt, exp_crc);
      end
    end
  endtask

  reg [7:0] img [0:86399];
  reg [7:0] rect [0:3999];

  task send_rect(input integer x, input integer y, input integer w, input integer h,
                 input integer src, input integer extra);
    integer i;
    reg [15:0] c; integer n;
    begin
      spi_begin;
      spi_byte(8'h57); spi_byte(0); spi_byte(0); spi_byte(0);
      spi_byte(x[7:0]); spi_byte(x[15:8]); spi_byte(y[7:0]); spi_byte(y[15:8]);
      spi_byte(w[7:0]); spi_byte(w[15:8]); spi_byte(h[7:0]); spi_byte(h[15:8]);
      for (i = 0; i < w * h; i = i + 1)
        spi_byte(src == 0 ? img[y * 360 + i] : rect[i]);
      for (i = 0; i < extra; i = i + 1) spi_byte(8'hEE);   // must be ignored
      c = crc; n = nbytes;
      spi_end;
      check_last(c, n[15:0]);
    end
  endtask

  task send_swap;
    begin
      spi_begin;
      spi_byte(8'h53); spi_byte(0); spi_byte(0); spi_byte(0);
      spi_end;
    end
  endtask

  task wait_ready;
    integer t;
    begin
      t = 0;
      while (!ready && t < 40000) begin #1000000; t = t + 1; end
      if (!ready) begin $display("ERROR: READY never came back"); $finish; end
    end
  endtask

  // ---- composite capture ------------------------------------------------------
  // one DAC code per sample, starting near the end of a frame, 1 frame + 30 lines
  integer fd;
  // files go to the current directory (run.sh runs each corner in out/<corner>)
  task capture(input integer which);
    integer n;
    begin
      // start at line 250 so the file holds a whole frame after a vsync
      wait (dut.v == 9'd250 && dut.sub == 2'd0);
      case (which)
        0: fd = $fopen("bist.txt", "w");
        1: fd = $fopen("img1.txt", "w");
        default: fd = $fopen("img2.txt", "w");
      endcase
      for (n = 0; n < 910 * (262 + 30); n = n + 1) begin
        @(posedge dut.clk); @(posedge dut.clk);
        $fwrite(fd, "%0d\n", dac);
        @(posedge dut.clk); @(posedge dut.clk);
      end
      $fclose(fd);
      $display("[TB] captured picture %0d at %0t us", which, $time / 1000000);
    end
  endtask

  // DAC pins may only change once per sample (every 4 clocks)
  integer dac_glitch = 0;
  reg [5:0] dac_prev;
  always @(posedge dut.clk) begin
    if (dac !== dac_prev && dut.sub != 2'd0 && $time > 2000000) dac_glitch = dac_glitch + 1;
    dac_prev <= dac;
  end

  // ---- the test ------------------------------------------------------------------
  integer bands, errors;
  initial begin
    $readmemh("../img1.hex", img);
    $readmemh("../rect.hex", rect);
    $display("[TB] corner %0s: T_CO %0d ps, T_CLK %0d ps, T_IN %0d ps, tAC %0d ps, CL %0d",
             CORNER, T_CO, T_CLK, T_IN, T_AC, CL);

    // 1. power-up self test: the pattern comes back out of the SDRAM
    wait (ready);
    $display("[TB] self test written, READY high at %0t us", $time / 1000000);
    spi_status; spi_status;
    if (status_rx[63:56] != 8'hA5 || status_rx[49] != 1'b1) begin
      $display("ERROR: bad status after self test: %h", status_rx);
      link_err = link_err + 1;
    end
    capture(0);
    if (dut.verify_words < 180 * 240) begin
      $display("ERROR: self-test checker only saw %0d words", dut.verify_words);
      link_err = link_err + 1;
    end

    // 2. a full ESP32 frame: 10 bands of 24 rows, then swap
    for (bands = 0; bands < 10; bands = bands + 1)
      send_rect(0, bands * 24, 360, 24, 0, 0);
    send_swap;
    #(1000000);
    if (ready) begin $display("ERROR: READY stayed high after a swap request"); link_err = link_err + 1; end
    wait_ready;
    capture(1);

    if (!QUICK) begin
      // 3. odd x, clipped right and bottom edges, trailing junk bytes, and an
      //    ignored 'X' transaction in between
      spi_begin; spi_byte(8'h58); spi_byte(1); spi_byte(2); spi_byte(3); spi_end;
      send_rect(301, 211, 100, 40, 1, 7);
      send_rect(7, 3, 0, 5, 1, 3);           // zero width: ignored
      send_swap;
      wait_ready;
      capture(2);
    end

    spi_status; spi_status;
    mem.report;
    errors = mem.errors + margin_err + link_err + dac_glitch;
    if (status_rx[52] || status_rx[51] || status_rx[50]) begin
      $display("ERROR: sticky flags set: %b (fetch late, fifo overflow, verify)", status_rx[52:50]);
      errors = errors + 1;
    end
    $display("[TB] status %h, self-test words checked %0d, dac glitches %0d, read margin errors %0d, link errors %0d",
             status_rx, dut.verify_words, dac_glitch, margin_err, link_err);
    $display("[TB] %s (%0d errors)", errors ? "FAIL" : "PASS", errors);
    $finish;
  end

  initial begin
    #(400 * 1000000000.0);   // 400 ms
    $display("ERROR: timeout");
    $finish;
  end
endmodule
