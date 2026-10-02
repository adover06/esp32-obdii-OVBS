// SPI slave, mode 0 (ESP32 is the master), MSB first.
//
// Receive: bits are shifted in on the SPI clock itself (so SCK can be much
// faster than an oversampling design would allow, ~40 MHz). Each finished
// byte is held in rx_byte for the next 8 SCK periods while a toggle flag
// crosses into the system clock domain; the byte is copied when the toggle
// arrives. Every byte carries a `first` flag (first byte since CS went low),
// so the command parser never needs CS itself.
//
// Transmit: MISO shifts out the 64-bit `status` that was captured at the end
// of the previous transaction (MSB first, changes on falling SCK). Reads are
// meant for slow status polls (<= ~10-20 MHz).
//
// CS high resets the SCK-domain bit counters asynchronously.
//
// ---- Why the "two clock domains" care ---------------------------------------
// The SPI bits arrive on the ESP32's clock (SCK); the rest of the FPGA runs on
// its own 57 MHz clock. The two are unrelated, so a value copied from one to
// the other at the wrong instant can be caught half-changed. The safe pattern
// used here: when a byte is complete, store it and flip ONE bit (rx_tog). The
// system side copies that single bit through 3 registers (tog_s) to let it
// settle, and when it sees the bit change, it reads the byte - which by then
// has been sitting still for a while and won't change for 8 more SCK cycles.
// It's like a mailbox flag: put the letter in first, then raise the flag.
module spi_slave (
  input  wire        clk,
  input  wire        rst,
  input  wire        sck,
  input  wire        mosi,
  input  wire        cs_n,
  output wire        miso,
  output wire        miso_oe,
  input  wire [63:0] status,
  output reg         byte_valid,   // one clock per received byte
  output reg  [7:0]  byte_data,
  output reg         byte_first,
  output reg         xfer_end      // one clock, after the last byte of a transaction
);
  // ---- SCK domain -----------------------------------------------------------
  reg [2:0] bitcnt;
  reg       seen;
  reg [6:0] sh;
  reg [7:0] rx_byte;
  reg       rx_first;
  reg       rx_tog;

  initial begin
    rx_tog = 1'b0; rx_byte = 8'd0; rx_first = 1'b0; sh = 7'd0;
  end

  always @(posedge sck or posedge cs_n) begin
    if (cs_n) begin
      bitcnt <= 3'd0;
      seen   <= 1'b0;
    end else begin
      bitcnt <= bitcnt + 3'd1;
      if (bitcnt == 3'd7) seen <= 1'b1;
    end
  end

  always @(posedge sck) begin
    if (!cs_n) begin
      sh <= {sh[5:0], mosi};
      if (bitcnt == 3'd7) begin
        rx_byte  <= {sh, mosi};
        rx_first <= ~seen;
        rx_tog   <= ~rx_tog;
      end
    end
  end

  // MISO: bit index counts falling edges since CS went low
  reg [6:0] mcnt;
  always @(negedge sck or posedge cs_n) begin
    if (cs_n) mcnt <= 7'd0;
    else if (!mcnt[6]) mcnt <= mcnt + 7'd1;
  end
  reg [63:0] status_hold;
  assign miso    = mcnt[6] ? 1'b0 : status_hold[~mcnt[5:0]];
  assign miso_oe = ~cs_n;

  // ---- system clock domain ------------------------------------------------
  reg [2:0] tog_s;
  reg [1:0] cs_s;
  reg       cs_prev;
  reg [9:0] end_dly;   // CS rise, delayed 10 clocks so every byte is delivered first

  initial begin
    tog_s = 3'd0; cs_s = 2'b11; cs_prev = 1'b1; end_dly = 10'd0; status_hold = 64'd0;
  end

  wire tog_edge = tog_s[2] ^ tog_s[1];
  wire cs_rise  = cs_s[1] & ~cs_prev;

  always @(posedge clk) begin
    tog_s   <= {tog_s[1:0], rx_tog};
    cs_s    <= {cs_s[0], cs_n};
    cs_prev <= cs_s[1];
    if (tog_edge) begin
      byte_data  <= rx_byte;
      byte_first <= rx_first;
    end
    if (end_dly[9]) status_hold <= status;
    if (rst) begin
      byte_valid <= 1'b0;
      xfer_end   <= 1'b0;
      end_dly    <= 10'd0;
    end else begin
      byte_valid <= tog_edge;
      end_dly    <= {end_dly[8:0], cs_rise};
      xfer_end   <= end_dly[9];
    end
  end
endmodule
