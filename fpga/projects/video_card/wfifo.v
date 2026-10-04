// Pixel write FIFO, 512 x 27 bits, first-word-fall-through (head always
// visible on `head`). Single clock. Uses block RAM.
//
// A new entry becomes visible 2 clocks after it is pushed; that keeps the
// RAM read (address registered every clock) from ever returning an entry in
// the same clock it is written.
//
// It's a ring buffer: wp (write pointer) and rp (read pointer) chase each other
// around a 512-entry array, like
//     push: mem[wp] = x; wp = (wp + 1) % 512
//     pop:  rp = (rp + 1) % 512        (head is always mem[rp])
// empty when rp == wp, full when wp + 1 == rp. The pointers are 9 bits, so
// the % 512 happens by itself when they overflow.
module wfifo (
  input  wire        clk,
  input  wire        rst,
  input  wire        push,
  input  wire [26:0] din,
  output wire        full,
  output reg         overflow,   // sticky: a push was dropped
  input  wire        clear_err,
  input  wire        pop,
  output reg  [26:0] head,
  output wire        empty,
  output wire [9:0]  count
);
  reg [26:0] mem [0:511];
  reg [8:0]  wp, wp_d1, wp_d2, rp;

  wire [8:0] wp_next = wp + 9'd1;
  wire [8:0] rp_next = (pop && !empty) ? rp + 9'd1 : rp;
  assign full  = (wp_next == rp);
  assign empty = (rp == wp_d2);
  assign count = {1'b0, wp_d2 - rp};

  always @(posedge clk) begin
    if (push && !full) mem[wp] <= din;
    head <= mem[rp_next];
  end

  always @(posedge clk) begin
    if (rst) begin
      wp <= 9'd0; wp_d1 <= 9'd0; wp_d2 <= 9'd0; rp <= 9'd0;
      overflow <= 1'b0;
    end else begin
      if (push) begin
        if (full) overflow <= 1'b1;
        else wp <= wp_next;
      end
      if (clear_err) overflow <= 1'b0;
      wp_d1 <= wp;
      wp_d2 <= wp_d1;
      rp    <= rp_next;
    end
  end
endmodule
