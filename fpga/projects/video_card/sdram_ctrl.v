// SDRAM controller for the frame buffers (64 Mbit, 16-bit, 4 banks x 4096
// rows x 256 columns; MAX1000 default fit). Runs on the 57.27 MHz system clock.
//
// The memory only has three jobs, so the controller is a fixed, simple
// sequencer instead of a general-purpose one:
//   1. line fetch: read one picture line (180 words = 360 pixels) into the
//      line buffer, once per video line
//   2. refresh: one AUTO REFRESH every REF_INTERVAL clocks
//   3. pixel writes: drain the write FIFO in batches, one byte per clock,
//      using DQM byte masks (one picture line = one SDRAM row)
// Every operation opens a row, works on it and closes it again (precharge),
// so all banks are idle between operations. Burst length is 1: one READ or
// WRITE command per clock, which keeps the bookkeeping trivial.
//
// Clocking: the SDRAM clock pin gets the system clock shifted 180 degrees
// (PLL c1). Commands change on the rising system edge and are sampled by the
// SDRAM half a clock later. Read data is captured on the FALLING system edge
// (the middle of the data-valid window for CL=2/3 at this clock rate) and then
// re-registered on the rising edge. Result: read data for a READ issued at
// edge n is in dq_r after edge n + CL + 1  (RD_LAT below).
// sim/sdram_model.v checks all of this, including the timing window.
//
// ---- SDRAM in plain words --------------------------------------------------
// The chip is like a 2-D list: mem[row][column], 4096 rows x 256 columns of
// 16-bit words (per bank; we only use bank 0). You can't read it directly:
//   1. ACTIVE (open) a row: copies that whole row into the chip's "row buffer".
//      Takes T_RCD clocks before you may use it.
//   2. READ / WRITE columns of the open row, one per clock.
//   3. PRECHARGE (close) the row: writes it back. Takes T_RP clocks.
// The cells are tiny capacitors that leak, so every row must be REFRESHed
// every 64 ms; AUTO REFRESH does a few rows at a time, 4096 times per 64 ms.
// A "command" is just a pattern on the CS/RAS/CAS/WE pins for one clock
// (table below); the other clocks get NOP (do nothing).
//
// ---- How the state machine reads ------------------------------------------
// `state` says what we're doing; each clock tick runs the `case` branch for
// the current state, which issues at most one command and picks the next
// state. Python sketch of the same idea:
//     while True:                       # one loop iteration = one clock
//         if state == IDLE:
//             if refresh_due: send(REF); wait(T_RFC); state = IDLE
//             elif line_wanted: send(ACT, row); wait(T_RCD); state = READ
//             elif fifo_has_work: send(ACT, row); wait(T_RCD); state = WRITE
//         elif state == READ:
//             send(READ, col); col += 1
//             if col == 180: state = DRAIN      # then PRECHARGE, back to IDLE
//         ...
// `go_wait(n, next)` is the wait(): it parks in S_WAIT so the next command
// goes out exactly n clocks after this one.
module sdram_ctrl #(
  parameter CL           = 2,       // CAS latency (2 or 3; both fine at 57 MHz)
  parameter INIT_WAIT    = 14000,   // NOP clocks after power-up (>= 200 us = 11455)
  parameter REF_INTERVAL = 700,     // clocks per refresh: 12.2 us, so even a refresh delayed by
                                    // a line fetch + write batch stays under the 15.6 us spec
  parameter T_RP         = 3,       // clocks from PRECHARGE to next command (>= 20 ns)
  parameter T_RCD        = 3,       // ACTIVE to READ/WRITE (>= 20 ns)
  parameter T_RFC        = 6,       // REFRESH to next command (>= 70 ns)
  parameter T_MRD        = 3,       // MODE REGISTER SET to next command
  parameter T_WR         = 3,       // last WRITE to PRECHARGE (>= 15 ns or 2 clk)
  parameter LINE_WORDS   = 180,
  parameter BATCH_MIN    = 32,      // start a write batch at this many queued bytes...
  parameter AGE_MAX      = 400,     // ...or when the oldest byte has waited this long
  parameter BATCH_MAX    = 128      // longest write batch (bounds refresh/fetch latency)
) (
  input  wire        clk,
  input  wire        rst,
  output reg         init_done,

  // line fetch request (one per video line)
  input  wire        fetch_req,
  input  wire [8:0]  fetch_row,     // {buffer, picture line}
  input  wire        fetch_half,    // which half of the line buffer to fill
  output reg         fetch_overrun, // sticky: a request came before the last one finished
  input  wire        clear_err,
  output reg         lb_we,
  output reg  [8:0]  lb_waddr,
  output reg  [15:0] lb_wdata,

  // write FIFO (first-word-fall-through): {buf, line[7:0], col[7:0], hi, byte[7:0]}
  input  wire        wf_empty,
  input  wire [9:0]  wf_count,
  input  wire [25:0] wf_head,
  output wire        wf_pop,        // combinational: pops in the same clock as the WRITE
  input  wire        wf_flush,      // drain now (a buffer swap is waiting)
  output wire        wr_busy,

  // SDRAM pins (registered; place them in the I/O cells)
  output reg         sd_cs_n,
  output reg         sd_ras_n,
  output reg         sd_cas_n,
  output reg         sd_we_n,
  output reg  [1:0]  sd_ba,
  output reg  [11:0] sd_a,
  output reg  [1:0]  sd_dqm,
  inout  wire [15:0] sd_dq
);
  localparam RD_LAT = CL + 1;
  localparam [2:0] CL3 = CL;

  // commands {cs_n, ras_n, cas_n, we_n}
  localparam CMD_NOP   = 4'b0111;
  localparam CMD_ACT   = 4'b0011;
  localparam CMD_READ  = 4'b0101;
  localparam CMD_WRITE = 4'b0100;
  localparam CMD_PRE   = 4'b0010;
  localparam CMD_REF   = 4'b0001;
  localparam CMD_MRS   = 4'b0000;

  // mode register: burst length 1, sequential, CAS latency CL, single writes
  localparam [11:0] MODE = {2'b00, 1'b0, 2'b00, CL3, 1'b0, 3'b000};

  localparam S_INIT     = 4'd0;
  localparam S_INIT_PRE = 4'd1;
  localparam S_INIT_REF = 4'd2;
  localparam S_INIT_MRS = 4'd3;
  localparam S_IDLE     = 4'd4;
  localparam S_WAIT     = 4'd5;
  localparam S_READ     = 4'd6;
  localparam S_RD_DRAIN = 4'd7;
  localparam S_WRITE    = 4'd8;
  localparam S_PRE      = 4'd9;

  reg [3:0]  state, after;
  reg [15:0] wait_cnt;
  reg [3:0]  init_refs;

  // ---- data bus -----------------------------------------------------------
  reg [15:0] dq_out;
  reg        dq_oe;
  assign sd_dq = dq_oe ? dq_out : 16'bz;

  reg [15:0] dq_neg, dq_r;
  always @(negedge clk) dq_neg <= sd_dq;   // middle of the valid window
  always @(posedge clk) dq_r   <= dq_neg;

  // ---- refresh timer --------------------------------------------------------
  // Every REF_INTERVAL clocks, one more refresh is owed (ref_pending += 1);
  // the state machine pays them off when it's idle, before anything else.
  reg [10:0] ref_timer;
  reg [3:0]  ref_pending;
  reg        ref_take;
  always @(posedge clk) begin
    if (rst || !init_done) begin
      ref_timer   <= 11'd0;
      ref_pending <= 4'd0;
    end else begin
      if (ref_timer == REF_INTERVAL - 1) ref_timer <= 11'd0;
      else ref_timer <= ref_timer + 11'd1;
      case ({ref_timer == REF_INTERVAL - 1, ref_take})
        2'b10: if (ref_pending != 4'd15) ref_pending <= ref_pending + 4'd1;
        2'b01: ref_pending <= ref_pending - 4'd1;
        default: ;
      endcase
    end
  end

  // ---- fetch request latch ------------------------------------------------
  reg       fetch_pend;
  reg [8:0] f_row;
  reg       f_half;
  reg       fetch_take;
  always @(posedge clk) begin
    if (rst) begin
      fetch_pend    <= 1'b0;
      fetch_overrun <= 1'b0;
      f_row         <= 9'd0;
      f_half        <= 1'b0;
    end else if (fetch_req) begin
      if (fetch_pend && !fetch_take) fetch_overrun <= 1'b1;
      else if (clear_err) fetch_overrun <= 1'b0;
      fetch_pend <= 1'b1;
      f_row      <= fetch_row;
      f_half     <= fetch_half;
    end else begin
      if (fetch_take) fetch_pend <= 1'b0;
      if (clear_err) fetch_overrun <= 1'b0;
    end
  end

  // ---- write batching ------------------------------------------------------
  // Opening and closing a row costs ~8 clocks, so pixels are written in
  // batches: wait until 32 are queued (or the oldest has waited 400 clocks,
  // or a swap is waiting), then open the row and write all queued pixels for
  // that row, one per clock.
  reg [9:0] age;
  always @(posedge clk) begin
    if (rst || wf_empty) age <= 10'd0;
    else if (age != 10'h3FF) age <= age + 10'd1;
  end
  wire       wr_go   = !wf_empty && (wf_count >= BATCH_MIN || age >= AGE_MAX || wf_flush);
  wire [8:0] head_row = wf_head[25:17];

  // ---- read tag pipeline: which column each returning word belongs to -----
  // Read data comes back RD_LAT clocks after its READ command. So each READ
  // also drops its column number into this shift register (a conveyor belt,
  // like a fixed-length deque); when it falls off the end, the data arriving
  // at that moment belongs to that column and goes into the line buffer.
  reg [RD_LAT:0] rd_v;
  reg [7:0]      rd_col [0:RD_LAT];
  integer i;

  reg [7:0]  col;
  reg [8:0]  cur_row;
  reg        cur_half;
  reg [7:0]  batch_n;
  reg        writing;

  assign wr_busy = writing;

  // a WRITE goes out this clock when the FIFO head belongs to the open row
  wire do_write = (state == S_WRITE) && !wf_empty && head_row == cur_row && batch_n != BATCH_MAX;
  assign wf_pop = do_write;

  task issue(input [3:0] c);
    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= c;
  endtask

  // gap = clocks between this command and the next one (>= 2)
  task go_wait(input [15:0] gap, input [3:0] next);
    begin
      wait_cnt <= gap - 16'd2;
      after    <= next;
      state    <= S_WAIT;
    end
  endtask

  initial begin
    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} = CMD_NOP;
    dq_oe = 1'b0;
  end

  always @(posedge clk) begin
    // defaults: NOP, no data, no pops
    issue(CMD_NOP);
    dq_oe      <= 1'b0;
    sd_dqm     <= 2'b00;
    ref_take   <= 1'b0;
    fetch_take <= 1'b0;

    // returning read data -> line buffer
    rd_v[0]   <= 1'b0;
    rd_col[0] <= col;
    for (i = 1; i <= RD_LAT; i = i + 1) begin
      rd_v[i]   <= rd_v[i-1];
      rd_col[i] <= rd_col[i-1];
    end
    lb_we    <= rd_v[RD_LAT];
    lb_waddr <= {cur_half, rd_col[RD_LAT]};
    lb_wdata <= dq_r;

    if (rst) begin
      state     <= S_INIT;
      wait_cnt  <= INIT_WAIT;
      init_done <= 1'b0;
      init_refs <= 4'd0;
      writing   <= 1'b0;
      rd_v      <= {(RD_LAT+1){1'b0}};
      lb_we     <= 1'b0;
      sd_ba     <= 2'b00;
      sd_a      <= 12'd0;
    end else begin
      case (state)
        // power-up: NOPs, precharge all, 8 refreshes, load mode register
        S_INIT: begin
          if (wait_cnt == 0) state <= S_INIT_PRE;
          else wait_cnt <= wait_cnt - 16'd1;
        end
        S_INIT_PRE: begin
          issue(CMD_PRE);
          sd_a[10] <= 1'b1;                 // all banks
          go_wait(T_RP, S_INIT_REF);
        end
        S_INIT_REF: begin
          issue(CMD_REF);
          init_refs <= init_refs + 4'd1;
          go_wait(T_RFC, (init_refs == 4'd7) ? S_INIT_MRS : S_INIT_REF);
        end
        S_INIT_MRS: begin
          issue(CMD_MRS);
          sd_ba <= 2'b00;
          sd_a  <= MODE;
          go_wait(T_MRD, S_IDLE);
        end

        S_IDLE: begin
          init_done <= 1'b1;
          writing   <= 1'b0;
          if (ref_pending != 4'd0) begin
            issue(CMD_REF);
            ref_take <= 1'b1;
            go_wait(T_RFC, S_IDLE);
          end else if (fetch_pend) begin
            issue(CMD_ACT);
            sd_ba      <= 2'b00;
            sd_a       <= {3'b000, f_row};
            cur_half   <= f_half;
            fetch_take <= 1'b1;
            col        <= 8'd0;
            go_wait(T_RCD, S_READ);
          end else if (wr_go) begin
            issue(CMD_ACT);
            sd_ba   <= 2'b00;
            sd_a    <= {3'b000, head_row};
            cur_row <= head_row;
            batch_n <= 8'd0;
            writing <= 1'b1;
            go_wait(T_RCD, S_WRITE);
          end
        end

        // one READ per clock, columns 0..LINE_WORDS-1
        S_READ: begin
          issue(CMD_READ);
          sd_a      <= {4'b0000, col};      // A10 = 0: no auto-precharge
          rd_v[0]   <= 1'b1;
          rd_col[0] <= col;
          if (col == LINE_WORDS - 1) begin
            wait_cnt <= RD_LAT + 1;
            state    <= S_RD_DRAIN;
          end else begin
            col <= col + 8'd1;
          end
        end
        S_RD_DRAIN: begin                   // let the last words arrive
          if (wait_cnt == 0) state <= S_PRE;
          else wait_cnt <= wait_cnt - 16'd1;
        end

        // one WRITE per clock while the FIFO head is in this row
        S_WRITE: begin
          if (do_write) begin
            issue(CMD_WRITE);
            sd_a    <= {4'b0000, wf_head[16:9]};
            dq_out  <= {wf_head[7:0], wf_head[7:0]};
            dq_oe   <= 1'b1;
            sd_dqm  <= wf_head[8] ? 2'b01 : 2'b10;   // write only the addressed byte
            batch_n <= batch_n + 8'd1;
          end else begin
            go_wait(T_WR, S_PRE);
          end
        end

        S_PRE: begin
          issue(CMD_PRE);
          sd_a[10] <= 1'b1;
          go_wait(T_RP, S_IDLE);
        end

        S_WAIT: begin
          if (wait_cnt == 0) state <= after;
          else wait_cnt <= wait_cnt - 16'd1;
        end

        default: state <= S_IDLE;
      endcase
    end
  end
endmodule
