// Behavioural SDR SDRAM (4 banks x 4096 rows x 256 columns x 16 bit) that
// checks the controller instead of trusting it.
//
// Checked: power-up wait, init sequence (precharge, refreshes, mode register),
// mode register contents, ACTIVE/READ/WRITE/PRECHARGE/REFRESH rules and
// timings (tRCD tRP tRAS tRC tWR tRFC tMRD), refresh rate, command/address/
// data setup and hold at the SDRAM pins, bus contention, X on inputs.
// Read data is driven only inside its real valid window
// [edge(CL-1) + tAC, edge(CL) + tOH] and is X outside it, so a controller
// that samples at the wrong time reads garbage and the picture check fails.
//
// Timings (ns) are the slower of the common 64 Mbit parts the board uses
// (Winbond W9864G6JT-6, ISSI IS42S16400J-7).
`timescale 1ps/1ps
module sdram_model #(
  parameter CL         = 2,
  parameter T_AC       = 6000,      // access time from clock, CL=2/3 (ps)
  parameter T_OH       = 2500,      // output hold
  parameter T_IS       = 1500,      // input setup
  parameter T_IH       = 800,       // input hold
  parameter T_POWERUP  = 200000000, // 200 us
  parameter MAX_ROWS   = 1024       // rows modelled per bank (the design uses < 960)
) (
  input  wire        clk,
  input  wire        cke,
  input  wire        cs_n,
  input  wire        ras_n,
  input  wire        cas_n,
  input  wire        we_n,
  input  wire [1:0]  ba,
  input  wire [11:0] a,
  input  wire [1:0]  dqm,
  inout  wire [15:0] dq,
  output reg         drv_en,      // for the testbench's delayed copy of the bus
  output reg  [15:0] drv_val
);
  localparam T_RCD = 20000, T_RP = 20000, T_RAS = 42000, T_RAS_MAX = 100000000;
  localparam T_RC  = 63000, T_WR = 15000, T_RFC = 63000, T_MRD_CLK = 2;
  localparam T_REFI = 15625000;     // 64 ms / 4096

  reg [15:0] mem [0:4*MAX_ROWS*256-1];
  integer errors = 0;
  integer reads = 0, writes = 0, refs = 0;
  time    max_ref_gap = 0;

  task fail(input [8*80-1:0] msg);
    begin
      errors = errors + 1;
      if (errors <= 20) $display("[SDRAM] ERROR @%0t ns: %0s", $time / 1000, msg);
    end
  endtask

  // ---- input setup/hold at the pins ---------------------------------------
  time last_edge = 0, last_in_change = 0, last_dq_change = 0;
  reg  last_edge_wrote = 0;
  always @(cs_n or ras_n or cas_n or we_n or ba or a or dqm) begin
    last_in_change = $time;
    if ($time > 0 && last_edge > 0 && $time - last_edge < T_IH) fail("command/address hold time");
  end
  always @(dq) begin
    last_dq_change = $time;
    if (last_edge_wrote && $time - last_edge < T_IH) fail("write data hold time");
  end

  // ---- state ----------------------------------------------------------------
  reg        open   [0:3];
  reg [11:0] row    [0:3];
  time       t_act  [0:3];
  time       t_pre  [0:3];
  time       t_wr   [0:3];
  time       t_ref = 0, t_mrs = 0, t_first_cmd = 0;
  integer    edges_since_mrs = 99;
  reg        mode_set = 0;
  integer    init_refs = 0;
  reg        init_pre = 0;
  integer    b, cl_set;
  time       clk_period = 0;
  reg [31:0] rd_seq = 0;

  initial begin
    for (b = 0; b < 4; b = b + 1) begin
      open[b] = 0; t_act[b] = 0; t_pre[b] = 0; t_wr[b] = 0;
    end
    drv_en = 0;
    drv_val = 16'hxxxx;
  end

  assign dq = drv_en ? drv_val : 16'bz;

  // read data timing (scheduled from the command edge)
  reg [15:0] rd_data_q;
  reg [31:0] z_check;
  always @(z_check) if (z_check == rd_seq) drv_en = 1'b0;

  wire [3:0] cmd = {cs_n, ras_n, cas_n, we_n};
  reg  [22:0] addr_of;
  reg  [11:0] row_q;

  always @(posedge clk) begin
    if (last_edge != 0) clk_period = $time - last_edge;
    last_edge = $time;
    last_edge_wrote = 0;
    if (mode_set) edges_since_mrs = edges_since_mrs + 1;

    if ((^cmd === 1'bx) || cke !== 1'b1) begin
      if ($time > 1000000) fail("X or CKE low on command pins");
    end else if (cmd != 4'b0111 && cs_n == 1'b0) begin
      // setup time for every real command
      if ($time - last_in_change < T_IS) fail("command/address setup time");
      if (t_first_cmd == 0) begin
        t_first_cmd = $time;
        if ($time < T_POWERUP) fail("command before the 200 us power-up wait");
      end
      if (mode_set && edges_since_mrs < T_MRD_CLK) fail("tMRD");

      case (cmd)
        4'b0011: begin // ACTIVE
          if (!mode_set) fail("ACTIVE before mode register set");
          if (open[ba]) fail("ACTIVE to an open bank");
          if ($time - t_pre[ba] < T_RP) fail("tRP before ACTIVE");
          if ($time - t_act[ba] < T_RC) fail("tRC");
          if ($time - t_ref < T_RFC) fail("tRFC before ACTIVE");
          if (a >= MAX_ROWS) fail("row outside the modelled range");
          open[ba] = 1; row[ba] = a; t_act[ba] = $time;
        end
        4'b0101, 4'b0100: begin // READ / WRITE
          if (!open[ba]) fail("READ/WRITE to a closed bank");
          if ($time - t_act[ba] < T_RCD) fail("tRCD");
          if (a[10]) fail("auto-precharge not expected");
          if (^a[7:0] === 1'bx) fail("X column");
          row_q   = row[ba];
          addr_of = {ba, row_q[9:0], a[7:0]};   // must cover every row bit the design uses (MAX_ROWS)
          if (cmd == 4'b0101) begin
            reads = reads + 1;
            if (dqm !== 2'b00) fail("DQM set during READ");
            rd_data_q = mem[addr_of];
            rd_seq = rd_seq + 1;
            // valid from edge(CL-1)+tAC to edge(CL)+tOH, X in between reads
            drv_val <= #((cl_set - 1) * clk_period + T_AC) rd_data_q;
            drv_en  <= #((cl_set - 1) * clk_period + T_AC) 1'b1;
            drv_val <= #(cl_set * clk_period + T_OH) 16'hxxxx;
            z_check <= #(cl_set * clk_period + T_OH + 1) rd_seq;
          end else begin
            writes = writes + 1;
            last_edge_wrote = 1;
            if (drv_en) fail("WRITE while the SDRAM still drives the bus");
            if ($time - last_dq_change < T_IS) fail("write data setup time");
            if (dqm === 2'b11) fail("WRITE with both bytes masked");
            if (!dqm[0]) begin
              if (^dq[7:0] === 1'bx) fail("X write data (low byte)");
              mem[addr_of][7:0] = dq[7:0];
            end
            if (!dqm[1]) begin
              if (^dq[15:8] === 1'bx) fail("X write data (high byte)");
              mem[addr_of][15:8] = dq[15:8];
            end
            t_wr[ba] = $time;
          end
        end
        4'b0010: begin // PRECHARGE
          for (b = 0; b < 4; b = b + 1) begin
            if (a[10] || b == ba) begin
              if (open[b]) begin
                if ($time - t_act[b] < T_RAS) fail("tRAS (ACTIVE to PRECHARGE too short)");
                if ($time - t_act[b] > T_RAS_MAX) fail("row open longer than tRAS max");
                if ($time - t_wr[b] < T_WR) fail("tWR");
              end
              open[b] = 0; t_pre[b] = $time;
            end
          end
          if (!mode_set && a[10]) init_pre = 1;
        end
        4'b0001: begin // AUTO REFRESH
          for (b = 0; b < 4; b = b + 1) begin
            if (open[b]) fail("REFRESH with a bank open");
            if ($time - t_pre[b] < T_RP) fail("tRP before REFRESH");
          end
          if ($time - t_ref < T_RFC) fail("tRFC between refreshes");
          if (!mode_set) begin
            if (!init_pre) fail("init: REFRESH before PRECHARGE ALL");
            init_refs = init_refs + 1;
          end else begin
            refs = refs + 1;
            if (refs > 1 && $time - t_ref > max_ref_gap) max_ref_gap = $time - t_ref;
            // JEDEC allows postponing up to 8; this design never needs to
            if (refs > 1 && $time - t_ref > T_REFI) fail("refresh gap over 15.6 us");
          end
          t_ref = $time;
        end
        4'b0000: begin // MODE REGISTER SET
          for (b = 0; b < 4; b = b + 1) if (open[b]) fail("MRS with a bank open");
          if (init_refs < 2) fail("init: fewer than 2 refreshes before MRS");
          if (a[2:0] != 3'b000) fail("mode: burst length is not 1");
          if (a[3] != 1'b0) fail("mode: not sequential");
          if (a[6:4] != CL) fail("mode: CAS latency differs from the model's");
          if (a[8:7] != 2'b00) fail("mode: reserved bits");
          cl_set = a[6:4];
          mode_set = 1;
          edges_since_mrs = 0;
          t_mrs = $time;
        end
        default: fail("unsupported command (burst terminate?)");
      endcase
    end
  end

  // refresh rate: on average one every 15.625 us
  task report;
    begin
      if (mode_set && refs < ($time - t_mrs) / T_REFI - 8) fail("refresh rate too low on average");
      $display("[SDRAM] %0d reads, %0d writes, %0d refreshes, longest refresh gap %0d ns, errors %0d",
               reads, writes, refs, max_ref_gap / 1000, errors);
    end
  endtask
endmodule
