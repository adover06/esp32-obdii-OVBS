// 12 MHz -> 57.2727 MHz (x105 / 22, VCO 1260 MHz).
//   c0 = system clock (4 x 14.31818 MHz, exact NTSC 4fsc multiple)
//   c1 = same frequency shifted 6.73 ns (139 degrees), sent out as the SDRAM
//        clock. The SDRAM samples commands roughly mid-period, and read data
//        lands centred on the controller's falling-edge capture register.
//        (180 degrees failed the slow-board corner in simulation by 0.6 ns;
//        139 passes fast, typical, slow and CL=3 corners - see sim/run.sh.)
// Simulation uses sim/pll_57m_sim.v instead of this file.
module pll_57m (
  input  wire inclk,
  output wire c0,
  output wire c1,
  output wire locked
);
  wire [4:0] clk;

  altpll #(
    .intended_device_family ("MAX 10"),
    .lpm_type               ("altpll"),
    .operation_mode         ("NORMAL"),
    .pll_type               ("AUTO"),
    .compensate_clock       ("CLK0"),
    .inclk0_input_frequency (83333),
    .clk0_multiply_by       (105),
    .clk0_divide_by         (22),
    .clk0_duty_cycle        (50),
    .clk0_phase_shift       ("0"),
    .clk1_multiply_by       (105),
    .clk1_divide_by         (22),
    .clk1_duty_cycle        (50),
    .clk1_phase_shift       ("6730"),
    .width_clock            (5),
    .port_inclk0            ("PORT_USED"),
    .port_clk0              ("PORT_USED"),
    .port_clk1              ("PORT_USED"),
    .port_clk2              ("PORT_UNUSED"),
    .port_clk3              ("PORT_UNUSED"),
    .port_clk4              ("PORT_UNUSED"),
    .port_locked            ("PORT_USED"),
    .port_areset            ("PORT_UNUSED")
  ) pll (
    .inclk  ({1'b0, inclk}),
    .clk    (clk),
    .locked (locked)
  );

  assign c0 = clk[0];
  assign c1 = clk[1];
endmodule
