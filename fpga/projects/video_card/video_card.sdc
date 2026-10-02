# MAX1000 video card timing.
create_clock -name CLK12M -period 83.333 [get_ports {CLK12M}]
derive_pll_clocks
derive_clock_uncertainty

# PLL outputs: clk[0] = system clock, clk[1] = copy shifted 6.73 ns for the SDRAM pin
set pll_c1 [get_pins -compatibility_mode {*pll1|clk[1]}]
create_generated_clock -name sdram_clk_pin -source $pll_c1 [get_ports {SDRAM_CLK}]

# SDRAM (W9864G6JT-6 / IS42S16400J-7 class): tIS 1.5, tIH 0.8, tAC 6.0, tOH 2.5 ns,
# plus ~0.3 ns of board trace each way
set sd_out [get_ports {SDRAM_CS_N SDRAM_RAS_N SDRAM_CAS_N SDRAM_WE_N SDRAM_BA[*] SDRAM_A[*] SDRAM_DQM[*] SDRAM_DQ[*]}]
set_output_delay -clock sdram_clk_pin -max  1.8 $sd_out
set_output_delay -clock sdram_clk_pin -min -0.8 $sd_out
set_input_delay  -clock sdram_clk_pin -max  6.6 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock sdram_clk_pin -min  2.5 [get_ports {SDRAM_DQ[*]}]
set_false_path -to [get_ports {SDRAM_CKE SDRAM_A[12] SDRAM_A[13]}]

# SPI clock from the ESP32 (up to 40 MHz); its domain is handed over with a
# toggle synchroniser, so it is asynchronous to everything else
create_clock -name spi_sck -period 25.0 [get_ports {SPI_SCK}]
set_clock_groups -asynchronous -group [get_clocks {spi_sck}] \
                               -group [get_clocks {CLK12M *pll1|clk[*] sdram_clk_pin}]
set_false_path -from [get_ports {SPI_MOSI SPI_CS_N USER_BTN}]
set_false_path -to [get_ports {SPI_MISO FPGA_READY LED[*] DAC[*]}]

# Read data is launched by the SDRAM clock edge CL-1 after the READ and is
# captured by the falling system edge one full period after that edge (the
# design's RD_LAT = CL+1; verified in sim/). Without this the analyzer picks
# the falling edge only ~2 ns later, which is not the one the logic uses.
set_multicycle_path -from [get_ports {SDRAM_DQ[*]}] -to [get_clocks {*pll1|clk[0]}] -setup -end 2
# No hold multicycle: the data changes every clock, so the default hold check
# (the next word must not arrive before this capture edge) is the right one.
