create_clock -name CLK12M -period 83.333 [get_ports {CLK12M}]
derive_clock_uncertainty
set_false_path -from [get_ports {USER_BTN}]
set_false_path -to [get_ports {LED[*]}]
