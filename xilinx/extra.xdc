create_clock -name DPCLK -period 7.4 [get_nets dport0/dpclk]
set_false_path -from [get_clocks FCLK] -to [get_clocks DPCLK]
set_false_path -from [get_clocks DPCLK] -to [get_clocks FCLK]
