set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLUP [current_design]
create_clock -name FCLK -period 10.000 [get_pins {_PS7/FCLKCLK[0]}]
