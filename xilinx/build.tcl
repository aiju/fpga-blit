source fns.tcl

new
read_verilog -sv [glob [here]/*.v] [glob [here]/../*.v]
read_verilog -sv [here]/../fx68k.sv
read_xdc [here]/top.xdc
synth top none
implement
bitstream
