source fns.tcl

new
read_verilog -sv [notb [here]/*.v [here]/../*.v [here]/dport/*.v]
read_verilog -sv [here]/../fx68k.sv
read_xdc [here]/top.xdc
read_xdc [here]/extra.xdc
synth top none
implement
bitstream
