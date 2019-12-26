set thepart xc7z015clg485-1
set hdl [file normalize [file dirname [info script]]/..]

proc new {} {
	global thepart
	close_project -quiet
	create_project -in_memory -part $thepart
}

proc here {} {
	file dirname [info script]
}

proc checktarget {} {
	global target
	
	if {![info exists target]} {
		set target [here]/build
	}
	file mkdir $target
}

proc synth {top {flatten_hierarchy none} {mode default}} {
	global target thepart
	checktarget

	synth_design -top $top -part $thepart -flatten_hierarchy $flatten_hierarchy -mode $mode
	write_checkpoint -force $target/post_synth
}

proc implement {} {
	global target
	checktarget
	
	opt_design
	place_design
	write_checkpoint -force $target/post_place
	phys_opt_design
	route_design
	write_checkpoint -force $target/post_route
	
	report_timing_summary -file $target/post_route_timing.rpt
	report_utilization -file $target/post_route_util.rpt
}

proc bitstream {} {
	global target
	checktarget
	write_bitstream -force -bin_file -file $target/out.bit
	
	if {! [string match -nocase {*timing constraints are met*} \
	[report_timing_summary -no_header -no_detailed_paths -return_string]]} \
	{puts " **** timing constraints not met ****"}
}

proc notb {args} {
	set y {}
	foreach x [glob {*}$args] {
		if { ! [regexp {_tb\.v$} $x ] } {
			lappend y $x
		}
	}
	return $y
}
