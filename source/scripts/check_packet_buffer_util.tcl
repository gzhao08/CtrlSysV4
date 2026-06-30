set script_dir [file dirname [file normalize [info script]]]
set hdl_dir [file normalize [file join $script_dir .. hdl]]

create_project -in_memory -part xc7z020clg400-1
read_verilog -sv [list \
    [file join $hdl_dir config_pkg.sv] \
    [file join $hdl_dir packet_buffer.sv]]

synth_design -top packet_buffer -mode out_of_context
report_utilization -hierarchical
exit
