set script_dir [file normalize [file dirname [info script]]]
set project_path [file normalize [file join $script_dir .. .. Vivado_CtrlSysV4 Vivado_CtrlSysV4.xpr]]

open_project $project_path
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set status [get_property STATUS [get_runs impl_1]]
puts "Implementation status: $status"
if {![string match "write_bitstream Complete*" $status]} {
    error "Bitstream generation did not complete: $status"
}
close_project
