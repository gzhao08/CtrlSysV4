set script_dir [file normalize [file dirname [info script]]]
set project_path [file normalize [file join $script_dir .. .. Vivado_CtrlSysV4 Vivado_CtrlSysV4.xpr]]
set repo_root [file normalize [file join $script_dir .. ..]]
set bit_src [file normalize [file join $repo_root Vivado_CtrlSysV4 Vivado_CtrlSysV4.runs impl_1 design_1_wrapper.bit]]
set bit_dst [file normalize [file join $repo_root build design_1_wrapper.bit]]

open_project $project_path
update_ip_catalog -rebuild

set core_ips [get_ips -all -quiet *ctrlsys_core*]
if {[llength $core_ips] > 0} {
    if {[catch {upgrade_ip $core_ips} message]} {
        puts "IP upgrade note: $message"
    }

    foreach core_ip $core_ips {
        catch {reset_target all $core_ip}
        catch {generate_target all $core_ip}
    }
}

set core_runs [get_runs -quiet *ctrlsys_core*_synth_1]
foreach core_run $core_runs {
    reset_run $core_run
    launch_runs $core_run -jobs 4
}
foreach core_run $core_runs {
    wait_on_run $core_run
    set core_status [get_property STATUS [get_runs $core_run]]
    puts "$core_run status: $core_status"
    if {![string match "synth_design Complete*" $core_status]} {
        error "$core_run did not complete: $core_status"
    }
}

reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set status [get_property STATUS [get_runs impl_1]]
puts "Implementation status: $status"
if {![string match "write_bitstream Complete*" $status]} {
    error "Bitstream generation did not complete: $status"
}

file mkdir [file dirname $bit_dst]
file copy -force $bit_src $bit_dst
puts "Copied bitstream: $bit_dst"
close_project
