set script_dir [file dirname [file normalize [info script]]]
set hdl_dir [file normalize [file join $script_dir .. hdl]]

create_project -in_memory -part xc7z020clg400-1
read_verilog -sv [list \
    [file join $hdl_dir config_pkg.sv] \
    [file join $hdl_dir acquisition_controller.sv] \
    [file join $hdl_dir axil_regs_slave_lite_v1_0_S00_AXI.v] \
    [file join $hdl_dir axil_regs.v] \
    [file join $hdl_dir stopwatch_64.sv] \
    [file join $hdl_dir ICM_reader.sv] \
    [file join $hdl_dir Intan_reader.sv] \
    [file join $hdl_dir packet_writer.sv] \
    [file join $hdl_dir SPI_mux.sv] \
    [file join $hdl_dir packet_buffer.sv] \
    [file join $hdl_dir packet_to_axis.sv] \
    [file join $hdl_dir ctrlsys_core.sv]]

synth_design -top ctrlsys_core
report_utilization -hierarchical
exit
