# Set AXI Quad SPI to a breakout-friendly clock rate.
set script_dir [file normalize [file dirname [info script]]]
set repo_root [file normalize [file join $script_dir ".." ".."]]
set project_path [file join $repo_root "Vivado_CtrlSysV4" "Vivado_CtrlSysV4.xpr"]
set bd_path [file join $repo_root "Vivado_CtrlSysV4" "Vivado_CtrlSysV4.srcs" \
    "sources_1" "bd" "design_1" "design_1.bd"]

open_project $project_path
update_ip_catalog -rebuild
set core_ips [get_ips -all -quiet *ctrlsys_core*]
if {[llength $core_ips] > 0} {
    upgrade_ip $core_ips
}
open_bd_design $bd_path

# The Red Pitaya loads this design as a PL-only bitstream, so the PS fabric
# clocks retain their Linux runtime rates. FCLK_CLK0 measures 125 MHz on the
# board; divide it by 16 * 8 for an approximately 976.6 kHz SPI clock.
set_property -dict [list \
    CONFIG.C_SCK_RATIO {16} \
    CONFIG.Multiples16 {8}] [get_bd_cells axi_quad_spi_0]

set ext_spi_clk [get_bd_pins axi_quad_spi_0/ext_spi_clk]
set old_net [get_bd_nets -quiet -of_objects $ext_spi_clk]
if {$old_net ne ""} {
    disconnect_bd_net $old_net $ext_spi_clk
}
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] $ext_spi_clk

validate_bd_design
save_bd_design

generate_target all [get_files $bd_path]
puts "AXI Quad SPI runtime clock: 125 MHz / (16 * 8) = 976.5625 kHz"
close_project
