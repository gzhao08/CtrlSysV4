# Shared ICM SPI bus on the Red Pitaya expansion connector.
# NUM_ICM is 4 in config_pkg.sv, so all four MISO inputs are constrained even
# if only spi_miso_0[0] and spi_miso_0[1] are physically populated.
#
# DIO0_P: SCLK
# DIO0_N: MOSI
# DIO1_N: active-low CS
# DIO1_P: MISO[0]
# DIO2_P: MISO[1]
# DIO2_N: MISO[2]
# DIO3_P: MISO[3]

set_property PACKAGE_PIN G17 [get_ports spi_sclk_0]
set_property PACKAGE_PIN G18 [get_ports spi_mosi_0]
set_property PACKAGE_PIN H16 [get_ports {spi_miso_0[0]}]
set_property PACKAGE_PIN J18 [get_ports {spi_miso_0[1]}]
set_property PACKAGE_PIN H18 [get_ports {spi_miso_0[2]}]
set_property PACKAGE_PIN K17 [get_ports {spi_miso_0[3]}]
set_property PACKAGE_PIN H17 [get_ports spi_cs_n_0]

set_property IOSTANDARD LVCMOS33 [get_ports {spi_sclk_0 spi_mosi_0 spi_cs_n_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso_0[0] spi_miso_0[1] spi_miso_0[2] spi_miso_0[3]}]

set_property DRIVE 8 [get_ports {spi_sclk_0 spi_mosi_0 spi_cs_n_0}]
set_property SLEW FAST [get_ports {spi_sclk_0 spi_mosi_0 spi_cs_n_0}]
