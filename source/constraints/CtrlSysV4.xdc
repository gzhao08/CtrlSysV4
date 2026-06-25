# Single SPI sensor on the Red Pitaya expansion connector.
# DIO0_P: SCLK, DIO0_N: MOSI, DIO1_P: MISO, DIO1_N: active-low CS.

set_property PACKAGE_PIN G17 [get_ports spi_sclk_0]
set_property PACKAGE_PIN G18 [get_ports spi_mosi_0]
set_property PACKAGE_PIN H16 [get_ports {spi_miso_0[0]}]
set_property PACKAGE_PIN H17 [get_ports spi_cs_n_0]

set_property IOSTANDARD LVCMOS33 [get_ports {spi_sclk_0 spi_mosi_0 spi_cs_n_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso_0[0]}]

set_property DRIVE 8 [get_ports {spi_sclk_0 spi_mosi_0 spi_cs_n_0}]
set_property SLEW FAST [get_ports {spi_sclk_0 spi_mosi_0 spi_cs_n_0}]
