#EMAC1
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_clk_tx}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_txd[0]}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_txd[1]}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_txd[2]}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_txd[3]}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_tx_oe}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_clk_rx}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_rxd[0]}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_rxd[1]}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_rxd[2]}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_rxd[3]}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_phy_rxdv}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_gmii_mdc}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_gmii_mdo_i}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|emac1_inst|emac_gmii_mdo_o}] #

#SDIO
set_false_path -to [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_cmd_o}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_cmd_i}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_o[0]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_o[1]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_o[2]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_o[3]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_i[0]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_i[1]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_i[2]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|sdio_inst|sdmmc_data_i[3]}] #

#USB1
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[0]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[1]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[2]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[3]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[4]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[5]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_o[6]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[0]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[1]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[2]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[3]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[4]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[5]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_data_i[6]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_clk}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_stp}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_dir}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|usb1_inst|usb_ulpi_nxt}]

#SPIM1
set_false_path -to [get_pins {u0|hps_0|hps_io|border|spim1_inst|spi_master_sclk}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|spim1_inst|spi_master_txd}]
set_false_path -from [get_pins {u0|hps_0|hps_io|border|spim1_inst|spi_master_rxd}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|spim1_inst|spi_master_ss_0_n}]

#UART0
set_false_path -from [get_pins {u0|hps_0|hps_io|border|uart0_inst|uart_rxd}]
set_false_path -to [get_pins {u0|hps_0|hps_io|border|uart0_inst|uart_txd}]

#I2C0
set_false_path -from [get_pins {u0|hps_0|hps_io|border|i2c0_inst|i2c_data}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|i2c0_inst|i2c_data}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|i2c0_inst|i2c_clk}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|i2c0_inst|i2c_clk}] #

#I2C1
set_false_path -from [get_pins {u0|hps_0|hps_io|border|i2c1_inst|i2c_data}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|i2c1_inst|i2c_data}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|i2c1_inst|i2c_clk}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|i2c1_inst|i2c_clk}] #

#GPIO
set_false_path -from [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio0_porta_o[9]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio0_porta_o[9]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[6]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[6]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[11]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[11]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[24]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[24]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[25]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio1_porta_o[25]}] #
set_false_path -from [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio2_porta_o[3]}] #
set_false_path -to [get_pins {u0|hps_0|hps_io|border|gpio_inst|gpio2_porta_o[3]}] #