-- -----------------------------------------------------------------------
-- SOC-FPGA system
-- -----------------------------------------------------------------------

library IEEE;
    use IEEE.std_logic_1164.all;
    use IEEE.numeric_std.all;
use work.DAQ_Package.all;

use work.basic_package.all;
use work.FOOTpackage.all;

-- ----------------------------------------------
entity DE10DAQ_TOP is
------------------------------------------------
  port (
    -- Clocks
	 FPGA_CLK1_50    : in     std_logic; -- System clock 1 (50 MHz)
    FPGA_CLK2_50    : in     std_logic; -- System clock 2 (50 MHz)
    FPGA_CLK3_50    : in     std_logic; -- System clock 3 (50 MHz)

	 -- Keys
	 KEY : in std_logic_vector(1 downto 0);
	 -- Leds
    LED : out std_logic_vector(7 downto 0);
	 -- Switches
    SW  : in  std_logic_vector(3 downto 0);

	--Detector side A
    oNC_A           : out std_logic;  --GPIO1-16
    oFE_A_TEST      : out std_logic;  --GPIO1-0
    oFE_A_RESET     : out std_logic;  --GPIO1-2
    oFE_A0_HOLD     : out std_logic;  --GPIO1-4
    oFE_A0_SHIFT    : out std_logic;  --GPIO1-8
    oFE_A0_CLK      : out std_logic;  --GPIO1-12
    oFE_A1_HOLD     : out std_logic;  --GPIO1-6
    oFE_A1_SHIFT    : out std_logic;  --GPIO1-10
    oFE_A1_CLK      : out std_logic;  --GPIO1-14
    oADC_A_CS       : out std_logic;  --GPIO1-22
    oADC_A_SCLK     : out std_logic;  --GPIO1-24
    iADC_A_CS_RET   : in  std_logic;  --GPIO1-18
    iADC_A_SCK_RET  : in  std_logic;  --GPIO1-20
    iADC_A_SDATA0   : in  std_logic;  --GPIO1-26
    iADC_A_SDATA1   : in  std_logic;  --GPIO1-28
    iADC_A_SDATA2   : in  std_logic;  --GPIO1-30
    iADC_A_SDATA3   : in  std_logic;  --GPIO1-32
    iADC_A_SDATA4   : in  std_logic;  --GPIO1-34
    --Detector side B
    oNC_B           : out std_logic;  --GPIO1-17
    oFE_B_TEST      : out std_logic;  --GPIO1-3
    oFE_B_RESET     : out std_logic;  --GPIO1-1
    oFE_B0_HOLD     : out std_logic;  --GPIO1-5
    oFE_B0_SHIFT    : out std_logic;  --GPIO1-9
    oFE_B0_CLK      : out std_logic;  --GPIO1-13
    oFE_B1_HOLD     : out std_logic;  --GPIO1-7
    oFE_B1_SHIFT    : out std_logic;  --GPIO1-11
    oFE_B1_CLK      : out std_logic;  --GPIO1-15
    oADC_B_CS       : out std_logic;  --GPIO1-23
    oADC_B_SCLK     : out std_logic;  --GPIO1-25
    iADC_B_CS_RET   : in  std_logic;  --GPIO1-19
    iADC_B_SCK_RET  : in  std_logic;  --GPIO1-21
    iADC_B_SDATA0   : in  std_logic;  --GPIO1-27
    iADC_B_SDATA1   : in  std_logic;  --GPIO1-29
    iADC_B_SDATA2   : in  std_logic;  --GPIO1-31
    iADC_B_SDATA3   : in  std_logic;  --GPIO1-33
    iADC_B_SDATA4   : in  std_logic;  --GPIO1-35
    -- Central Acquisition side
    iBCO_CLK    : in  std_logic;      --GPIO0-16
    iBCO_CLK_n  : out  std_logic;     --GPIO0-18
    iBCO_RST    : in  std_logic;      --GPIO0-0
    iBCO_RST_n  : out  std_logic;     --GPIO0-2
    iEXT_TRIG   : in  std_logic;      --GPIO0-32
    iEXT_TRIG_n : out  std_logic;     --GPIO0-35
    oBUSY       : out std_logic;      --GPIO0-1
    oBUSY_n     : out std_logic;      --GPIO0-4

    oHK         : out std_logic_vector(27 downto 0);  --All the remainings

	 -- ADC
    ADC_CONVST : out std_logic;
    ADC_SCK    : out std_logic;
    ADC_SDI    : out std_logic;
    ADC_SDO    : in std_logic;

	 -- HPS
    HPS_CONV_USB_N : inout std_logic;
    HPS_DDR3_ADDR : out std_logic_vector(14 downto 0);
    HPS_DDR3_BA : out std_logic_vector(2 downto 0);
    HPS_DDR3_CAS_N : out std_logic;
    HPS_DDR3_CKE : out std_logic;
    HPS_DDR3_CK_N : out std_logic;
    HPS_DDR3_CK_P : out std_logic;
    HPS_DDR3_CS_N : out std_logic;
    HPS_DDR3_DM : out std_logic_vector(3 downto 0);
    HPS_DDR3_DQ : inout std_logic_vector(31 downto 0);
    HPS_DDR3_DQS_N : inout std_logic_vector(3 downto 0);
    HPS_DDR3_DQS_P : inout std_logic_vector(3 downto 0);
    HPS_DDR3_ODT : out std_logic;
    HPS_DDR3_RAS_N : out std_logic;
    HPS_DDR3_RESET_N : out std_logic;
    HPS_DDR3_RZQ : in std_logic;
    HPS_DDR3_WE_N : out std_logic;
    HPS_ENET_GTX_CLK : out std_logic;
    HPS_ENET_INT_N : inout std_logic;
    HPS_ENET_MDC : out std_logic;
    HPS_ENET_MDIO : inout std_logic;
    HPS_ENET_RX_CLK : in std_logic;
    HPS_ENET_RX_DATA : in std_logic_vector(3 downto 0);
    HPS_ENET_RX_DV : in std_logic;
    HPS_ENET_TX_DATA : out std_logic_vector(3 downto 0);
    HPS_ENET_TX_EN : out std_logic;
    HPS_GSENSOR_INT : inout std_logic;
    HPS_I2C0_SCLK : inout std_logic;
    HPS_I2C0_SDAT : inout std_logic;
    HPS_I2C1_SCLK : inout std_logic;
    HPS_I2C1_SDAT : inout std_logic;
    HPS_KEY : inout std_logic;
    HPS_LED : inout std_logic;
    HPS_LTC_GPIO : inout std_logic;
    HPS_SD_CLK : out std_logic;
    HPS_SD_CMD : inout std_logic;
    HPS_SD_DATA : inout std_logic_vector(3 downto 0);
    HPS_SPIM_CLK : out std_logic;
    HPS_SPIM_MISO : in std_logic;
    HPS_SPIM_MOSI : out std_logic;
    HPS_SPIM_SS : inout std_logic;
    HPS_UART_RX : in std_logic;
    HPS_UART_TX : out std_logic;
    HPS_USB_CLKOUT : in std_logic;
    HPS_USB_DATA : inout std_logic_vector(7 downto 0);
    HPS_USB_DIR : in std_logic;
    HPS_USB_NXT : in std_logic;
    HPS_USB_STP : out std_logic
  );
end entity DE10DAQ_TOP;

-----------------------------------------------------------------
architecture structural of DE10DAQ_TOP is
-----------------------------------------------------------------
  -- system signals
  signal Clock, Reset, nReset : std_logic := '0';
  constant nResetFixed : std_logic := '1';
  signal nColdReset, nWarmReset, nDebugReset : std_logic :='0';


   -- GPIO connections
  signal sFeA       : tFpga2FeIntf;
  signal sFeB       : tFpga2FeIntf;
  signal sAdcA      : tFpga2AdcIntf;
  signal sAdcB      : tFpga2AdcIntf;
  signal sMultiAdc  : tMultiAdc2FpgaIntf;

  signal sBcoClkSynch : std_logic;
  signal sBcoRstSynch : std_logic;
  signal sExtTrgSynch : std_logic;
  signal sBusy : std_logic;
  signal sErrors : std_logic;
  signal sDebug : std_logic_vector(7 downto 0);



  -- IO HPS-FPGA
  signal stm_hwevents  : std_logic_vector(27 downto 0);
  signal ledreg  : std_logic_vector(7 downto 0);
  signal KEYE : std_logic_vector(3 downto 0) := "0000";
  signal SWD : std_logic_vector(3 downto 0) := "0000";

  -- for fifo IO
  signal iofifo_datain    :  std_logic_vector(31 downto 0);                       -- datain
  signal iofifo_writereq  :  std_logic;                                           -- writereq
  signal iofifo_instatus  :  std_logic_vector(31 downto 0) := (others => 'X');    -- instatus
  signal iofifo_dataout   :  std_logic_vector(31 downto 0) := (others => 'X');    -- dataout
  signal iofifo_readack   :  std_logic;                                           -- readack
  signal iofifo_outstatus :  std_logic_vector(31 downto 0) := (others => 'X');    -- outstatus
  signal iofifo_regdataout   :  std_logic_vector(31 downto 0) := (others => 'X'); -- regdataout
  signal iofifo_regreadack   :  std_logic;                                        -- regreadack
  signal iofifo_regoutstatus :  std_logic_vector(31 downto 0) := (others => 'X'); -- regoutstatus

  -- for RAM module
  signal fpga_side_RAM_ctrl_reg, hps_side_RAM_ctrl_reg : std_logic_vector(31 downto 0) := (others => 'X');
  signal SDRAM_interface_enable, fifo_readack_from_RAM, event_fifo_empty : std_logic := '0';

  -- ADC
  signal adc_raw_values : adc_values_t := (others => "000000000000");

  -- ADC controller IP
  component ADC_controller_IP is
    port (
      CLOCK    : in  std_logic                     := 'X'; -- clk
      RESET    : in  std_logic                     := 'X'; -- reset
      CH0      : out std_logic_vector(11 downto 0);        -- CH0
      CH1      : out std_logic_vector(11 downto 0);        -- CH1
      CH2      : out std_logic_vector(11 downto 0);        -- CH2
      CH3      : out std_logic_vector(11 downto 0);        -- CH3
      CH4      : out std_logic_vector(11 downto 0);        -- CH4
      CH5      : out std_logic_vector(11 downto 0);        -- CH5
      CH6      : out std_logic_vector(11 downto 0);        -- CH6
      CH7      : out std_logic_vector(11 downto 0);        -- CH7
      ADC_SCLK : out std_logic;                            -- SCLK
      ADC_CS_N : out std_logic;                            -- CS_N
      ADC_DOUT : in  std_logic                     := 'X'; -- DOUT
      ADC_DIN  : out std_logic                             -- DIN
    );
  end component ADC_controller_IP;

  component soc_system is
    port (
      button_pio_external_connection_export : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
      clk_clk                               : in    std_logic                     := 'X';             -- clk
      dipsw_pio_external_connection_export  : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
      hps_0_f2h_cold_reset_req_reset_n      : in    std_logic                     := 'X';             -- reset_n
      hps_0_f2h_debug_reset_req_reset_n     : in    std_logic                     := 'X';             -- reset_n
      hps_0_f2h_stm_hw_events_stm_hwevents  : in    std_logic_vector(27 downto 0) := (others => 'X'); -- stm_hwevents
      hps_0_f2h_warm_reset_req_reset_n      : in    std_logic                     := 'X';             -- reset_n
      hps_0_h2f_reset_reset_n               : out   std_logic;                                        -- reset_n
      hps_0_hps_io_hps_io_emac1_inst_TX_CLK : out   std_logic;                                        -- hps_io_emac1_inst_TX_CLK
      hps_0_hps_io_hps_io_emac1_inst_TXD0   : out   std_logic;                                        -- hps_io_emac1_inst_TXD0
      hps_0_hps_io_hps_io_emac1_inst_TXD1   : out   std_logic;                                        -- hps_io_emac1_inst_TXD1
      hps_0_hps_io_hps_io_emac1_inst_TXD2   : out   std_logic;                                        -- hps_io_emac1_inst_TXD2
      hps_0_hps_io_hps_io_emac1_inst_TXD3   : out   std_logic;                                        -- hps_io_emac1_inst_TXD3
      hps_0_hps_io_hps_io_emac1_inst_RXD0   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD0
      hps_0_hps_io_hps_io_emac1_inst_MDIO   : inout std_logic                     := 'X';             -- hps_io_emac1_inst_MDIO
      hps_0_hps_io_hps_io_emac1_inst_MDC    : out   std_logic;                                        -- hps_io_emac1_inst_MDC
      hps_0_hps_io_hps_io_emac1_inst_RX_CTL : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CTL
      hps_0_hps_io_hps_io_emac1_inst_TX_CTL : out   std_logic;                                        -- hps_io_emac1_inst_TX_CTL
      hps_0_hps_io_hps_io_emac1_inst_RX_CLK : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CLK
      hps_0_hps_io_hps_io_emac1_inst_RXD1   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD1
      hps_0_hps_io_hps_io_emac1_inst_RXD2   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD2
      hps_0_hps_io_hps_io_emac1_inst_RXD3   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD3

      hps_0_hps_io_hps_io_sdio_inst_CMD     : inout std_logic                     := 'X';             -- hps_io_sdio_inst_CMD0
      hps_0_hps_io_hps_io_sdio_inst_D0      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D0
      hps_0_hps_io_hps_io_sdio_inst_D1      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D1
      hps_0_hps_io_hps_io_sdio_inst_CLK     : out   std_logic;                                        -- hps_io_sdio_inst_CLK
      hps_0_hps_io_hps_io_sdio_inst_D2      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D2
      hps_0_hps_io_hps_io_sdio_inst_D3      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D3
      hps_0_hps_io_hps_io_usb1_inst_D0      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D0
      hps_0_hps_io_hps_io_usb1_inst_D1      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D1
      hps_0_hps_io_hps_io_usb1_inst_D2      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D2
      hps_0_hps_io_hps_io_usb1_inst_D3      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D3
      hps_0_hps_io_hps_io_usb1_inst_D4      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D4
      hps_0_hps_io_hps_io_usb1_inst_D5      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D5
      hps_0_hps_io_hps_io_usb1_inst_D6      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D6
      hps_0_hps_io_hps_io_usb1_inst_D7      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D7
      hps_0_hps_io_hps_io_usb1_inst_CLK     : in    std_logic                     := 'X';             -- hps_io_usb1_inst_CLK
      hps_0_hps_io_hps_io_usb1_inst_STP     : out   std_logic;                                        -- hps_io_usb1_inst_STP
      hps_0_hps_io_hps_io_usb1_inst_DIR     : in    std_logic                     := 'X';             -- hps_io_usb1_inst_DIR
      hps_0_hps_io_hps_io_usb1_inst_NXT     : in    std_logic                     := 'X';             -- hps_io_usb1_inst_NXT
      hps_0_hps_io_hps_io_spim1_inst_CLK    : out   std_logic;                                        -- hps_io_spim1_inst_CLK
      hps_0_hps_io_hps_io_spim1_inst_MOSI   : out   std_logic;                                        -- hps_io_spim1_inst_MOSI
      hps_0_hps_io_hps_io_spim1_inst_MISO   : in    std_logic                     := 'X';             -- hps_io_spim1_inst_MISO
      hps_0_hps_io_hps_io_spim1_inst_SS0    : out   std_logic;                                        -- hps_io_spim1_inst_SS0
      hps_0_hps_io_hps_io_uart0_inst_RX     : in    std_logic                     := 'X';             -- hps_io_uart0_inst_RX
      hps_0_hps_io_hps_io_uart0_inst_TX     : out   std_logic;                                        -- hps_io_uart0_inst_TX
      hps_0_hps_io_hps_io_i2c0_inst_SDA     : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SDA
      hps_0_hps_io_hps_io_i2c0_inst_SCL     : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SCL
      hps_0_hps_io_hps_io_i2c1_inst_SDA     : inout std_logic                     := 'X';             -- hps_io_i2c1_inst_SDA
      hps_0_hps_io_hps_io_i2c1_inst_SCL     : inout std_logic                     := 'X';             -- hps_io_i2c1_inst_SCL
      hps_0_hps_io_hps_io_gpio_inst_GPIO09  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO09
      hps_0_hps_io_hps_io_gpio_inst_GPIO35  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO35
      hps_0_hps_io_hps_io_gpio_inst_GPIO40  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO40
      hps_0_hps_io_hps_io_gpio_inst_GPIO53  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO53
      hps_0_hps_io_hps_io_gpio_inst_GPIO54  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO54
      hps_0_hps_io_hps_io_gpio_inst_GPIO61  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO61
      led_pio_external_connection_export    : out   std_logic_vector(7 downto 0);                     -- export
	   version_pio_external_connection_export : in    std_logic_vector(31 downto 0) := (others => 'X');  -- export
      memory_mem_a                          : out   std_logic_vector(14 downto 0);                    -- mem_a
      memory_mem_ba                         : out   std_logic_vector(2 downto 0);                     -- mem_ba
      memory_mem_ck                         : out   std_logic;                                        -- mem_ck
      memory_mem_ck_n                       : out   std_logic;                                        -- mem_ck_n
      memory_mem_cke                        : out   std_logic;                                        -- mem_cke
      memory_mem_cs_n                       : out   std_logic;                                        -- mem_cs_n
      memory_mem_ras_n                      : out   std_logic;                                        -- mem_ras_n
      memory_mem_cas_n                      : out   std_logic;                                        -- mem_cas_n
      memory_mem_we_n                       : out   std_logic;                                        -- mem_we_n
      memory_mem_reset_n                    : out   std_logic;                                        -- mem_reset_n
      memory_mem_dq                         : inout std_logic_vector(31 downto 0) := (others => 'X'); -- mem_dq
      memory_mem_dqs                        : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs
      memory_mem_dqs_n                      : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs_n
      memory_mem_odt                        : out   std_logic;                                        -- mem_odt
      memory_mem_dm                         : out   std_logic_vector(3 downto 0);                     -- mem_dm
      memory_oct_rzqin                      : in    std_logic                     := 'X';             -- oct_rzqin

      reset_reset_n                         : in    std_logic                     := 'X';             -- reset_n
      iofifo_datain                         : out   std_logic_vector(31 downto 0);                    -- datain
      iofifo_writereq                       : out   std_logic;                                        -- writereq
      iofifo_instatus                       : in    std_logic_vector(31 downto 0) := (others => 'X'); -- instatus
      iofifo_dataout                        : in    std_logic_vector(31 downto 0) := (others => 'X'); -- dataout
      iofifo_readack                        : out   std_logic;                                        -- readack
      iofifo_outstatus                      : in    std_logic_vector(31 downto 0) := (others => 'X'); -- outstatus
	   iofifo_regdataout                     : in    std_logic_vector(31 downto 0) := (others => 'X'); -- regdataout
      iofifo_regreadack                     : out   std_logic;                                        -- regreadack
      iofifo_regoutstatus                   : in    std_logic_vector(31 downto 0) := (others => 'X'); -- regoutstatus

      to_ram_ctrl_fpga_side_reg              : out   std_logic_vector(31 downto 0);                    -- fpga_side_reg
      to_ram_ctrl_hps_side_reg               : in    std_logic_vector(31 downto 0) := (others => 'X'); -- hps_side_reg
      to_ram_ctrl_enable                     : in    std_logic                     := 'X';             -- enable
      to_ram_fifo_data_ack                   : out   std_logic;                                        -- data_ack
      to_ram_fifo_data_event                 : in    std_logic_vector(31 downto 0) := (others => 'X'); -- data_event
      to_ram_fifo_data_empty                 : in    std_logic                     := 'X';              -- data_empty
      hps_fpga_ram_sync_gp_in                : in    std_logic_vector(31 downto 0) := (others => 'X'); -- gp_in
      hps_fpga_ram_sync_gp_out               : out   std_logic_vector(31 downto 0)                     -- gp_out
    );
  end component soc_system;

begin

  -- Main clock and resets
  Clock <= FPGA_CLK1_50; -- From external part
  Reset <= not(nReset);  -- From HPS



  -- GPIO connections ----------------------------------------------------------
  oNC_A           <= '0';
  iBCO_CLK_n      <= '0';
  iBCO_RST_n      <= '0';
  iEXT_TRIG_n     <= '0';
  oFE_A_TEST      <= sFeA.TestOn;
  oFE_A_RESET     <= sFeA.DRst;
  oFE_A0_HOLD     <=  not sFeA.Hold;
  oFE_A0_SHIFT    <= sFeA.ShiftIn;
  oFE_A0_CLK      <= not sFeA.Clk;
  oFE_A1_HOLD     <= sFeA.Hold;
  oFE_A1_SHIFT    <= sFeA.ShiftIn;
  oFE_A1_CLK      <= sFeA.Clk;
  oADC_A_CS       <= sAdcA.Cs;
  oADC_A_SCLK     <= sAdcA.Sclk;
  sMultiAdc(0).SData <= iADC_A_SDATA0;
  sMultiAdc(1).SData <= iADC_A_SDATA1;
  sMultiAdc(2).SData <= iADC_A_SDATA2;
  sMultiAdc(3).SData <= iADC_A_SDATA3;
  sMultiAdc(4).SData <= iADC_A_SDATA4;
  --Detector side B
  oNC_B           <= '0';
  oFE_B_TEST      <= sFeB.TestOn;
  oFE_B_RESET     <= sFeB.DRst;
  oFE_B0_HOLD     <= not sFeB.Hold;
  oFE_B0_SHIFT    <= sFeB.ShiftIn;
  oFE_B0_CLK      <= not sFeB.Clk;
  oFE_B1_HOLD     <= sFeB.Hold;
  oFE_B1_SHIFT    <= sFeB.ShiftIn;
  oFE_B1_CLK      <= sFeB.Clk;
  oADC_B_CS       <= sAdcB.Cs;
  oADC_B_SCLK     <= sAdcB.Sclk;
  sMultiAdc(5).SData <= iADC_B_SDATA0;
  sMultiAdc(6).SData <= iADC_B_SDATA1;
  sMultiAdc(7).SData <= iADC_B_SDATA2;
  sMultiAdc(8).SData <= iADC_B_SDATA3;
  sMultiAdc(9).SData <= iADC_B_SDATA4;

  RET_GEN : for gg in 0 to 4 generate
    sMultiAdc(gg).clkRet   <= iADC_A_SCK_RET;
    sMultiAdc(gg).csRet    <= iADC_A_CS_RET;
    sMultiAdc(gg+5).clkRet <= iADC_B_SCK_RET;
    sMultiAdc(gg+5).csRet  <= iADC_B_CS_RET;
  end generate RET_GEN;

 -- Central Acquisition side
  BCO_CLK_SYNCH : sync_edge
    generic map (
      pSTAGES => 2
      )
    port map (
      iCLK    => Clock,
      iRST    => '0',
      iD      => iBCO_CLK,
      oQ      => sBcoClkSynch
      );

  BCO_RST_SYNCH : sync_edge
    generic map (
      pSTAGES => 2
      )
    port map (
      iCLK    => Clock,
      iRST    => '0',
      iD      => iBCO_RST,
      oQ      => sBcoRstSynch
      );

  EXT_TRG_SYNCH : sync_edge
    generic map (
      pSTAGES => 2
      )
    port map (
      iCLK    => Clock,
      iRST    => '0',
      iD      => iEXT_TRIG,
      oQ      => sExtTrgSynch
      );

  IOFFD : process(Clock)
  begin
    if rising_edge(clock) then
      oBUSY <= sBusy;
      oBUSY_n <= '0';
    end if;
  end process IOFFD;
  --
  --
  oHK(7 downto 0)   <= sDebug;
  oHK(27 downto 8)  <= sBcoClkSynch & sBcoRstSynch & sExtTrgSynch
                       & iofifo_readack & sErrors
                       & "000" & x"000";
  ------------------------------------------------------------------------------




  u0 : component soc_system
  port map (
    clk_clk                               => Clock,                                              --                            clk.clk
    reset_reset_n                         => nResetFixed,                                        --                          reset.reset_n
    hps_0_f2h_cold_reset_req_reset_n      => nColdReset,                                         --       hps_0_f2h_cold_reset_req.reset_n
    hps_0_f2h_debug_reset_req_reset_n     => nDebugReset,                                        --      hps_0_f2h_debug_reset_req.reset_n
    hps_0_f2h_stm_hw_events_stm_hwevents  => stm_hwevents,                                       --        hps_0_f2h_stm_hw_events.stm_hwevents
    hps_0_f2h_warm_reset_req_reset_n      => nWarmReset,                                         --       hps_0_f2h_warm_reset_req.reset_n
    hps_0_h2f_reset_reset_n               => nReset,                                             --                hps_0_h2f_reset.reset_n
	 -- HPS Ethernet
    hps_0_hps_io_hps_io_emac1_inst_TX_CLK => HPS_ENET_GTX_CLK,                                   --                   hps_0_hps_io.hps_io_emac1_inst_TX_CLK
    hps_0_hps_io_hps_io_emac1_inst_TXD0   => HPS_ENET_TX_DATA(0),                                --                               .hps_io_emac1_inst_TXD0
    hps_0_hps_io_hps_io_emac1_inst_TXD1   => HPS_ENET_TX_DATA(1),                                --                               .hps_io_emac1_inst_TXD1
    hps_0_hps_io_hps_io_emac1_inst_TXD2   => HPS_ENET_TX_DATA(2),                                --                               .hps_io_emac1_inst_TXD2
    hps_0_hps_io_hps_io_emac1_inst_TXD3   => HPS_ENET_TX_DATA(3),                                --                               .hps_io_emac1_inst_TXD3
    hps_0_hps_io_hps_io_emac1_inst_RXD0   => HPS_ENET_RX_DATA(0),                                --                               .hps_io_emac1_inst_RXD0
    hps_0_hps_io_hps_io_emac1_inst_MDIO   => HPS_ENET_MDIO,                                      --                               .hps_io_emac1_inst_MDIO
    hps_0_hps_io_hps_io_emac1_inst_MDC    => HPS_ENET_MDC,                                       --                               .hps_io_emac1_inst_MDC
    hps_0_hps_io_hps_io_emac1_inst_RX_CTL => HPS_ENET_RX_DV,                                     --                               .hps_io_emac1_inst_RX_CTL
    hps_0_hps_io_hps_io_emac1_inst_TX_CTL => HPS_ENET_TX_EN,                                     --                               .hps_io_emac1_inst_TX_CTL
    hps_0_hps_io_hps_io_emac1_inst_RX_CLK => HPS_ENET_RX_CLK,                                    --                               .hps_io_emac1_inst_RX_CLK
    hps_0_hps_io_hps_io_emac1_inst_RXD1   => HPS_ENET_RX_DATA(1),                                --                               .hps_io_emac1_inst_RXD1
    hps_0_hps_io_hps_io_emac1_inst_RXD2   => HPS_ENET_RX_DATA(2),                                --                               .hps_io_emac1_inst_RXD2
    hps_0_hps_io_hps_io_emac1_inst_RXD3   => HPS_ENET_RX_DATA(3),                                --                               .hps_io_emac1_inst_RXD3
	 -- HPS SD Card
    hps_0_hps_io_hps_io_sdio_inst_CMD     => HPS_SD_CMD,                                         --                               .hps_io_sdio_inst_CMD
    hps_0_hps_io_hps_io_sdio_inst_D0      => HPS_SD_DATA(0),                                     --                               .hps_io_sdio_inst_D0
    hps_0_hps_io_hps_io_sdio_inst_D1      => HPS_SD_DATA(1),                                     --                               .hps_io_sdio_inst_D1
    hps_0_hps_io_hps_io_sdio_inst_CLK     => HPS_SD_CLK,                                         --                               .hps_io_sdio_inst_CLK
    hps_0_hps_io_hps_io_sdio_inst_D2      => HPS_SD_DATA(2),                                     --                               .hps_io_sdio_inst_D2
    hps_0_hps_io_hps_io_sdio_inst_D3      => HPS_SD_DATA(3),                                     --                               .hps_io_sdio_inst_D3
	 -- HPS USB
    hps_0_hps_io_hps_io_usb1_inst_D0      => HPS_USB_DATA(0),                                    --                               .hps_io_usb1_inst_D0
    hps_0_hps_io_hps_io_usb1_inst_D1      => HPS_USB_DATA(1),                                    --                               .hps_io_usb1_inst_D1
    hps_0_hps_io_hps_io_usb1_inst_D2      => HPS_USB_DATA(2),                                    --                               .hps_io_usb1_inst_D2
    hps_0_hps_io_hps_io_usb1_inst_D3      => HPS_USB_DATA(3),                                    --                               .hps_io_usb1_inst_D3
    hps_0_hps_io_hps_io_usb1_inst_D4      => HPS_USB_DATA(4),                                    --                               .hps_io_usb1_inst_D4
    hps_0_hps_io_hps_io_usb1_inst_D5      => HPS_USB_DATA(5),                                    --                               .hps_io_usb1_inst_D5
    hps_0_hps_io_hps_io_usb1_inst_D6      => HPS_USB_DATA(6),                                    --                               .hps_io_usb1_inst_D6
    hps_0_hps_io_hps_io_usb1_inst_D7      => HPS_USB_DATA(7),                                    --                               .hps_io_usb1_inst_D7
    hps_0_hps_io_hps_io_usb1_inst_CLK     => HPS_USB_CLKOUT,                                     --                               .hps_io_usb1_inst_CLK
    hps_0_hps_io_hps_io_usb1_inst_STP     => HPS_USB_STP,                                        --                               .hps_io_usb1_inst_STP
    hps_0_hps_io_hps_io_usb1_inst_DIR     => HPS_USB_DIR,                                        --                               .hps_io_usb1_inst_DIR
    hps_0_hps_io_hps_io_usb1_inst_NXT     => HPS_USB_NXT,                                        --                               .hps_io_usb1_inst_NXT
	 -- HPS SPI
    hps_0_hps_io_hps_io_spim1_inst_CLK    => HPS_SPIM_CLK,                                       --                               .hps_io_spim1_inst_CLK
    hps_0_hps_io_hps_io_spim1_inst_MOSI   => HPS_SPIM_MOSI,                                      --                               .hps_io_spim1_inst_MOSI
    hps_0_hps_io_hps_io_spim1_inst_MISO   => HPS_SPIM_MISO,                                      --                               .hps_io_spim1_inst_MISO
    hps_0_hps_io_hps_io_spim1_inst_SS0    => HPS_SPIM_SS,                                        --                               .hps_io_spim1_inst_SS0
	 -- HPS UART
    hps_0_hps_io_hps_io_uart0_inst_RX     => HPS_UART_RX,                                        --                               .hps_io_uart0_inst_RX
    hps_0_hps_io_hps_io_uart0_inst_TX     => HPS_UART_TX,                                        --                               .hps_io_uart0_inst_TX
	 -- HPS I2C0
    hps_0_hps_io_hps_io_i2c0_inst_SDA     => HPS_I2C0_SDAT,                                      --                               .hps_io_i2c0_inst_SDA
    hps_0_hps_io_hps_io_i2c0_inst_SCL     => HPS_I2C0_SCLK,                                      --                               .hps_io_i2c0_inst_SCL
	 -- HPS I2C1
    hps_0_hps_io_hps_io_i2c1_inst_SDA     => HPS_I2C1_SDAT,                                      --                               .hps_io_i2c1_inst_SDA
    hps_0_hps_io_hps_io_i2c1_inst_SCL     => HPS_I2C1_SCLK,                                      --                               .hps_io_i2c1_inst_SCL
	 -- GPIO
    hps_0_hps_io_hps_io_gpio_inst_GPIO09  => HPS_CONV_USB_N,                                     --                               .hps_io_gpio_inst_GPIO09
    hps_0_hps_io_hps_io_gpio_inst_GPIO35  => HPS_ENET_INT_N,                                     --                               .hps_io_gpio_inst_GPIO35
    hps_0_hps_io_hps_io_gpio_inst_GPIO40  => HPS_LTC_GPIO,                                       --                               .hps_io_gpio_inst_GPIO40
    hps_0_hps_io_hps_io_gpio_inst_GPIO53  => HPS_LED,                                            --                               .hps_io_gpio_inst_GPIO53
    hps_0_hps_io_hps_io_gpio_inst_GPIO54  => HPS_KEY,                                            --                               .hps_io_gpio_inst_GPIO54
    hps_0_hps_io_hps_io_gpio_inst_GPIO61  => HPS_GSENSOR_INT,                                    --                               .hps_io_gpio_inst_GPIO61

	 -- HPS DDR3
    memory_mem_a                          => HPS_DDR3_ADDR,                                      --                         memory.mem_a
    memory_mem_ba                         => HPS_DDR3_BA,                                        --                               .mem_ba
    memory_mem_ck                         => HPS_DDR3_CK_P,                                      --                               .mem_ck
    memory_mem_ck_n                       => HPS_DDR3_CK_N,            				             --                               .mem_ck_n
    memory_mem_cke                        => HPS_DDR3_CKE,                                       --                               .mem_cke
    memory_mem_cs_n                       => HPS_DDR3_CS_N,                                      --                               .mem_cs_n
    memory_mem_ras_n                      => HPS_DDR3_RAS_N,                                     --                               .mem_ras_n
    memory_mem_cas_n                      => HPS_DDR3_CAS_N ,                                    --                               .mem_cas_n
    memory_mem_we_n                       => HPS_DDR3_WE_N,                                      --                               .mem_we_n
    memory_mem_reset_n                    => HPS_DDR3_RESET_N,                                   --                               .mem_reset_n
    memory_mem_dq                         => HPS_DDR3_DQ,                                        --                               .mem_dq
    memory_mem_dqs                        => HPS_DDR3_DQS_P,                                     --                               .mem_dqs
    memory_mem_dqs_n                      => HPS_DDR3_DQS_N,                                     --                               .mem_dqs_n
    memory_mem_odt                        => HPS_DDR3_ODT,                                       --                               .mem_odt
    memory_mem_dm                         => HPS_DDR3_DM,                                        --                               .mem_dm
    memory_oct_rzqin                      => HPS_DDR3_RZQ,                                       --                               .oct_rzqin

	 -- HPS Parallel I/O and user modules
    button_pio_external_connection_export => KEYE,      -- button_pio_external_connection.export
    dipsw_pio_external_connection_export  => SWD,       --  dipsw_pio_external_connection.export
    led_pio_external_connection_export    => ledreg,    --    led_pio_external_connection.export
	 version_pio_external_connection_export => Firmware_Version,  -- fixed value!!
	 -- HPS Fifo control
    iofifo_datain                         => iofifo_datain, -- datain
    iofifo_writereq                       => iofifo_writereq,
    iofifo_instatus                       => iofifo_instatus,
    iofifo_dataout                        => iofifo_dataout,
    iofifo_readack                        => iofifo_readack,
    iofifo_outstatus                      => iofifo_outstatus,
    iofifo_regdataout                     => iofifo_regdataout,
    iofifo_regreadack                     => iofifo_regreadack,
    iofifo_regoutstatus                   => iofifo_regoutstatus,
	 -- event_FIFO to DDR3 manager------------------------------------------------------------------
    to_ram_ctrl_fpga_side_reg  => fpga_side_RAM_ctrl_reg,      --     to_ram_ctrl.fpga_side_reg
    to_ram_ctrl_hps_side_reg   => hps_side_RAM_ctrl_reg,       --    .hps_side_reg
    to_ram_ctrl_enable         => SDRAM_interface_enable,   --      .enable
    to_ram_fifo_data_ack       => fifo_readack_from_RAM,         --       to_ram_fifo.data_ack
    to_ram_fifo_data_event     => iofifo_dataout,          --          .data_event
    to_ram_fifo_data_empty     => event_fifo_empty,          --     .data_empty
    hps_fpga_ram_sync_gp_in    => fpga_side_RAM_ctrl_reg,      --       hps_fpga_ram_sync.gp_in
    hps_fpga_ram_sync_gp_out   => hps_side_RAM_ctrl_reg        --     .gp_out
  );

  FPGA0: entity work.FPGATop
  port map (
    --Inputs
    Clock => Clock,
    Reset => Reset,
	 KEY => KEY,
	 adc_raw_values => adc_raw_values,
	 -- Leds
    LED => LED,
	 -- Swithces
    SW  => SW,
	  --///////// GPIO /////////
      oFE0                => sFeA,
      oFE1                => sFeB,
      oADC0               => sAdcA,
      oADC1               => sAdcB,
      iMULTI_ADC          => sMultiAdc,
      --
      iBcoClk             => sBcoClkSynch,
      iBcoRst             => sBcoRstSynch,
      iExtTrig            => sExtTrgSynch,
      oBusy               => sBusy,
      --
      oDEBUG              => sDebug,
      oErrors             => sErrors,
	 -- HPS
    f2h_stm_hw_events  => stm_hwevents,
    button_pio         => KEYE,
    dipsw_pio          => SWD,
    led_pio            => ledreg,
    iofifo_datain      => iofifo_datain,
    iofifo_writereq    => iofifo_writereq,
    iofifo_instatus    => iofifo_instatus,
    iofifo_dataout     => iofifo_dataout,
    iofifo_readack     => iofifo_readack,
    iofifo_outstatus   => iofifo_outstatus,
    iofifo_regdataout     => iofifo_regdataout,
    iofifo_regreadack     => iofifo_regreadack,
    iofifo_regoutstatus   => iofifo_regoutstatus,
	SDRAM_interface_enable => SDRAM_interface_enable,
	event_fifo_empty       => event_fifo_empty,
	fifo_readack_from_RAM  => fifo_readack_from_RAM,
	fpga_side_RAM_ctrl_reg => fpga_side_RAM_ctrl_reg,
	hps_side_RAM_ctrl_reg => hps_side_RAM_ctrl_reg
  );

  -- ADC Controller
  adc_0 : component ADC_controller_IP
    port map (
      CLOCK    => Clock,    --                clk.clk
      RESET    => Reset,    --              reset.reset
		-- Analog values
      CH0      => adc_raw_values(0),      --           readings.CH0
		CH1      => adc_raw_values(1),      --                   .CH1
		CH2      => adc_raw_values(2),      --                   .CH2
		CH3      => adc_raw_values(3),      --                   .CH3
		CH4      => adc_raw_values(4),      --                   .CH4
		CH5      => adc_raw_values(5),      --                   .CH5
		CH6      => adc_raw_values(6),      --                   .CH6
		CH7      => adc_raw_values(7),      --                   .CH7
		-- FPGA pins
		ADC_SCLK => ADC_SCK, -- external_interface.SCLK
		ADC_CS_N => ADC_CONVST, --                   .CS_N
		ADC_DOUT => ADC_SDO, --                   .DOUT
		ADC_DIN  => ADC_SDI   --                   .DIN
  );

  --
  --  Cold, Warm and Debug RESETs
  --
  process(Clock)
    variable oldRes : std_logic;
	 variable counter : natural := 0;
  begin
    if rising_edge(Clock) then
	 -- Generate different resets based on counter (see below)
      if counter>1 and counter<4 then
	     nWarmReset <= '0';-- short
	   else
	     nWarmReset <= '1';
	   end if;
      if counter>1 and counter<8 then
	     nColdReset <= '0';-- longer
	   else
	     nColdReset <= '1';
	   end if;
      if counter>1 and counter<32 then
	     nDebugReset <= '0';-- longest
	   else
	     nDebugReset <= '1';
	   end if;
		-- When rising edge on reset, start counting to 100, then reset and wait
      if Reset='1' and oldRes='0' then
	     counter :=1;
	   elsif counter > 0 and counter<100 then
        counter := counter+1;
	   else
	     counter := 0;
	   end if;
	   oldRes := Reset;
	 end if;
  end process;


end architecture structural;
