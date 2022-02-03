library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;
use work.DAQ_Package.all;

use work.basic_package.all;
use work.FOOTpackage.all;


entity DAQ_OnFPGA is
  port(
    Clock                   : in std_logic;
	 Enable                  : in std_logic;
	 Reset                   : in std_logic;
	 Reset_Errors            : in std_logic;
	 -- Input from Ethernet --
	 Ethernet_Rdreq          : in std_logic;
	 Ethernet_RegistersRdreq : in std_logic;
	 Ethernet_Wrreq          : in std_logic;
	 Ethernet_DataIn         : in std_logic_vector(31 downto 0);
	 -- Signals from GPIO --
	 BCOClock                : in std_logic;
	 BCOReset                : in std_logic;
	 Trigger                 : in std_logic;

	 -- To State Signals --
	 TowardRun               : in std_logic;
	 TowardIdle              : in std_logic;
	 ReadAll                 : in std_logic;

	 -- ADC --
	 adc_raw_values          : in adc_values_t;

	 -- Outputs --
	 Busy_Out                : out std_logic;
	 Trigger_Out             : out std_logic;

	 Ethernet_DataOut        : out std_logic_vector(31 downto 0);
	 Ethernet_RegistersOut   : out std_logic_vector(31 downto 0);

	 regfifostatus           : out std_logic_vector (31 downto 0);-- This is the FIFO control structure to HPS
	 TX_emptyFlag            : out std_logic;-- from local TX, fifo_data from event builder
	 RX_almostFullFlag       : out std_logic;-- from local rx, almost full of fifo rx
	 SDRAM_interface_enable  : out std_logic;
	 Errors                  : out std_logic;-- General flag, Errors Output

	  --Inputs from msd
    iMULTI_ADC : in tMultiAdc2FpgaIntf;  --!Input signals from the ADC1

	  --outputs signals to msd
    oCAL_TRIG : out std_logic;
    oFE0      : out tFpga2FeIntf;       --!Output signals to the FE1
    oADC0     : out tFpga2AdcIntf;      --!Output signals to the ADC1
    oFE1      : out tFpga2FeIntf;       --!Output signals to the FE2
    oADC1     : out tFpga2AdcIntf;      --!Output signals to the ADC2

   -- beam simulation
   --beam : out std_logic;
   --beamStart : out std_logic;
   --beamEnd : out std_logic;

   -- Led State --
   Led_State                : out std_logic_vector ( 2 downto 0);
   -- Debug --
   debugVector              : out std_logic_vector(7 downto 0);

   -- DDR3 pointers
   fpga_side_RAM_ctrl_reg   : in    std_logic_vector(31 downto 0);                 -- fpga_side_RAM_ctrl_reg
   hps_side_RAM_ctrl_reg    : in    std_logic_vector(31 downto 0)                  --  hps_side_RAM_ctrl_reg
  );
end entity;

architecture structural of DAQ_OnFPGA is

  component DAQ_Module
    port(
      -- Inputs --
      Clock: in std_logic;
	   Reset : in std_logic;
	   Reset_Errors : in std_logic;
	   -- Input from Data Generator --
	   Data_Valid : in std_logic;
	   EndOfEvent : in std_logic;
	   In_Data : in unsigned (31 downto 0);
	   -- Input from Ethernet --
	   Ethernet_Rdreq : in std_logic;
	   Ethernet_RegistersRdreq : in std_logic;
	   Ethernet_Wrreq : in std_logic;
	   Ethernet_DataIn : in std_logic_vector(31 downto 0);
	   -- Signals from GPIO --
	   BCOClock : in std_logic;
	   BCOReset : in std_logic;
	   Trigger : in std_logic;
		-- ADC --
		adc_raw_values : in adc_values_t;

	   -- Outputs --
     Busy_In : in std_logic;
	   Busy_Out : out std_logic;
	   Status : out std_logic_vector ( 2 downto 0);
	   Trigger_Out : out std_logic;
	   Ethernet_DataOut : out std_logic_vector(31 downto 0);
	   Ethernet_RegistersOut : out std_logic_vector(31 downto 0);
	   regfifostatus : out std_logic_vector (31 downto 0);
	   TX_emptyFlag : out std_logic;
	   RX_almostFullFlag : out std_logic;
		SDRAM_interface_enable : out std_logic;
		simulated_data_enable : out std_logic;
	   Errors : out std_logic;
  	   -- Debug --
      debugVector : out std_logic_vector(7 downto 0);
      fpga_side_RAM_ctrl_reg : in    std_logic_vector(31 downto 0);                 -- fpga_side_RAM_ctrl_reg
      hps_side_RAM_ctrl_reg  : in    std_logic_vector(31 downto 0);                  --  hps_side_RAM_ctrl_reg
     oRstFromFsm: out std_logic;
     oRunningFromFsm: out std_logic;
    -- Register to the MSD interface
    Register_msd_config : out msd_config
    );
  end component;

  --component Event_Simulator
    --port (
      -- Inputs
     -- Clock : in std_logic;
	   -- Reset : in std_logic;
	   -- Trigger
	   -- Trigger : in std_logic;
      -- Frame counter
      --FrameCount : in unsigned (31 downto 0);
      -- Outputs
	   --Data_Stream : out unsigned (31 downto 0);
	   --Data_Valid : out std_logic;
	   -- Is '1' if that is the last word of the sequence
      --EndOfEvent : out std_logic
   -- );
  --end component;

 -- component Event_Simulator_dummy
   -- port (
      -- Inputs
     -- Clock : in std_logic;
	  --  Reset : in std_logic;
	   -- Trigger
	   -- Trigger : in std_logic;
      -- Frame counter
      --FrameCount : in unsigned (31 downto 0);
      -- Outputs
	  -- Data_Stream : out unsigned (31 downto 0);
	  -- Data_Valid : out std_logic;
	   -- Is '1' if that is the last word of the sequence
     -- EndOfEvent : out std_logic
   -- );
  --end component;

  --component BeamSimu is
  --port(
   -- clk: in std_logic; -- assume it is 50 MHz
    --reset: in std_logic;
   -- beam : out std_logic;
    --beamStart : out std_logic;
   -- beamEnd : out std_logic
 -- );
  --end component;



 -- State of the DAQ Module --
  signal DAQ_State : std_logic_vector ( 2 downto 0);
  signal sRstFromFsm      : std_logic;
  signal sRunningFromFsm  : std_logic;
  -- Internal Signals --
  signal internalData_valid : std_logic:='0';
  signal internalEndOfEvent : std_logic;
  signal internalData       : unsigned (31 downto 0);

  signal dataFromSimulator : unsigned (31 downto 0);
  signal Data_ValidFromSimulator : std_logic:='0';
  signal EndOfEventFromSimulator : std_logic;

  signal data : unsigned (31 downto 0);
  signal Data_Valid : std_logic:='0';
  signal EndOfEvent : std_logic;

  signal sTrigInt         : std_logic;

  signal sReg_msd_config  : msd_config;
  signal soFE0            : tFpga2FeIntf;
  signal soFE1            : tFpga2FeIntf;
  signal soADC0           : tFpga2AdcIntf;
  signal soADC1           : tFpga2AdcIntf;
  signal siMULTI_ADC      : tMultiAdc2FpgaIntf;

  signal data_from_msd      : unsigned (31 downto 0);
  signal data_from_msd_temp : tCollFifoOut;
  signal Busy_Out_DM        : std_logic;
  signal Errors_DM          : std_logic;

  signal sCntOut : tControlIntfOut;

  signal simulated_data_enable : std_logic;

  signal internalTrigger : std_logic;
  signal addressToRead : natural := 0;
   -- Virtual Ethernet Controls --
  signal Virtual_Ethernet_Wrreq :  std_logic:='0';
  signal Virtual_Ethernet_DataIn :  std_logic_vector(31 downto 0);
  -- Past Values --
  signal pastReadAll:  std_logic:='0';
  signal pastStateSignal:  std_logic:='0';
  signal StateSignal :std_logic:='0';
  -- States --
  type state_toState is (idle, writeLength, writeHeader, WriteState,
    lengthToRead, readHeader, writeAddresses, writeFooter);
  signal present_state, next_state : state_toState;

  constant nullVector : std_logic_vector(29 downto 0):=conv_std_logic_vector(0,30);

  signal FrameCount : unsigned(31 downto 0);

begin

 --- Combinatorial assignments MSD------------------------------------------------
  oFE1        <= soFE1;
  oFE0        <= soFE0;
  oADC0       <= soADC0;
  oADC1       <= soADC1;
  oCAL_TRIG   <= sTrigInt;
  Led_State   <= DAQ_State;
  Trigger_Out <= internalTrigger;

  siMULTI_ADC <= iMULTI_ADC;

  Errors      <= Errors_DM or sCntOut.error;
  Busy_Out    <= Busy_Out_DM;


  data_from_msd <= unsigned (data_from_msd_temp.q);
-----------------------------------------------------------------------------

  Led_State <= DAQ_State;

  DAQ_System : DAQ_Module
  port map(
    Clock => Clock,
	 Reset => Reset,
	 Reset_Errors => Reset_Errors,
	 Data_Valid => internalData_Valid,
	 EndOfEvent => internalEndOfEvent,
	 In_Data => data_from_msd,--modified
	 Ethernet_RegistersRdreq => Ethernet_RegistersRdreq,
	 Ethernet_Rdreq => Ethernet_Rdreq,
	 Ethernet_Wrreq => Virtual_Ethernet_Wrreq,
	 Ethernet_DataIn => Virtual_Ethernet_DataIn,
	 BCOClock => BCOClock,
	 BCOReset => BCOReset,
	 Trigger => Trigger,
	 adc_raw_values => adc_raw_values,
   Busy_In => sCntOut.busy,--modified
	 Busy_Out => Busy_Out_DM,--modified
	 Trigger_Out => internalTrigger,
	 Status => DAQ_State,
	 Ethernet_DataOut => Ethernet_DataOut,
	 Ethernet_RegistersOut => Ethernet_RegistersOut,
	 regfifostatus => regfifostatus,
	 TX_emptyFlag => TX_EmptyFlag,
	 RX_almostFullFlag => RX_almostFullFlag,
	 SDRAM_interface_enable => SDRAM_interface_enable,
	 simulated_data_enable => simulated_data_enable,
	 Errors => Errors_DM,
    debugVector => debugVector,
	 -- DDR pointers
    fpga_side_RAM_ctrl_reg => fpga_side_RAM_ctrl_reg,
	 hps_side_RAM_ctrl_reg => hps_side_RAM_ctrl_reg,
   oRstFromFsm => sRstFromFsm,
   oRunningFromFsm => sRunningFromFsm,
   -- Register to the MSD interface
   Register_msd_config => sReg_msd_config
    );

 -- ES : Event_Simulator
  -- port map(
     -- Clock => Clock,
	  -- Reset => Reset,
	  -- Trigger => internalTrigger,
     -- FrameCount => FrameCount,

	  -- Data_Stream => dataFromSimulator,
	  -- Data_Valid => Data_ValidFromSimulator,
     -- EndOfEvent => EndOfEventFromSimulator
  --  );

  --ES_dummy : Event_Simulator_dummy-- This will be replaced by the real sensor interface from Perugia
   --port map(
     -- Clock => Clock,
	  -- Reset => Reset,
	  -- Trigger => internalTrigger,
     -- FrameCount => FrameCount,

	  -- Data_Stream => data,
	   --Data_Valid => Data_Valid,
      --EndOfEvent => EndOfEvent
   -- );

  -- MUX to select simulator or real data from sensors (the two above)
  --process(simulated_data_enable, Data_ValidFromSimulator, EndOfEventFromSimulator, dataFromSimulator, Data_Valid, EndOfEvent, data)
  --begin
   -- if (simulated_data_enable='1') then
	   -- internalData_Valid <= Data_ValidFromSimulator;
		-- internalEndOfEvent <= EndOfEventFromSimulator;
		 --internalData       <= dataFromSimulator;
	 --else
	    --internalData_Valid <= Data_Valid;
		 --internalEndOfEvent <= EndOfEvent;
		-- internalData       <= data;
	-- end if;
  --end process;


  --!@todo Connect the sCntOut flags
  --!@brief Top module that instantiates multiAdcPlaneInterface and Data_Builder
  DB : Data_Builder_Top
    port map(
      iCLK         => Clock,            --!Main clock
      iRST         => Reset or sRstFromFsm or (not sRunningFromFsm), --!Main reset
      -- control interface
      iEN          => Enable and sRunningFromFsm, --!Enable
      iTRIG        => internalTrigger,  --!External trigger
      oCNT         => sCntOut,          --!Control signals in output
      oCAL_TRIG    => sTrigInt,         --!Internal trigger output
      iMSD_CONFIG  => sReg_msd_config,  --!Configuration from the control registers
      -- First FE-ADC chain ports
      oFE0         => soFE0,            --!Output signals to the FE1
      oADC0        => soADC0,           --!Output signals to the ADC1
      iMULTI_ADC   => siMULTI_ADC,      --!Input signals from the ADC1
      -- Second FE-ADC chain ports
      oFE1         => soFE1,            --!Output signals to the FE2
      oADC1        => soADC1,           --!Output signals to the ADC2
      -- to event builder signals
      oCOLL_FIFO    => data_from_msd_temp,
      oDATA_VALID   => internalData_Valid,
      oEND_OF_EVENT => internalEndOfEvent
      );





  -- The following FSM works like an adapter: from the signals from the HPS to the DAQ_Module
  StateSignal <= TowardRun or TowardIdle;
  process(present_state, StateSignal, pastStateSignal, pastReadAll, ReadAll, addressToRead)
  begin
    case present_state is

	   when idle =>
		  if StateSignal = '1' and pastStateSignal = '0' then
	       next_state <= writeLength;
		  elsif ReadAll = '1' and pastReadAll = '0' then
		    next_state <= lengthToRead;
		  else
		    next_state <= idle;
		  end if;

	   when writeLength =>
	     next_state <=writeHeader;

	   when writeHeader =>
	     next_state <= writeState;

		when writeState =>
	     next_state <= writeFooter;

		when lengthToRead =>
		  next_state <= readHeader;

		when readHeader =>
		  next_state <= writeAddresses;

		when writeAddresses=>
		  if addressToRead >= N_MONITOR_REGS+N_CONTROL_REGS-1 then
		    next_state <= writeFooter;
		  else
		    next_state <= writeAddresses;
		  end if;

		when writeFooter =>
		  next_state <= idle;

	   when others =>
	     next_state <= idle;

	 end case;
  end process;


  process(present_state, Ethernet_Wrreq, Ethernet_DataIn, TowardRun, TowardIdle, DAQ_State,addressToRead)
  begin
    case present_state is

		when idle =>-- Direct connection
        Virtual_Ethernet_Wrreq <= Ethernet_Wrreq;
        Virtual_Ethernet_DataIn <= Ethernet_DataIn;

		when writeLength =>-- HPS wants to change state
        Virtual_Ethernet_Wrreq <= '1';
        Virtual_Ethernet_DataIn <= conv_std_logic_vector(4,32);-- Lenght of change state command is 4 words

		when writeHeader =>
        Virtual_Ethernet_Wrreq <= '1';
        Virtual_Ethernet_DataIn <= Header_ChangeState;

		when writeState =>
        Virtual_Ethernet_Wrreq <= '1';
		  if TowardRun = '1' then
		    if DAQ_State = "000" then -- It means IDLE
            Virtual_Ethernet_DataIn <= nullVector & IdleToConfig;
			 else
			   Virtual_Ethernet_DataIn <= nullVector & ConfigToRun;
		    end if;
		  else -- Must be TowardIdle = '1' --
		    if DAQ_State = "011" then -- It means RUN
			   Virtual_Ethernet_DataIn <= nullVector &  RunToConfig;
			 else
			   Virtual_Ethernet_DataIn <= nullVector &  ConfigToIdle;
          end if;
        end if;

		when lengthToRead =>
		  Virtual_Ethernet_Wrreq <= '1';
        Virtual_Ethernet_DataIn <= conv_std_logic_vector(N_MONITOR_REGS+N_CONTROL_REGS+2,32);-- HPS wnats to read every register (2 words of overhead)

	   when readHeader =>
		  Virtual_Ethernet_Wrreq <= '1';
        Virtual_Ethernet_DataIn <= Header_ReadRegs;

		when writeAddresses =>
		  Virtual_Ethernet_Wrreq <= '1';
        Virtual_Ethernet_DataIn <= std_logic_vector(conv_unsigned(addressToRead,32));-- Reading...

		when writeFooter =>
		  Virtual_Ethernet_Wrreq <= '1';
        Virtual_Ethernet_DataIn <= Instruction_Footer;

		when others =>
		  Virtual_Ethernet_Wrreq <= Ethernet_Wrreq;
        Virtual_Ethernet_DataIn <= Ethernet_DataIn;

    end case;
  end process;

  process(Clock, Reset)
  begin
    if Reset = '1' then
      pastStateSignal <= '0';
		pastReadAll <= '0';
		addressToRead <= 0;
    elsif rising_edge(Clock) then
	   if present_state = writeAddresses then
		  addressToRead <= addressToRead+1;
		else
		  addressToRead <= 0;
		end if;
      present_state <= next_state;
		pastStateSignal <= StateSignal;
		pastReadAll <= ReadAll;
    end if;
  end process;

  --BS:  BeamSimu
  --port map(
  --  Clk => Clock,
	-- Reset => Reset,
  --  beam => beam,
   -- beamStart => beamStart,
   -- beamEnd => beamEnd
  --);

 -- FC: work.FrameCounter
  --generic map ( ticks => 9000) -- ticks to count before incrementing
  --port map (
   -- Clk => Clock,
	--  Reset => Reset,
   -- FCOUNT => FrameCount
 -- );


end architecture;
