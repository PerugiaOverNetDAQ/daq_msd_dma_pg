-- -----------------------------------------------------------------------
-- FPGA Top for DE0DAQ  application
-- -----------------------------------------------------------------------          

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.DAQ_Package.all;	
use work.FOOTpackage.all;		

------------------------------------------------
entity FPGATop is
------------------------------------------------
  port (
	 Clock    : in     std_logic;-- System clock  1 (50 MHz)
    Reset    : in     std_logic;
	 -- Keys
	 KEY : in std_logic_vector(1 downto 0);
	 -- ADC channles values
	 adc_raw_values : in adc_values_t;
	 -- Leds
    LED : out std_logic_vector(7 downto 0);
	-- Switches
    SW  : in  std_logic_vector(3 downto 0);
	--///////// GPIO /////////
    oFE0                : out   tFpga2FeIntf;
    oFE1                : out   tFpga2FeIntf;
    oADC0               : out   tFpga2AdcIntf;
    oADC1               : out   tFpga2AdcIntf;
    iMULTI_ADC          : in    tMultiAdc2FpgaIntf;
    --
    iBcoClk             : in    std_logic;
    iBcoRst             : in    std_logic;
    iExtTrig            : in    std_logic;
    oBusy               : out   std_logic;
    --
    oDEBUG              : out   std_logic_vector(7 downto 0);
    oErrors             : out   std_logic;
	-- To HPS
   f2h_stm_hw_events   : out  std_logic_vector(27 downto 0) := (others => 'X'); -- stm_hwevents
   button_pio          : out   std_logic_vector(3 downto 0)  := (others => 'X'); -- export
   dipsw_pio           : out   std_logic_vector(3 downto 0)  := (others => 'X'); -- export
   led_pio             : in   std_logic_vector(7 downto 0);                     -- export
   iofifo_datain       : in    std_logic_vector(31 downto 0);                    -- datain
   iofifo_writereq     : in    std_logic;                                        -- writereq
   iofifo_instatus     : out   std_logic_vector(31 downto 0) := (others => 'X'); -- instatus
   iofifo_dataout      : out   std_logic_vector(31 downto 0) := (others => 'X'); -- dataout
   iofifo_readack      : in    std_logic;                                        -- readack
   iofifo_outstatus    : out   std_logic_vector(31 downto 0) := (others => 'X'); -- outstatus
   iofifo_regdataout   : out   std_logic_vector(31 downto 0) := (others => 'X'); -- regdataout
   iofifo_regreadack   : in    std_logic;                                        -- regreadack
   iofifo_regoutstatus : out std_logic_vector(31 downto 0) := (others => 'X'); -- regoutstatus 
   SDRAM_interface_enable : out std_logic;
   event_fifo_empty       : out std_logic;
   fifo_readack_from_RAM  : in  std_logic;
   fpga_side_RAM_ctrl_reg : in    std_logic_vector(31 downto 0);                 -- fpga_side_RAM_ctrl_reg
   hps_side_RAM_ctrl_reg  : in    std_logic_vector(31 downto 0)                    --  hps_side_RAM_ctrl_reg
  );
end entity FPGATop;
	  
-----------------------------------------------------------------
architecture structural of FPGATop is
-----------------------------------------------------------------
  signal KEYD, nKEY : std_logic_vector(1 downto 0) := "00";   
  signal SWD : std_logic_vector(3 downto 0) := "0000";   

  signal MainTriggerExt, MainTriggerKey, MainTrigger : std_logic;
  --signal enabletrg, randomtrigger : std_logic;
  signal BCOClock, BCOReset : std_logic;

  signal ledst   : std_logic_vector(7 downto 0);
  
  signal longerReadAck, longerEndOfEvent, longerTrigger : std_logic := '0';

  -- From DAQModule 
  signal Data_Stream : std_logic_vector (31 downto 0);  
  signal RegData_Stream : std_logic_vector (31 downto 0);
  signal readFromDAQ, writeReqToOutFifo : std_logic;
  signal readFromRegister, wrreqToOutFifo_reg : std_logic;
  signal Busy_Out, Trigger_Out  : std_logic := '0';
  signal TX_Empty : std_logic;
  signal RegTX_Empty : std_logic;
  signal RX_AlmostFullFlag : std_logic; 
  signal Errors : std_logic;
  signal Led_State: std_logic_vector(2 downto 0);
  signal Ethernet_Wrreq : std_logic;
  signal Ethernet_DataIn : std_logic_vector(31 downto 0);
  
  -- for fifo IO
  signal fifo_readack : std_logic ;
  signal full, almost_full, empty : std_logic ;
  signal dataOut : std_Logic_vector (31 DOWNTO 0);
  signal usedw	 : std_Logic_vector (13 DOWNTO 0);
  
  -- for register fifo
  signal regfifostatus : std_Logic_vector (31 downto 0);
  signal regfifo_readack : std_logic := '0';
  signal regfull, regalmost_full, regempty : std_logic ;
  signal regdataOut : std_Logic_vector (31 DOWNTO 0);
  signal regusedw	 : std_Logic_vector (8 DOWNTO 0);
  signal debugVector	 : std_Logic_vector (7 DOWNTO 0);

  --signal beam, beamStart, beamEnd : std_logic := '0';
  
  
  signal sTrigInt   : std_logic;

  
begin
  
  --
  --      debounce KEY and SWITCH signals
  --  
  -- debounce KEY      NOT NECESSARY, THERE ARE 74AUC17 ICs
  nKEY <= not(KEY);
  dbc0: entity work.Debounce  
    port map( Clk=>Clock, Reset=>Reset,   button=>nKEY(0),   pulse=>KEYD(0) );
  dbc1: entity work.Debounce  
    port map( Clk=>Clock, Reset=>Reset,   button=>nKEY(1),   pulse=>KEYD(1) );
  
  -- debounce switches
  dbc2: entity work.Debounce  
    port map(Clk=> Clock, Reset=>Reset,   button=>SW(0),     pulse=>SWD(0) );
  dbc3: entity work.Debounce  
    port map(Clk=> Clock, Reset=>Reset,   button=>SW(1),     pulse=>SWD(1) );
  dbc4: entity work.Debounce  
    port map(Clk=> Clock, Reset=>Reset,   button=>SW(2),     pulse=>SWD(2) );
  dbc5: entity work.Debounce  
    port map(Clk=> Clock, Reset=>Reset,   button=>SW(3),     pulse=>SWD(3) );

  -- Maintrigger can be sent via a key (debug) or via an external pin
  MainTrigger <= MainTriggerKey or iExtTrig; -- or randomtrigger;  --!@todo trigger is an OR!! FOR THE MOMENT ONLY!! (From Bologna)

  
  --
  --  DAQ Module
  --  
  EvSim: entity work.DAQ_OnFPGA
    port map(
      Clock => Clock,
	   Reset => Reset,
		Enable => SWD(3), --!@todo Remove it and use a register
	   Reset_Errors => KEYD(1), --!@todo Remove it and use a register
		-- Input from Ethernet --
	   Ethernet_Rdreq => readFromDAQ,
	   Ethernet_RegistersRdreq => readFromRegister,
	   Ethernet_Wrreq => Ethernet_Wrreq,
	   Ethernet_DataIn => Ethernet_DataIn,	  
		-- Signals from GPIO --
	   BCOClock => iBcoClk,
	   BCOReset => iBcoRst,
	   Trigger => MainTrigger,
      -- To State Signals --
	   TowardRun => '0', --SWD(0),
	   TowardIdle => '0', --SWD(1),
	   ReadAll => '0', --SWD(2),
		-- ADC --
		adc_raw_values => adc_raw_values,
		
		-- Outputs --
	   Busy_Out => oBusy,
	   Trigger_Out => Trigger_Out,
		
		Ethernet_DataOut => Data_Stream,
	   Ethernet_RegistersOut => regData_Stream,
		
      regfifostatus     => regfifostatus,    -- This is the FIFO control structure to HPS
	   TX_emptyFlag      => TX_Empty,         -- from local TX, fifo_data from event builder
      RX_AlmostFullFlag => RX_AlmostFullFlag,-- from local RX, almost full of fifo rx
		SDRAM_interface_enable => SDRAM_interface_enable,-- SDRAM module enable
      Errors            => Errors,           -- General flag, Errors Output
      -- beam simulation
     -- beam => beam, 
      --beamStart => beamStart,
      --beamEnd => beamEnd,
	--input signals from MSD
      iMULTI_ADC => iMULTI_ADC,

      --output signals to MSD
      oCAL_TRIG => sTrigInt,

      -- First FE-ADC chain ports
      oFE0  => oFE0,                   --!Output signals to the FE1
      oADC0 => oADC0,                  --!Output signals to the ADC1
      -- Second FE-ADC chain ports
      oFE1  => oFE1,                   --!Output signals to the FE2
      oADC1 => oADC1,                  --!Output signals to the ADC2
		
		
	   -- Led State
	   Led_State => Led_State,
	  	-- Debug
      debugVector => oDEBUG,
	  -- DDR pointers
      fpga_side_RAM_ctrl_reg => fpga_side_RAM_ctrl_reg,
	   hps_side_RAM_ctrl_reg => hps_side_RAM_ctrl_reg
    );
	
  --
  --  Fifo for input (register requests/commands)
  --  
  InCmd: entity work.InputFifoControl
    port map(
      Clk => Clock,
	   Reset => Reset,
 	   -- HPS signal
      iofifo_datain      => iofifo_datain,
      iofifo_writereq    => iofifo_writereq,
      iofifo_instatus    => iofifo_instatus,
      -- DAQ_OnFPGA
	   RX_AlmostFullFlag  => RX_AlmostFullFlag,-- from local rx, almost full of fifo rx
      Ethernet_Wrreq     => Ethernet_Wrreq,-- To DAQ_OnFPGA, to local rx
	   Ethernet_DataIn    => Ethernet_DataIn-- To DAQ_OnFPGA, to local rx
	 );

  --
  -- Manage transmissions between OUTPUT fifos (the two fifos down below)
  --  
  -- When the "fifo_data" of local TX is not empty, data are sent to Event FIFO (another FIFO, in this entity)
  process(Clock)
    variable oldKey0 : std_logic;
  begin
    if ( rising_edge(Clock) ) then
	  WriteReqToOutFifo <= readFromDAQ and not(TX_Empty);-- write request to event fifo (the last one)
      if TX_Empty = '0' and almost_full = '0' then-- when the receiving event fifo is not almost full
	    readFromDAQ <= '1';
	  else
	    readFromDAQ <= '0';
	  end if;
	  if( KEYD(0)='1' and oldkey0='0' )then
	    MainTriggerKey <= '1';
	  else
	    MainTriggerKey <= '0';
	  end if;
	  oldKey0 := KEYD(0);
	end if;
  end process;
  
  -- When the "fifo rx" of local TX is not empty. data are sent to reg fifo (another FIFO, in this entity)
  RegTX_Empty <= regfifostatus(16);-- from the fifo RX of local TX (regsfifo)
  process(Clock)
  begin
    if ( rising_edge(Clock) ) then
	  wrreqToOutFifo_reg <= readFromRegister and not(regTX_Empty);-- write request to registrer fifo (the last one)
     if RegTX_Empty = '0' and regalmost_full = '0' then
	    readFromRegister <= '1';
	  else
	    readFromRegister <= '0';
	  end if;
	end if;
  end process;	 
	 
  --
  --  Fifos for output (events and register requests) -> it's a show-ahead fifo
  --  
  EVF: entity work.EventFifo-- This FIFO can be read through the lw bus or the RAM
  port map (
	 clock		 => Clock,
	 data		    => Data_Stream,
	 wrreq		 => WriteReqToOutFifo,
	 rdreq		 => fifo_readack or fifo_readack_from_RAM,
	 q		       => DataOut,
	 almost_full => almost_full,
	 empty		 => empty,
	 full		    => full,
	 usedw		 => usedw
  );
  -- TO HPS
  iofifo_dataout <= DataOut;
  event_fifo_empty <= empty;
  -- Status
  iofifo_outstatus <=   x"000"& '0' & full & almost_full & empty & '0' & full & usedw;
	 
  RegEVF: entity work.RegEventFifo
  port map (
	 clock		 => Clock,
	 data		    => RegData_Stream,
	 wrreq		 => wrreqToOutFifo_reg,
	 rdreq		 => regfifo_readack,      
	 q		       => RegDataOut,
	 almost_full => regalmost_full,
	 empty		 => regempty,
	 full	       => regfull,
	 usedw	    => regusedw
  );
  -- TO HPS
  iofifo_regdataout <= RegDataOut;
  -- Status
  iofifo_regoutstatus <= x"000"& '0' 
		& regfull & regalmost_full & regempty & 
		"000000" & regfull & regusedw;
  
  --
  --  Fifos for output (events and register requests) reading acknowledge provided by HPS
  --  
  process(Clock, reset)
    variable oldReadack, oldRegReadack: std_logic := '0';
  begin
	if( reset='1') then
	  fifo_readack <= '0';
     regfifo_readack <= '0';
	  oldReadack := '0';
	  oldRegReadack := '0';
   elsif ( rising_edge(Clock) ) then-- It's like a edge detector of the iofifo_readack and iofifo_regreadack
     -- event fifo
	  if( (iofifo_readack='1') and (oldReadack='0') )then
	    fifo_readack <= '1';
	  else
	    fifo_readack <= '0';
	  end if;
	  oldReadack := iofifo_readack;
	  -- reg fifo
	  if( (iofifo_regreadack='1') and (oldRegReadack='0') )then
	    regfifo_readack <= '1';
	  else
	    regfifo_readack <= '0';
	  end if;
	  oldRegReadack := iofifo_regreadack;
	end if;
  end process;  
  
  -- 
  -- under suitable conditions (i.e. SWD(0)=1) generate continuously a trigger
  --
  --enabletrg <= '1' when SWD(0)='1' and Led_State="011" and Busy_Out ='0'
  --             else '0';
  --trgint: entity work.Trigger_generator
  --        port map(Clock=>Clock, Reset=>Reset,  Enable=>enabletrg,   trigger=>randomtrigger );
  
  --
  --    Other outputs to HPS
  --  
  -- This contains keys and most important FIFO bits
  button_pio <= almost_full & empty & KEYD;-- key extended: contains push button and event fifo flags
  -- switch status
  dipsw_pio <= SWD;
  -- stm_hwevents:  how these information is used??
  f2h_stm_hw_events <=  "000" & full & almost_full & empty & '0' & full & usedw & swd & KEYD;  
			
  --
  --    step 6: provide some output signals to LED and GPIO
  --  

  -- make some signals longer so that they become suitable for LED pulses
  lp1: entity work.LongerPulse
       port map( Clk=>Clock, Reset=>Reset,  pulse=>MainTrigger,   longPulse=>longerTrigger );
  lp2: entity work.LongerPulse
       port map( Clk=>Clock, Reset=>Reset,  pulse=>fifo_readack,  longPulse=>longerReadAck );
 	
  -- Out to LEDs
  ledst <= Busy_Out & almost_full & not(empty) & longerTrigger & Led_State & Errors;
  LED <= ledst or led_pio;
	 
 

end architecture structural;
