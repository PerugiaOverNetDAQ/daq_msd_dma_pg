library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;


entity Trigger_Control is
  port (
    -- Inputs 
    Clock : in std_logic;          
    Reset : in std_logic;                
    -- The Finite state machine is in Running State
    DAQIsRunning : in std_logic;  
    -- Resets Counters preparing for run 
    DAQ_Reset : in std_logic;
	 -- Resets errors
    Reset_Errors: in std_logic;	 
    -- External clock 
    BCOClock : in std_logic;      
    BCOReset : in std_logic;    
    -- Trigger signal from GPIO
    Trigger : in std_logic;       
    -- Internl Busy from Event Builder
    Busy : in std_logic;
    
    -- Outputs
    -- ClkCounter counts the number of clocks since DAQIsRunning passed from 0 to 1. The counter has 38bit
    ClkCounter : out unsigned (31 downto 0);
    LSB_ClkCounter : out unsigned (5 downto 0);
    -- BCOCounter Counts the number of BCOClocks      
    BCOCounter : out unsigned (31 downto 0);  
    -- TriggerCounter Counts the number of Main_Trigger since DAQIsRunning passed from 0 to 1    
    triggerCounter : out unsigned (31 downto 0);  
    -- Trigger is sent to the inner part of the machine if DAQ is Running and the event builder is not busy
    Internal_Trigger : out std_logic ;
    --Busy out passes a stretched version of the Event Builder Busy to GPIO 
    Busy_Out : out std_logic; 
    -- Errors
    Error_notRunning : out std_logic; 
    Error_busy : out std_logic;
	debugReg : out std_logic_vector(31 downto 0);
	debugTrg : out std_logic_vector(7 downto 0)
  );
end entity;
   
   -- Architecture of the Trigger Control
  
architecture Structural of Trigger_Control is

  -- Declaration of components
  -- ClockCounter 38bit
  component Counter_nbit
  generic ( nbit : natural := 38);
    port(
      CLK   : in std_logic;
      RESET : in std_logic;
      COUNT : out unsigned (nbit-1 downto 0)
    );
  end Component;
  
  -- Used for BCOClockCounter and TriggerCounter
  component RisingEdge_Counter 
    port(
      clock        : in std_logic;
      reset        : in std_logic;
      eventToCount : in std_logic;   
      -- Output 
      counts       : out unsigned (31 downto 0)
    );
    end Component;
	 
  -- Busy_Stretch
  -- Makes BusyOut = '0' if Busy has been '0' for 16 or more clocks
  component Busy_Stretch 
    generic (nbit: natural := 16); 
    port (
      -- Inputs
      Clock : in std_logic;
      Reset : in std_logic;
      Busy : in std_logic;
      -- Outputs
      Busy_Out : out std_logic
    );
   end component;
    
  -- Internal Signals
  -- External clock
  signal BCOClockR, BCOClock0 : std_logic:= '0';      
  -- Trigger signal from GPIO
  signal TriggerR, Trigger0 : std_logic := '0';       
                        
  signal Internal_Trigger_candidate : std_logic :='0';     
  -- Used if DaqReset acts like Reset  
  signal EveryReset : std_logic;                        
  -- Used if BCOReset acts like Reset
  signal BCOCounterReset : std_logic;                  
  signal past_trigger : std_logic := '0';  
  signal triggerDelayer : std_logic_vector(31 downto 0) := (others=>'0');  
  signal BusyM : std_logic := '0';
  signal triggerError : std_logic := '0';  
  
begin
                                                  
  sync_signals : process (Clock)  
    variable BCOResetLong : std_logic_vector(7 downto 0):= (others=>'0');
  begin
    if rising_edge(Clock) then   
	   -- synching resets
      EveryReset <= Reset or DAQ_Reset;  
		
      if( BCOResetLong = x"FF" )then	  
        BCOCounterReset <= '1';-- On when BCOReset lasts for 8*20= 160 ns
	  else
        BCOCounterReset <= '0';
	  end if;
	  -- registering external signals
	  TriggerR <= Trigger0;
	  BCOClockR <= BCOClock0;
	  Trigger0 <= Trigger;
	  BCOClock0 <= BCOClock;
	  BCOResetLong := BCOResetLong(6 downto 0) &  BCOReset;
	end if;
  end process;
                                                  
  -- Instantiation of components
  -- Counts the number of clocks
  ClockCounter : Counter_nbit 
  port map(
    CLK => Clock,
    RESET => EveryReset,
    COUNT(37 downto 6) => ClkCounter,
    COUNT(5 downto 0) =>  LSB_ClkCounter
  );

  -- Counts the number of main trigger
  -- note: it counts a delayed signal so that it is possible to
  -- store in the event the current trigger number (starting from 0)
  MainTriggerCounter : RisingEdge_Counter
  port map(                                       
    clock => Clock,
    reset => EveryReset,
    eventToCount =>  triggerDelayer(31),
    counts => triggerCounter
  );  
                                                  
                                                
  -- BCO_Counter--
  -- Counts the number of BCOClocks
  BCO_Counter : RisingEdge_Counter
  port map(                                        
    clock => Clock,
    reset => BCOCounterReset,
    eventToCount => BCOClockR,
    counts => BCOCounter
  );
   
  -- prepare a long busy
  BusyM <= '0' when Busy='0' -- and Trigger='0' and triggerDelayer=(others=>'0')  -- check later
               else '1';
 
  -- Busy_Stretcher for external pin
  -- Busy_Out = '0' if Busy has been '0' for 16 or more clocks
  Busy_Stretcher : Busy_Stretch
  port map(                                        
    Clock => Clock,
    Reset => Reset,
    Busy => BusyM,
    Busy_Out => Busy_Out
  );
   
  Trigger_Verify : process (Clock, EveryReset)  
  begin
    if EveryReset = '1' then      
      Error_Busy <='0';
      Error_notRunning <='0';
      Internal_Trigger_candidate<='0';   
      past_trigger<='0';
	   triggerDelayer<= (others=>'0');
    elsif rising_edge(Clock) then   
      if TriggerR = '1' and past_trigger = '0' then-- at rising edge of triggerR
        if Busy = '0' and DAQIsRunning = '1' then-- check status of FSM and Event_Builder
          Internal_Trigger_candidate <= '1';-- GENERATE TRIGGER
        else
          Internal_Trigger_candidate <= '0';-- DON'T
          if Busy = '1' then-- Why haven't you generated a trigger?
            Error_Busy <= '1';-- because Event_Builder has been busy
          end if;
          if DAQIsRunning = '0' then
            Error_notRunning <= '1';-- because the system was not running
          end if;
        end if;
      else
        Internal_Trigger_candidate <= '0';
      end if;
      past_trigger <= TriggerR;
	   triggerDelayer <=  triggerDelayer(30 downto 0) & Internal_Trigger_candidate; 
      if Reset_Errors = '1' then-- Reset error flags
        Error_Busy <='0';
        Error_notRunning <='0';	 
      end if;
    end if;
  end process;
  
  Internal_Trigger <= Internal_Trigger_candidate;
  
  debugTrg <= TriggerError & Internal_Trigger_candidate & Busy & past_trigger & triggerR & DAQIsRunning & Clock & EveryReset; 
  
  -- Debug trigger conditions
  Trigger_Check : process (Clock, EveryReset)
    variable triggerRet,pastTriggerRet, BusyRet,DAQIsRunningRet,TrigcandRet : std_logic_vector(7 downto 0) := (others=>'0');  
	 variable counter : integer :=0;
  begin
    if EveryReset = '1' then
	   TriggerError <= '0';
	   pastTriggerRet := (others=>'0');
	   triggerRet := (others=>'0');
	   BusyRet := (others=>'0');
	   DAQIsRunningRet := (others=>'0');
	   TrigcandRet := (others=>'0');
	   counter :=0;
	 elsif rising_edge(Clock) then
	   -- If a trigger arrived but it wasn't catched, then something is wrong
	   if( triggerRet(7 downto 6)="01" and Busy='0' ) then
	     TriggerError <='1';
		  debugreg <= BusyRet(7 downto 0 ) & triggerRet(7 downto 2 ) & pastTriggerRet(7 downto 2 ) &  DAQIsRunningRet(7 downto 2 ) & TrigcandRet(7 downto 2 );
	   end if;
	  
      triggerRet := triggerRet(6 downto 0 ) & TriggerR;
      pastTriggerRet :=  pastTriggerRet(6 downto 0 ) & past_trigger;
	   BusyRet := BusyRet(6 downto 0 ) & Busy;
	   DAQIsRunningRet := DAQIsRunningRet(6 downto 0 ) & DAQIsRunning;
	   TrigcandRet := TrigcandRet(6 downto 0 ) & Internal_Trigger_candidate;
		
	   if( counter<100000000 ) then
	     counter := counter+1;
		  TriggerError <='0';
	   else
	     counter := 0;
	   end if;
    end if;
  end process;
  
end architecture;
     

     