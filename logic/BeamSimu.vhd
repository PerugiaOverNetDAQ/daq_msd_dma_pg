library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

entity BeamSimu is
  port(
    clk: in std_logic; -- assume it is 50 MHz
    reset: in std_logic;
    beam : out std_logic;
    beamStart : out std_logic;
    beamEnd : out std_logic
  );
end BeamSimu;


architecture behavior of BeamSimu is

  component PeriodicTick is
  generic ( tickPeriod : natural := 50);
  port(
    Clock : in std_logic;
    Reset : in std_logic;
    Tick  : out std_logic
  );
  end component;

  component Random_generator is
  port (
    Clock : in std_logic;
 	  Reset : in std_logic;
	  Enable : in std_logic;
	  Random_Data : out unsigned (15 downto 0)
  );
  end component;

  constant steps : integer := 20;
  constant stepDuration : integer := 50000;  -- in clk clocks (20 ns)
                          -- in milliseconds
  type myarray is array (0 to steps-1) of natural;
  constant stepLength : myarray:= 
    (100, 100, 100, 200, 500,
      500, 500,    500, 500,     500, 500,      500, 500,
      400, 400,    400, 400,     50,50,1300); 
  constant stepIntensity : myarray := 
    (0,0,20,100,300,
      600,2200,   8000,4000,    2500,1800,     1000,1000,
       800,400,    200,30,        0,0,0);
   

  signal microtick, millitick, lbeam : std_logic := '0';
  signal currentStep : natural := 0;  
	signal random : unsigned (15 downto 0);
  signal nbeam : natural := 0;  

begin 

  -- start with milli and micro ticks

  Microtimer: entity work.PeriodicTick
    generic map (tickPeriod => 1013) -- about 20 us; 1013 is a prime number!
    port map (Clock => CLK, Reset => RESET, tick => microtick);

  Millitimer: entity work.PeriodicTick
    generic map (tickPeriod => 50000) -- 1 ms
    port map (Clock => CLK, Reset => RESET, tick => millitick);

  randomg: entity work.Random_generator
    port map (Clock => CLK, Reset => RESET, 
      enable => '1', random_data => random);

  -- define the current step
  
  process(CLK, RESET)  
    variable millicounts : natural := 0; 
  begin					
    if RESET = '1' then
      currentStep <= 0;
      millicounts := 0;
	  elsif rising_edge(CLK) then

      if( millitick = '1') then
        millicounts := millicounts +1;
        if( millicounts >= stepLength(currentStep) ) then
          millicounts := 0;
          if( currentStep =19 ) then
            currentStep <= 0;
          else 
            currentStep <= currentStep+1;
          end if;
        end if;
      end if;
    end if;
  end process;

-- define beam start and beam end (4 clock ticks long each)
  
  process(CLK, RESET)  
    variable OldCurrentStep : natural := 0; 
    variable longBeamStart, longBeamEnd : std_logic_vector(3 downto 0) := (others=>'0'); 
  begin					
    if RESET = '1' then
      beamStart <= '0';
      beamEnd <= '0';
      longBeamStart := (others=>'0');
      longBeamEnd := (others=>'0');
      OldCurrentStep := 0;
	  elsif rising_edge(CLK) then
      if longBeamStart /="0000" then
        beamStart <= '1';
      else
        beamStart <= '0';
      end if;
      if longBeamEnd /="0000" then
        beamEnd <= '1';
      else
        beamEnd <= '0';
      end if;
      
      if currentStep=1 and OldCurrentStep=0 then
        longBeamStart := longBeamStart(2 downto 0) & '1';
      else
        longBeamStart := longBeamStart(2 downto 0) & '0';
      end if;
      if currentStep=19 and OldCurrentStep=18 then
        longBeamEnd := longBeamEnd(2 downto 0) & '1';
      else
        longBeamEnd := longBeamEnd(2 downto 0) & '0';
      end if;      
      OldCurrentStep := currentStep;
    end if;
  end process;

  -- define the output signal
  process(CLK, RESET)  
    variable longBeam : std_logic_vector(3 downto 0) := (others=>'0'); 
  begin					
    if RESET = '1' then
      lbeam <= '0';
      longBeam := (others=>'0');
	  elsif rising_edge(CLK) then
      -- output beam!
      if longBeam /="0000" then
        lbeam <= '1';
      else
        lbeam <= '0';
      end if;      
      -- test at each us if it is needed to generate a beam particle
      if microtick='1' and unsigned(random)<stepIntensity(currentStep) then
        longBeam := longBeam(2 downto 0) & '1';
      else
        longBeam := longBeam(2 downto 0) & '0';
      end if;
    end if;
  end process;
  
  beam <= lbeam;

  -- count beams in each period
  process(CLK, RESET)  
    variable oldstep : natural := 0; 
    variable oldbeam : std_logic := '0'; 
  begin					
    if RESET = '1' then
      nbeam <= 0;
      oldstep := 0;
      oldbeam := '0';
	  elsif rising_edge(CLK) then
      -- output beam!
      if lbeam='1' and oldbeam='0' then
        nbeam <= nbeam+1;
      end if;      
      -- test at each us if it is needed to generate a beam particle
      if currentStep /=oldstep then
        nbeam <= 0;
      end if;
      oldstep := currentStep;
      oldbeam := lbeam;
    end if;
  end process;
  
end architecture;