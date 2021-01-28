library IEEE;
use IEEE.std_logic_1164.all;

entity PeriodicTick is
  generic ( tickPeriod : natural := 50);
  port(
    Clock : in std_logic;
    Reset : in std_logic;
    Tick  : out std_logic
  );
end PeriodicTick;


architecture behavior of PeriodicTick is

begin 

  process(Clock, Reset)  
    variable timer : natural := 0; 
  begin					
    if Reset = '1' then
      Tick <= '0';
      timer := 0;
	  elsif rising_edge(Clock) then
      Tick <= '0';
      timer := timer +1;
      if( timer >= tickPeriod ) then
        tick <= '1';
        timer := 0;
      end if;
    end if;
  end process;

end architecture;



