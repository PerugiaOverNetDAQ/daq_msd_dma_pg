library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

entity Trigger_generator is

  port (
    Clock : in std_logic;
	Reset : in std_logic;
	Enable : in std_logic;
	 
	trigger : out std_logic
  );
  
 end entity;
  
architecture behavior of Trigger_generator is

  signal oldEnable : std_logic := '0';
  signal delayLine : std_logic_vector(31 downto 0) := x"00000000";

begin	
	
  process(Clock,Reset) is
  begin
	if(Reset = '1') then
      oldEnable <= '0';
	  delayLine <= x"00000000";
	elsif rising_edge(Clock) then
	  if delayLine(31 downto 28)="0000" then
	    trigger <= '0';
	  else
	    trigger <= '1';
	  end if;
	  if Enable='1' and oldEnable='0' then
	    delayLine <= delayLine(30 downto 0) & '1';
	  else 
	    delayLine <= delayLine(30 downto 0) & '0';
	  end if;
      oldEnable <= Enable;
    end if;
  end process;	

end behavior;