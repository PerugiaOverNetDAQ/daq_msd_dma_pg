library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

entity FrameCounter is
  generic ( ticks : natural := 9000); -- ticks to count before incrementing
  port(
    CLK: in std_logic; -- assune it is 50 MHz
    RESET: in std_logic;
    FCOUNT : out unsigned (31 downto 0)
  );
end FrameCounter;


architecture behavior of FrameCounter is

  signal frameCount_temp : unsigned (31 downto 0) := ( others =>'0');

begin 

  process(CLK, RESET)  
    variable timer : unsigned (31 downto 0) := ( others =>'0');
  begin					
    if RESET = '1' then
      frameCount_temp <= ( others =>'0');
      timer := ( others =>'0');
	  elsif rising_edge(CLK) then
      timer := timer + conv_unsigned(1,32);
      if( timer >= ticks ) then
        timer := ( others =>'0');        
        frameCount_temp <= frameCount_temp + conv_unsigned(1,32); 
      end if;
    end if;
  end process;

  FCOUNT <= frameCount_temp;

end architecture;