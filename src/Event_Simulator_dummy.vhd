library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;
use work.DAQ_Package.all;

entity Event_Simulator_dummy is -- QUESTA ENTITY È DA ELIMINARE QUANDO ARRIVERÀ BLOCCO DA PERUGIA
  port (
    -- Inputs
    Clock : in std_logic;
	 Reset : in std_logic;
	 -- Trigger
	 Trigger : in std_logic;
    -- Frame counter
    FrameCount : in unsigned (31 downto 0);
    -- Outputs
	 Data_Stream : out unsigned (31 downto 0);  
	 Data_Valid : out std_logic;
	 -- Is '1' if that is the last word of the sequence
    EndOfEvent : out std_logic	 
  );
end entity;

architecture algorithm of Event_Simulator_dummy is
  
  -- A Random_Generator of 16bit width is used to create the randomic data
  component Random_generator 
    port (
	   -- Inputs
      Clock : in std_logic;
	   Reset : in std_logic;
	   Enable : in std_logic;
	   -- Outputs
	   Random_Data : out unsigned (15 downto 0)
    );
  end component;
  
  -- Internal Counter declaration 
  component Counter_nbit 
    generic ( nbit : natural := 8);  
    port(
      CLK: in std_logic;
      RESET: in std_logic;
      COUNT : out unsigned (nbit-1 downto 0)
  );
  end component;
  
  -- Possible states of the FSM
  -- Idle: machine is waiting for trigger to go from 0 to 1 to start
  -- LengthDeclaration : 32bit word containing the length of the total stream is the output
  -- Header: Data_Stream sends the header's words sequentially 
  -- Sequence : output is now a random sequence of 32bit words
  -- Footer : the sequence ends with a constant ending sequence (footer)
  type state is (idle,lengthDeclaration,header_1,header_2,header_3,header_4,sequence,footer_1,footer_2,footer_3);
  
  -- Internal signals
  
  signal present_state, next_state : state := idle;
  -- Counters
  signal internalCounter : unsigned (7 downto 0);
  -- Register that holds the length of the sequence
  signal sequenceLength : unsigned (7 downto 0) := (others=>'0'); 
  -- Counter Reset
  signal internalReset : std_logic;
  signal counterReset : std_logic;
  -- randomBuses carry the random numbers created by the random generator
  signal randomBus1 : unsigned (15 downto 0) := x"0000";
  
begin

  -- Random Generators Instantiation
  randomGen1 : Random_generator
  port map (
    Clock => Clock,
    Reset => Reset,
    Enable => '1',  
	 Random_Data => randomBus1
  );
  
  -- Used to count words in sequence state (it is also used as part of an output word for debug purposes)
  counterMachine : Counter_nbit 
  generic map( nbit => 8) 
  port map (
	 CLK => Clock,
	 RESET => counterReset,
	 COUNT => internalCounter
  );  
  
  internalReset <= '1' when next_state = idle-- Reset counter when in idle
                       else '0';
  counterReset <= internalReset or Reset;

  -- Decides the next state of the machine
  process(Trigger, present_state, internalCounter, SequenceLength)
  begin
    case present_state is 
	 
      when idle => 
        if Trigger = '1' then     
          next_state <= lengthDeclaration;
		  else
			 next_state <= present_state; 
        end if;
		  
		when lengthDeclaration =>                                      
        next_state <= header_1;
		
		when header_1 =>                                      
        next_state <= header_2;
		
		when header_2 =>                                      
        next_state <= header_3;
		
		when header_3 =>                                      
        next_state <= header_4;
		
		when header_4 =>
        next_state <= sequence;
		  
	   when sequence =>
        if internalCounter >= sequenceLength-conv_unsigned(3,8) then-- lenght of the message minus the 3 footers
          next_state <= footer_1;                                   -- that haven't arrived yet
        else
          next_state<=present_state;			 
        end if;
		
		when footer_1 =>
        next_state <= footer_2;
		
		when footer_2 =>                                      
        next_state <= footer_3;
		
		when footer_3 =>                                      
        next_state <= idle;
		  
		when others =>
		  next_state<=idle; 
		  
    end case;
  end process;
	 
  -- It generates the Data_Stream output value based on actual internal state and counters
 process( present_state, internalCounter, randomBus1, sequenceLength, FrameCount)
 begin
   case present_state is
	 
     when idle  => 	
	    Data_Stream <= conv_unsigned(0,32);
	    
	  when lengthDeclaration => 
		 Data_Stream <= conv_unsigned(0,24) & sequenceLength;
	   
	  when header_1 =>
		 Data_Stream <= Header1_ES_DUMMY; -- UNICO CAMBIAMENTO RISPETTO A EVENT SIMULATOR VERO--------------------<
		
	  when header_2 =>
		 Data_Stream <= Header2_ES; 
		
	  when header_3 =>
		 Data_Stream <= Header3_ES;
		
	  when header_4 =>
		 Data_Stream <= FrameCount;-- NB there is Header4_ES in the package

	  when sequence =>
		 Data_Stream <= internalCounter & constantPart(15 downto 8) & randomBus1;
	  
	  when footer_1 =>
	    Data_Stream <= Footer1_ES; 
		
	  when footer_2 =>
	    Data_Stream <= Footer2_ES; 
		
	  when footer_3 =>
	    Data_Stream <= Footer3_ES; 

	  when others => Data_Stream<=conv_unsigned(0,32);
	
	end case;
  end process;
  
  -- Data_Valid OutPut
  Data_Valid <= '0' when present_state = idle
	                else '1';
  
  -- End_Of_Sequence
  EndOfEvent <= '1' when present_state = footer_3
                    else '0';
 
  -- Sequential part 
  process(Clock, Reset)
  begin
    if Reset = '1' then
	   present_state<=idle;
		sequenceLength <= (others=>'0');
    elsif rising_edge(Clock) then
      present_state <= next_state;
		if present_state = idle then-- sequenceLength chosen in idle
		  if randomBus1(7 downto 0) >= conv_unsigned (13,8) then
		    sequenceLength <= randomBus1(7 downto 0);
		  else
			 sequenceLength(6 downto 0) <= randomBus1(6 downto 0);
          sequenceLength(7) <= (not randomBus1(7));-- just to increase variablity
		  end if;
		end if;
    end if;
  end process;
  
end architecture;