--  UART_TB -- Testbench for the UART pheripheral compatible with the BenchRV32 RISC-V Core
--  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
--  
--  A file containing the testbench for the developped UART
--
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
--  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
--  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
--  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
--  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
--  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
--  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
 
entity TB_UART is
end TB_UART;
 
architecture behavioral of TB_UART is
  
	component UART_RX is generic (
		gCLKS_x_BIT : integer := 115
		);	
	port (
		oRX_Byte   : out std_logic_vector(7 downto 0);    
		oRX_GotIt  : out std_logic;
		iRX_Serial : in  std_logic;
		iCLK       : in  std_logic      
		);
	end component UART_RX;

	component UART_TX is generic (
		gCLKS_x_BIT : integer := 115   -- Needs to be set correctly
		);
	port (
		oTX_Done   : out std_logic;
		oTX_Serial : out std_logic;		    
		iTX_Byte   : in  std_logic_vector(7 downto 0);
		iTX_Enable : in  std_logic;
		iCLK       : in  std_logic
		);
	end component UART_TX;  

	-- Test Bench uses a 100 MHz Clock while interface is set to 2000000 baud UART
	-- 10000000 / 2000000 = 5 Clocks Per Bit.
	constant tb_CLKSxBIT : integer := 5;
	constant c_BIT_PERIOD : time := 500 ns;

	signal rCLK		   : std_logic := '0';
	signal rTX_Enable : std_logic := '0';
	signal rTX_BYTE   : std_logic_vector(7 downto 0) := (others => '0');
	signal wTX_SERIAL : std_logic;
	signal wTX_DONE   : std_logic;
	signal wRX_GotIt  : std_logic;
	signal wRX_BYTE   : std_logic_vector(7 downto 0);
	signal rRX_SERIAL : std_logic := '1';

	procedure UART_WRITE_BYTE (
		i_data_in       : in  std_logic_vector(7 downto 0);
		signal o_serial : out std_logic) is begin
			-- Send Start Bit
			o_serial <= '0';
			wait for c_BIT_PERIOD;

			-- Send Data Byte
			for ii in 0 to 7 loop
			o_serial <= i_data_in(ii);
			wait for c_BIT_PERIOD;
			end loop;  -- ii

			-- Send Stop Bit
			o_serial <= '1';
			wait for c_BIT_PERIOD;
		end UART_WRITE_BYTE;


	begin

		-- Instantiating UART Receiver
		UART_RX_INST : UART_RX generic map (
			gCLKS_x_BIT => tb_CLKSxBIT
		) 
		port map (
			oRX_Byte   => wRX_BYTE,
			oRX_GotIt  => wRX_GotIt,      
			iRX_Serial => rRX_SERIAL,      
			iCLK       => rCLK      
		);  

		-- Instantiating UART Transmitter
		UART_TX_INST : UART_TX generic map (
			gCLKS_x_BIT => tb_CLKSxBIT
			)
		port map (
			oTX_Done   => wTX_DONE,    
			oTX_Serial => wTX_SERIAL,
			iTX_Byte   => rTX_BYTE,      
			iTX_Enable => rTX_Enable,
			iCLK       => rCLK      
		);      

		rCLK <= not rCLK after 50 ns;

		process is
			begin
			-- Tell the UART to send a command.
			wait until rising_edge(rCLK);
			wait until rising_edge(rCLK);
				rTX_Enable   <= '1';
				rTX_BYTE 	 <= X"AB";
			wait until rising_edge(rCLK);
				rTX_Enable   <= '0';
			wait until wTX_DONE = '1';

			-- Send a command to the UART
			wait until rising_edge(rCLK);
				UART_WRITE_BYTE(X"3F", rRX_SERIAL);
			wait until rising_edge(rCLK);

			-- Check that the correct command was received
			if wRX_BYTE = X"3F" then
				report "Test Passed - Correct Byte Received" severity note;
			else
				report "Test Failed - Incorrect Byte Received" severity failure;
			end if;

			assert false report "Tests Complete" severity failure;

			end process;

end behavioral;
