--  UART -- A simple UART pheripheral compatible with the BenchRV32 RISC-V Core
--  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
--  
--  A file containing the TOP Architecture
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
 
entity TOP_UART is generic (
		tCLKS_x_BIT : integer := 115
		);	
	port (
		oRX_Byte   : out std_logic_vector(7 downto 0);
		oRX_GotIt  : out std_logic;
		oTX_Done   : out std_logic;    
		oTX_Serial : out std_logic;
		iTX_Byte   : in  std_logic_vector(7 downto 0);
		iTX_Enable : in  std_logic;
		iRX_Serial : in  std_logic;
		iCLK       : in  std_logic
		);
end TOP_UART;
 
architecture structural of TOP_UART is
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
  
	signal wRX_BYTE   : std_logic_vector(7 downto 0);
	signal wRX_GotIt  : std_logic;
	signal rRX_SERIAL : std_logic := '1';
	signal rTX_BYTE   : std_logic_vector(7 downto 0) := (others => '0');
	signal rTX_Enable : std_logic := '0';
	signal wTX_SERIAL : std_logic;
	signal wTX_DONE   : std_logic;
  
begin 

	-- Instantiating UART Receiver
	UART_RX_INST : UART_RX generic map (
		gCLKS_x_BIT => tCLKS_x_BIT
		) 
	port map (
		oRX_Byte   => wRX_BYTE,
		oRX_GotIt     => wRX_GotIt,      
		iRX_Serial => iRX_Serial,      
		iCLK       => iCLK      
		);  

	-- Instantiating UART Transmitter
	UART_TX_INST : UART_TX generic map (
		gCLKS_x_BIT => tCLKS_x_BIT
		)
	port map (
		oTX_Done   => wTX_DONE,    
		oTX_Serial => oTX_Serial,
		iTX_Byte   => rTX_BYTE,      
		iTX_Enable => rTX_Enable,
		iCLK       => iCLK      
		);
  
end structural;
