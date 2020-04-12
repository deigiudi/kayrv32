--  UART TX -- UART Transmitter pheripheral compatible with the BenchRV32 RISC-V Core
--  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
--  
--  A file containing the Transmitter Component of the UART
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
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
 
entity UART_TX is generic (
		gCLKS_x_BIT : integer := 115     -- Needs to be set correctly
		);
	port (
		oTX_Done   : out std_logic;  
		oTX_Serial : out std_logic;		
		iTX_Byte   : in  std_logic_vector(7 downto 0);
		iTX_Enable : in  std_logic;
		iCLK       : in  std_logic   
		);
end UART_TX;
 
 
architecture RTL of UART_TX is
	type tSM_Main is (sIdle, sTX_StartBit, sTX_DataBits, sTX_StopBit, sCleanup);
	signal rSM_Main : tSM_Main := sIdle;

	signal rClk_Count : integer range 0 to gCLKS_x_BIT-1 := 0;
	signal rBit_Index : integer range 0 to 7 := 0;  -- 8 Bits Total
	signal rTX_Data   : std_logic_vector(7 downto 0) := (others => '0');
	signal rTX_Done   : std_logic := '0';
   
begin

		UART_TX : process (iCLK) begin
		case rSM_Main is
			-- Idle State
			when sIdle =>
				oTX_Serial <= '1';         -- Drive Line High for Idle
				rTX_Done   <= '0';
				rClk_Count <= 0;
				rBit_Index <= 0;
				
				if iTX_Enable = '1' then
					rTX_Data <= iTX_Byte;
					rSM_Main <= sTX_StartBit;
				else
					rSM_Main <= sIdle;
				end if;

			-- Communication begins. Start bit = 0
			when sTX_StartBit =>
				oTX_Serial <= '0';

				-- Wait gCLKS_x_BIT-1 clock cycles for start bit to finish
				if rClk_Count < gCLKS_x_BIT-1 then
					rClk_Count <= rClk_Count + 1;
					rSM_Main   <= sTX_StartBit;
				else
					rClk_Count <= 0;
					rSM_Main   <= sTX_DataBits;
				end if;

			-- Wait gCLKS_x_BIT-1 clock cycles        
			when sTX_DataBits =>
				oTX_Serial <= rTX_Data(rBit_Index);

				if rClk_Count < gCLKS_x_BIT-1 then
					rClk_Count <= rClk_Count + 1;
					rSM_Main   <= sTX_DataBits;
				else
					rClk_Count <= 0;

					-- Check if we have sent out all bits
					if rBit_Index < 7 then
						rBit_Index <= rBit_Index + 1;
						rSM_Main   <= sTX_DataBits;
					else
						rBit_Index <= 0;
						rSM_Main   <= sTX_StopBit;
					end if;
				end if;

			-- Communication ends. Stop bit = 1
			when sTX_StopBit =>
				oTX_Serial <= '1';

				-- Wait gCLKS_x_BIT-1 clock cycles
				if rClk_Count < gCLKS_x_BIT-1 then
					rClk_Count <= rClk_Count + 1;
					rSM_Main   <= sTX_StopBit;
				else
					rTX_Done   <= '1';
					rClk_Count <= 0;
					rSM_Main   <= sCleanup;
				end if;

			-- Preparations for a new transaction
			when sCleanup =>
				rTX_Done <= '1';
				rSM_Main <= sIdle;

			when others =>
				rSM_Main <= sIdle;

			end case;
		end process UART_TX;

	oTX_Done <= rTX_Done;
   
end RTL;
