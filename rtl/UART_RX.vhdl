--  UART_RX -- RX Receiver of UART pheripheral compatible with the BenchRV32 RISC-V Core
--  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
--  
--  A file containing the Receiver Component of the UART
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
 
entity UART_RX is generic (
		gCLKS_x_BIT : integer := 115     -- Needs to be set correctly
		);
	port (
		oRX_Byte   : out std_logic_vector(7 downto 0);
		oRX_GotIt  : out std_logic;  
		iRX_Serial : in  std_logic;  
		iCLK       : in  std_logic     
		);
end UART_RX;
 
 
architecture rtl of UART_RX is
	type tSM_Main is (sIdle, sRX_StartBit, sRX_DataBits, sRX_StopBit, sCleanup);
	signal rSM_Main : tSM_Main := sIdle;

	signal rRX_Data_R : std_logic := '0';
	signal rRX_Data   : std_logic := '0';
	signal rClk_Count : integer range 0 to gCLKS_x_BIT-1 := 0;
	signal rBit_Index : integer range 0 to 7 := 0;  -- 8 Bits Total
	signal rRX_Byte   : std_logic_vector(7 downto 0) := (others => '0');
	signal rRX_GotIt  : std_logic := '0';
   
begin
	Metastability : process (iCLK)
	begin
		if rising_edge(iCLK) then
			rRX_Data_R <= iRX_Serial;
			rRX_Data   <= rRX_Data_R;
		end if;
	end process Metastability;

	UART_RX : process (iCLK)
	begin
		if rising_edge(iCLK) then
			case rSM_Main is

				when sIdle =>
					rRX_GotIt     <= '0';
					rClk_Count <= 0;
					rBit_Index <= 0;
					-- Start bit detected
					if rRX_Data = '0' then       
						rSM_Main <= sRX_StartBit;
					else
						rSM_Main <= sIdle;
					end if;

				-- Check middle of start bit to make sure it's still low
				when sRX_StartBit =>
					if rClk_Count = (gCLKS_x_BIT-1)/2 then
						if rRX_Data = '0' then
							rClk_Count <= 0;  -- reset counter since we found the middle
							rSM_Main   <= sRX_DataBits;
						else
							rSM_Main   <= sIdle;
						end if;
					else
						rClk_Count <= rClk_Count + 1;
						rSM_Main   <= sRX_StartBit;
					end if;

				-- Wait gCLKS_x_BIT-1 clock cycles to sample serial data
				when sRX_DataBits =>
					if rClk_Count < gCLKS_x_BIT-1 then
						rClk_Count <= rClk_Count + 1;
						rSM_Main   <= sRX_DataBits;
					else
						rClk_Count            <= 0;
						rRX_Byte(rBit_Index) <= rRX_Data;
						-- Check if we have sent out all bits
						if rBit_Index < 7 then
							rBit_Index <= rBit_Index + 1;
							rSM_Main   <= sRX_DataBits;
						else
							rBit_Index <= 0;
							rSM_Main   <= sRX_StopBit;
						end if;
					end if;

				-- Receive Stop bit.  Stop bit = 1
				when sRX_StopBit =>
					-- Wait gCLKS_x_BIT-1 clock cycles for Stop bit to finish
					if rClk_Count < gCLKS_x_BIT-1 then
						rClk_Count <= rClk_Count + 1;
						rSM_Main   <= sRX_StopBit;
					else
						rRX_GotIt     <= '1';
						rClk_Count <= 0;
						rSM_Main   <= sCleanup;
					end if;

			-- Preparations for a new transaction
				when sCleanup =>
					rSM_Main <= sIdle;
					rRX_GotIt   <= '0';

				when others =>
					rSM_Main <= sIdle;

				end case;
			end if;
		end process UART_RX;

	oRX_GotIt   <= rRX_GotIt;
	oRX_Byte <= rRX_Byte;
   
end rtl;
