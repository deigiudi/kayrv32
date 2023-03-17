/*
--------------------------------------------------------------------------------
-- COPYRIGHT (c) 2023, Alessandro Dei Giudici <alessandro.deig@live.it>
--------------------------------------------------------------------------------
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" -
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   -
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  -
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   -
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         -
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        -
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    -
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     -
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     -
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  -
-- POSSIBILITY OF SUCH DAMAGE.                                                 -
--------------------------------------------------------------------------------
-- Project     : KayRV32
-- Function    : Istruction Fetch stage
-- Description : Fetches instruction from memory based on necessity of jump or
                 branch to be performed, pipeline stalling or normal operation
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"


module Core_pipIF (
	// System
	input wire						 i_Clk,
	input wire             i_Rstn,
	// Branch & Jump	
	input wire             i_EX_Branch_En,    // Branch enabler
	input wire [`MemAddr ] i_EX_Branch_Addr,  // Branch address to new location
	// Istruction Memory
	input wire [`BusWidth] i_IMEM_Data,       // Instruction fetched from iMemory
	output reg             o_IMEM_En,         // Instruction read enabler
	output reg [`MemAddr ] o_IMEM_Addr,       // Instruction address to iMemory
	// Output
	output reg [`BusWidth] o_InstrData,       // Instruction data to decode
	output reg [`MemAddr ] o_PC,              // Program counter to decode	
	// Control IO
	input wire             i_StallEn,         // Stall enabler
	input wire             i_FlushEn,         // Flush enabler
	output reg             o_Event            // Error event notifier
	);

	// Program Counter register
	reg  [`PCWidth] r_nextPC;

	always @(posedge i_Clk)
	begin
	  if (i_Rstn == 1'b0) begin
	  	// Reset is active
			o_IMEM_En   <= 1;
			o_IMEM_Addr <= 0;
			o_InstrData <= 0;			
			o_PC				<= 0;
			o_Event			<= 0;			
		end else begin
			o_Event     = 0;			
			if (i_FlushEn == 1'b1) begin
				// Flush pipeline
				o_IMEM_En   <= 0;
				o_InstrData <= 0;
			end else begin
				// Normal operation
				o_IMEM_En     = 1;
			  case ({i_EX_Branch_En, i_StallEn})
					2'b00 : // No branch or stall =
						r_nextPC  = o_IMEM_Addr + 4;	
					2'b01 : // Stall ==============
						r_nextPC  = o_IMEM_Addr - 4;
					2'b10 : // Branch =============
						r_nextPC  = i_EX_Branch_Addr;
					default: begin
						r_nextPC  = 0;
						o_Event   = 1;
						end
				endcase
				// Align PC to o_IMEM_Data 
				o_IMEM_Addr <= r_nextPC;
				o_InstrData <= i_IMEM_Data;
				o_PC				<= (i_StallEn) ? o_PC : o_IMEM_Addr;				
			end
		end
	end
endmodule