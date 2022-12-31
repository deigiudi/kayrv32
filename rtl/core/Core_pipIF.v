/*
--------------------------------------------------------------------------------
-- COPYRIGHT (c) 2022, Alessandro Dei Giudici <alessandro.deig@live.it>
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
-- Project : KayRV32                                                           -
-- Function: KayRV32 Istruction Fetch stage                      					 -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"

module Core_pipIF (
	// System
	input wire 			 	 i_Clk,
	input wire 			 	 i_Rstn,
	// Control input
	input wire				 i_StallEn,		// stall enabler
	input wire				 i_FlushEn,		// flush enabler
	// Branch
	input wire 			 	 i_BranchEn,	// branch enabler
	input wire [`MemAddr] i_BranchAddr, // branch address to new location
	// Jump
	input wire 			 	 i_JumpEn,		// jump enabler
	input wire [`MemAddr] i_JumpAddr,   // jump address to new location
	// Istruction Memory
	output reg 			 	 o_ReadEn,     // read op enabler	
	output reg [`MemAddr] o_ReadAddr,   // istruction address to memory
	output reg [`MemAddr] o_PC,			// istruction address to decode	
	// Control output
	output reg         	 o_Event			// error event notifier
	);

	// Program Counter register
	wire [`PCWidth] w_PC;
	reg  [`PCWidth] r_nextPC;

	// Program Counter logic
	always @(posedge i_Clk)
	begin
	   if (i_Rstn==1'b0) begin  // Reset is active
			o_Event 	  <=  1'b0;
			r_nextPC   <= 32'b0;
			o_ReadEn	  <=  1'b0;
			o_ReadAddr <= 32'b0;
		end else begin
			o_Event <=  1'b0;			
			if (i_FlushEn) begin  // Flush pipeline
				w_PC <= 32'b0;
			end else begin        // Normal operation
				w_ReadEn = 1'b1;
			   case ({i_JumpEn, i_BranchEn, i_StallEn})
				2'b000 : // No stall, branch or jump
					w_PC  <= r_nextPC + 4;			
				2'b001 : // Stall ==================
					w_PC  <= r_nextPC - 4;	
				2'b010 : // Branch =================
					w_PC  <= i_BranchAddr;
				2'b100 : // Jump ===================
					w_PC  <= i_JumpAddr;
				default: begin
					w_ReadEn	<=  1'b0;
					o_Event  <=  1'b1;
					r_nextPC <= 32'b0;
					end
				endcase
			end
			o_ReadEn   <= w_ReadEn;
			o_ReadAddr <= w_PC;
			o_PC       <= r_PC;			
			r_PC 		  <= w_PC;
		end
	end
endmodule