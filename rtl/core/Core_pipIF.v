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
	input wire						i_Clk,
	input wire						i_Rstn,
	// Control input
	input wire						i_StallEn,		// stall enabler
	input wire						i_FlushEn,		// flush enabler
	// Branch
	input wire						i_BranchEn,	// branch enabler
	input wire [`MemAddr] i_BranchAddr, // branch address to new location
	// Jump
	input wire						i_JumpEn,		// jump enabler
	input wire [`MemAddr] i_JumpAddr,   // jump address to new location
	// Istruction Memory
	input wire [`MemData] i_InstrData,  // instruction fetched from iMemory
	output reg            o_InstrEn,    // instruction read enabler
	output reg [`MemAddr] o_InstrAddr,  // instruction address to memory
	output reg [`MemData] o_InstrData,  // instruction data to decode
	output reg [`MemAddr] o_PC,			// program counter to decode	
	// Control output
	output reg						o_Event			// error event notifier
	);

	// Program Counter register
	reg  [`PCWidth] r_nextPC;

	// Program Counter logic
	always @(posedge i_Clk)
	begin
	   if (i_Rstn==1'b0) begin  // Reset is active
			o_Event			<= 0;
			o_PC				<= 0;
			o_InstrEn   <= 0;
			o_InstrAddr <= 0;
			o_InstrData <= 0;
		end else begin
			o_Event <= 0;			
			if (i_FlushEn) begin  // Flush pipeline
				o_InstrEn   <= 0;
				o_InstrData <= 0;
			end else begin        // Normal operation
			   case ({i_JumpEn, i_BranchEn, i_StallEn})
					2'b000 : // No jump, branch or stall
						r_nextPC  <= o_InstrAddr + 4;			
					2'b001 : // Stall ==================
						r_nextPC  <= o_InstrAddr - 4;	
					2'b010 : // Branch =================
						r_nextPC  <= i_BranchAddr;
					2'b100 : // Jump ===================
						r_nextPC  <= i_JumpAddr;
					default: begin
						o_Event  <= 1;
						r_nextPC <= 0;
						end
				endcase
				o_InstrAddr <= r_nextPC;
				o_InstrData <= i_InstrData;
			end
		end
	end
endmodule