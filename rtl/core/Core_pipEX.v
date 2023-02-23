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
-- Function: KayRV32 Exexute stage	 			                   					 -
--------------------------------------------------------------------------------
*/
 
`timescale 1ns / 1ps
`include "kayrv32_defines.vh"
`include "riscv_opcodes.vh"

module Core_pipEX (
	// System
	input wire 							i_Clk,
	input wire 							i_Rstn,	
	// From Decode Stage
	input wire [`MemAddr	] i_PC,				 // program counter	
	input wire [`PortSel  ] i_Port_sel,	 // execution port selector
	input wire [`OperSel  ] i_Oper_sel,	 // operation selector
	input wire [`InputSel ] i_Input_sel, // input selector
	input wire [`RegFAddr ] i_RegFAddr,  // destination address	
	input wire [`RegFWidth] i_src1, 	 	 // operand 1 for the ALU
	input wire [`RegFWidth] i_src2, 	 	 // operand 2 for the ALU
	input wire [`RegFWidth] i_offset, 	 // immediate operand
	// Data out
	output reg              o_LD_WR,	 	 // Load/Write selector
	output reg [`OperSel  ] o_Oper_sel,	 // operation selector
	output reg [`RegFAddr ] o_RegFAddr,  // register file address
	output reg              o_WriteEn,	 // write enabler
	output reg [`MemAddr  ] o_WriteAddr, // memmory address
	output reg [`MemData  ] o_WriteData, // register file data
	output reg              o_JumpEn,    // jump enabler
	output reg [`PCWidth  ] o_JumpAddr,  // jump istruction address			
	// Control output
	output reg 					    o_StallEn,	 // Stall request to stall controller
	output reg              o_Event
	);

	/* Execute cases structure */	
	always @(posedge i_Clk)
	begin
    // Signal passtrough
    assign o_Oper_sel = i_Oper_sel;	
	
		if (i_Rstn==1'b0 || i_Port_sel==`PORT_NOP) begin
			o_JumpEn    <=  1'b0;
			o_WriteEn   <=  1'b0;
			o_RegFAddr  <=  4'b0;
			o_WriteAddr <= 32'b0;
			o_WriteData <= 32'b0;
			o_Event 		<=  1'b0;
		end else begin
			o_JumpEn     =  1'b0;
			o_WriteEn    =  1'b0;
			o_RegFAddr  <= i_RegFAddr;
			o_WriteAddr  = 32'b0;
			case (i_Port_sel)
				`PORT_SHIFT  : begin // SHIFT =====
					case (i_Oper_sel)
						`OP_SLL : o_WriteData = i_src1 << i_src2[4:0];
						`OP_SRL : o_WriteData = i_src1 >> i_src2[4:0];
						`OP_SRA : o_WriteData = $signed(i_src1) >>> i_src2[4:0]; 
						default : o_WriteData = 0;
					endcase
				end
				`PORT_LOGIC  : begin // LOGIC =====
					case (i_Oper_sel)
						`OP_AND : o_WriteData = i_src1 & i_src2;
						`OP_OR  : o_WriteData = i_src1 | i_src2;												
						`OP_XOR : o_WriteData = i_src1 ^ i_src2;
						default : o_WriteData = 0;
					endcase
				end
				`PORT_ARITH  : begin // ARITH =====
					case (i_Oper_sel)
						`OP_ADD  : o_WriteData = i_src1 + i_src2;
						`OP_SUB  : o_WriteData = i_src1 - i_src2;
						`OP_SLT  : o_WriteData = ($signed(i_src1) < $signed(i_src2)) ? 1:0;
						`OP_SLTU : o_WriteData = (i_src1 < i_src2) ? 1:0;
						default  : o_WriteData = 0;
					endcase
				end
				`PORT_JUMP   : begin // JUMP ======
					o_JumpEn    = 1'b1;
					o_WriteData = i_PC+4;
					case (i_Oper_sel)
						`OP_JAL  : o_JumpAddr = i_src1 + i_src2;
						`OP_JALR : o_JumpAddr = i_src1 - i_src2;
						default  : begin
							o_JumpEn    = 1'b0;
							o_WriteEn   = 1'b0;
							o_WriteAddr = 0;
							o_WriteData = 0;
							o_Event     = 1;
						end		
					endcase						
				end
				`PORT_BRANCH : begin // BRANCH ====
					case (i_Oper_sel)
		            `OP_BEQ : o_WriteData = (i_src1 == i_src2) ? 1:0;
		            `OP_BNE : o_WriteData = (i_src1 == i_src2) ? 0:1;
		            `OP_BLT : o_WriteData = ($signed(i_src1) < $signed(i_src2)) ? 1:0; 
		            `OP_BLTU: o_WriteData = (i_src1 < i_src2) ? 1:0;
		            `OP_BGE : o_WriteData = ($signed(i_src1) >= $signed(i_src2)) ? 1:0; 
		            `OP_BGEU: o_WriteData = (i_src1 >= i_src2) ? 1:0;		            
					endcase
				end
				`PORT_LOAD   : begin // LOAD ======
					o_WriteAddr = i_src1 + i_offset;
					o_WriteData = 0;
				end
				`PORT_STORE  : begin // STORE =====
					o_WriteEn   = 1'b1;
					o_WriteAddr = i_src1 + i_offset;
					o_WriteData = i_src2;
				end
				default:	begin
					o_WriteAddr = 0;
					o_WriteData = 0;
					o_Event     = 1;
				end		
			endcase // end of opcodes case structure
		end
	end

endmodule