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
-- Function    : Exexute stage
-- Description : Execute instruction decoded in previous block. Eventually
								 directly computes branch destination to be forwarded to IF
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
	input wire [`MemAddr  ] i_ID_PC,          // program counter	
	input wire [`PortSel  ] i_ID_Port_sel,    // execution port selector
	input wire [`OperSel  ] i_ID_Oper_sel,    // operation selector
	input wire [`InputSel ] i_ID_Input_sel,   // input selector
	input wire [`RegFAddr ] i_ID_rd_Addr,     // destination address	
	input wire [`RegFWidth] i_ID_rs1_Data,    // operand 1 data for the ALU
	input wire [`RegFWidth] i_ID_rs2_Data,    // operand 2 for the ALU
	input wire [`RegFWidth] i_ID_offset,      // immediate operand
	// Data out
	output reg [`OperSel  ] o_Oper_sel,	      // operation selector
	output reg [`RegFAddr ] o_rd_Addr,        // register file address
	output reg              o_RW_En,          // write enabler
	output reg [`MemAddr  ] o_WriteAddr,      // memmory address
	output reg [`BusWidth ] o_WriteData,      // register file data
	output reg              o_BranchEn,       // jump enabler
	output reg [`PCWidth  ] o_BranchAddr,     // jump istruction address			
	// Control IO
	input wire							i_FlushEn,        // Flush enabler		
	output reg              o_Event
	);

	always @(posedge i_Clk)
		begin
    // Signal passtrough =======================================================
    assign o_Oper_sel = i_Oper_sel;	


		// Execute cases structure =================================================
		if (i_Rstn==1'b0 || i_Port_sel==`PORT_NOP) begin
			o_BranchEn  <=  1'b0;
			o_RW_En   <=  1'b0;
			o_rd_Addr   <=  4'b0;
			o_WriteAddr <= 32'b0;
			o_WriteData <= 32'b0;
			o_Event 		<=  1'b0;
		end else begin
			o_BranchEn   =  1'b0;
			o_RW_En    =  1'b0;
			o_WriteAddr  = 32'b0;
			o_rd_Addr   <= i_rd_Addr;			
			case (i_Port_sel)
				`PORT_SHIFT  : begin // SHIFT =====
					case (i_Oper_sel)
						`OP_SLL : o_WriteData = i_rs1_Data << i_rs2_Data[4:0];
						`OP_SRL : o_WriteData = i_rs1_Data >> i_rs2_Data[4:0];
						`OP_SRA : o_WriteData = $signed(i_rs1_Data) >>> i_rs2_Data[4:0]; 
						default : o_WriteData = 0;
					endcase
				end
				`PORT_LOGIC  : begin // LOGIC =====
					case (i_Oper_sel)
						`OP_AND : o_WriteData = i_rs1_Data & i_rs2_Data;
						`OP_OR  : o_WriteData = i_rs1_Data | i_rs2_Data;												
						`OP_XOR : o_WriteData = i_rs1_Data ^ i_rs2_Data;
						default : o_WriteData = 0;
					endcase
				end
				`PORT_ARITH  : begin // ARITH =====
					case (i_Oper_sel)
						`OP_ADD  : o_WriteData = i_rs1_Data + i_rs2_Data;
						`OP_SUB  : o_WriteData = i_rs1_Data - i_rs2_Data;
						`OP_SLT  : o_WriteData = ($signed(i_rs1_Data) < $signed(i_rs2_Data)) ? 1:0;
						`OP_SLTU : o_WriteData = (i_rs1_Data < i_rs2_Data) ? 1:0;
						default  : o_WriteData = 0;
					endcase
				end
				`PORT_JUMP   : begin // JUMP ======
					o_BranchEn  = 1'b1;
					o_WriteData = i_PC+4;
					case (i_Oper_sel)
						`OP_JAL  : o_BranchAddr = i_rs1_Data + i_rs2_Data;
						`OP_JALR : o_BranchAddr = i_rs1_Data - i_rs2_Data;
						default  : begin
							o_BranchEn    = 1'b0;
							o_RW_En   = 1'b0;
							o_WriteAddr = 0;
							o_WriteData = 0;
							o_Event     = 1;
						end		
					endcase						
				end
				`PORT_BRANCH : begin // BRANCH ====
					case (i_Oper_sel)
		            `OP_BEQ : o_WriteData = (i_rs1_Data == i_rs2_Data) ? 1:0;
		            `OP_BNE : o_WriteData = (i_rs1_Data == i_rs2_Data) ? 0:1;
		            `OP_BLT : o_WriteData = ($signed(i_rs1_Data) < $signed(i_rs2_Data)) ? 1:0; 
		            `OP_BLTU: o_WriteData = (i_rs1_Data < i_rs2_Data) ? 1:0;
		            `OP_BGE : o_WriteData = ($signed(i_rs1_Data) >= $signed(i_rs2_Data)) ? 1:0; 
		            `OP_BGEU: o_WriteData = (i_rs1_Data >= i_rs2_Data) ? 1:0;		            
					endcase
				end
				`PORT_LOAD   : begin // LOAD ======
					o_WriteAddr = i_rs1_Data + i_offset;
					o_WriteData = 0;
				end
				`PORT_STORE  : begin // STORE =====
					o_RW_En   = 1'b1;
					o_WriteAddr = i_rs1_Data + i_offset;
					o_WriteData = i_rs2_Data;
				end
				default:	begin
					o_WriteAddr = 0;
					o_WriteData = 0;
					o_Event     = 1;
				end		
			endcase // end of opcodes case structure
		end


		// Branch Control ==========================================================
    if(ID_branch_op_i[1]) begin
        if(ID_jump_en_i) begin
            EX_branch_en_o <= 0;
        end
        else begin
            if(ID_branch_flag_i == 1'b0) begin
                EX_branch_en_o <= (test_result) ? 1:0;            
            end
            else begin
                EX_branch_en_o <= (test_result) ? 0:1;  
            end
        end 
    end
    else begin
        EX_branch_en_o <= 0;
    end 
	end

endmodule