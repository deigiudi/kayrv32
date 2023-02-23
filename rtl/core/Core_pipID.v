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
-- Function: KayRV32 Istruction Decode stage                   					 -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"
`include "riscv_opcodes.vh"

module Core_pipID (
	// System
	input wire						i_Clk,
	input wire						i_Rstn,
	// Control input
	input wire						i_StallEn,	 // stall enabler
	input wire						i_FlushEn,	 // flush enabler
	// Data from memory
	input wire [`MemAddr  ] i_PC,  		 // program counter from fetch
	input wire [`MemData  ] i_Istr,  	 // istruction data from fetch stage
	// Data to memory
	input wire [`RegFAddr ] i_MemAddr,   // memory address from writeback
	input wire [`RegFWidth] i_MemData,   // memory data from writeback	
	// Data to register file
	input wire              i_ExeEn,		 // write enabler from execute stage
	input wire [`RegFAddr ] i_ExeAddr,   // register file address from execute
	input wire [`RegFWidth] i_ExeData,   // register file data from execute
	// Output to decode stage
	output reg [`MemAddr	] o_PC, 		 // program counter passthrough	
	output reg [`PortSel  ] o_Port_sel,	 // execution port selector
	output reg [`OperSel  ] o_Oper_sel,	 // operation selector
	output reg [`InputSel ] o_Input_sel, // input selector
	output reg [`RegFAddr ] o_RegFAddr,  // destination address	
	output reg [`RegFWidth] o_src1, 	 	 // operand 1 for the ALU
	output reg [`RegFWidth] o_src2, 	 	 // operand 2 for the ALU
	output reg [`RegFWidth] o_offset, 	 // immediate operand
	// Control output
	output reg              o_Event      // ECALL or EBREAK detected
	);

	// RISC-V RV32I 32 bit register file
	reg [`RegFWidth] r_RegFile [`RegFDepth];

	// Decoding the istructions
	wire [6:0]  w_opcode =  i_Istr[ 6: 0];
	wire [4:0]  w_rd 	   =  i_Istr[11: 7];
	wire [4:0]  w_rs1 	 =  i_Istr[19:15];
	wire [4:0]  w_rs2 	 =  i_Istr[24:20];
	wire [2:0]  w_funct3 =  i_Istr[14:12];
	wire [6:0]  w_funct7 =  i_Istr[31:25];
	wire [11:0] w_imm_I  = {{20{i_Istr[31]}}, i_Istr[31:20]};
	wire [19:0] w_imm_U  = {i_Istr[31:12], {12{1'b0}}};
	wire [11:0] w_imm_S  = {{20{i_Istr[31]}}, i_Istr[31:25], i_Istr[11:7]};
	wire [11:0] w_imm_B  = {{20{i_Istr[31]}}, i_Istr[7], i_Istr[30:25], i_Istr[11:8], 1'b0};
	wire [11:0] w_imm_J  = {{12{i_Istr[31]}}, i_Istr[19:12], i_Istr[20], i_Istr[30:21], 1'b0};

	/* Decoder cases structure */
	always @(posedge i_Clk)
	begin
      // Force REG1 to ZERO
      r_RegFile[0] = 0;	
	
      // Program counter passtrough
      o_PC = i_PC;
      
		if (i_Rstn==1'b0 || i_StallEn==1'b1) begin	
			o_Port_sel  <= `PORT_NOP;
			o_Oper_sel  <= `OP_NOP;
			o_Input_sel <= `IN_OP1_OP2;
			o_src1      <= 32'b0;
			o_src2      <= 32'b0;
			o_offset    <= 32'b0;
			o_RegFAddr  <= 4'b0;
			o_Exception <= 1'b0;
			o_Event 		<= 1'b0;
		end else begin
			o_src1      <= 32'b0;
			o_src2      <= 32'b0;			
			o_Exception <= 1'b0;
			o_Event 		<= 1'b0;
			case (w_opcode)	
				`OP_LUI   : begin // LUI ==========
					o_Port_sel  <= `PORT_ARITH;
					o_Oper_sel  <= `OP_ADD;
					o_Input_sel <= `IN_OP1_OFF;
					o_offset    <= w_imm_U;
					o_RegFAddr  <= w_rd;
				end
				`OP_AUIPC : begin // AUIPC ========
					o_Port_sel  <= `PORT_ARITH;
					o_Oper_sel  <= `OP_ADD;
					o_Input_sel <= `IN_PC_OFF;
					o_offset    <= w_imm_U;					
					o_RegFAddr  <= w_rd;
				end
				`OP_JAL   : begin // JAL ==========
					o_Port_sel  <= `PORT_JUMP;
					o_Oper_sel  <= `OP_JAL;
					o_Input_sel <= `IN_PC_OFF;
					o_offset    <= w_imm_J;
					o_RegFAddr  <= w_rd;
				end
				`OP_JALR  : begin // JALR =========
					o_Port_sel  <= `PORT_JUMP;
					o_Oper_sel  <= `OP_JALR;		
					o_Input_sel <= `IN_OP1_OFF;				
					o_src1      <= r_RegFile[w_rs1];
					o_offset    <= w_imm_J;					
					o_RegFAddr  <= w_rd;
				end
				`OP_BRANCH: begin // BRANCH =======
					o_Port_sel  <= `PORT_BRANCH;
					o_Input_sel <= `IN_OP1_OP2;
					o_src1      <= r_RegFile[w_rs1];
					o_src2      <= r_RegFile[w_rs2];
					o_offset    <= w_imm_B;
					o_RegFAddr  <= w_rd;					
					case (w_funct3)
						`FUNCT3_BEQ : o_Oper_sel <= `OP_BEQ;
						`FUNCT3_BNE : o_Oper_sel <= `OP_BNE;
						`FUNCT3_BLT : o_Oper_sel <= `OP_BLT;
						`FUNCT3_BLTU: o_Oper_sel <= `OP_BLTU;
						`FUNCT3_BGE : o_Oper_sel <= `OP_BGE;
						`FUNCT3_BGEU: o_Oper_sel <= `OP_BGEU;
						default: begin							
							o_Port_sel <= `PORT_NOP;
							o_Oper_sel <= `OP_NOP;
							o_Event    <= 1'b1;
						end
					endcase
				end
				`OP_LOAD  : begin // LOAD =========
					o_Port_sel  <= `PORT_LOAD;
					o_Input_sel <= `IN_OP1_OFF;
					o_src1      <= r_RegFile[w_rs1];
					o_offset    <= w_imm_I;
					o_RegFAddr  <= w_rd;
					case (w_funct3)						
						`FUNCT3_LW : o_Oper_sel <= `OP_LW;
						`FUNCT3_LH : o_Oper_sel <= `OP_LH;
						`FUNCT3_LHU: o_Oper_sel <= `OP_LHU;
						`FUNCT3_LB : o_Oper_sel <= `OP_LB;
						`FUNCT3_LBU: o_Oper_sel <= `OP_LBU;
						default: begin
							o_Port_sel <= `PORT_NOP;
							o_Oper_sel <= `OP_NOP;							
							o_Event    <= 1'b1;
						end
					endcase
				end
				`OP_STORE : begin // STORE ========
					o_Port_sel  <= `PORT_STORE;
					o_Input_sel <= `IN_OP1_OFF;
					o_src1      <= r_RegFile[w_rs1];
					o_offset    <= w_imm_I;
					o_RegFAddr  <= r_RegFile[w_rs2];					
					case (w_funct3)
						`FUNCT3_SW: o_Oper_sel <= `OP_SW;
						`FUNCT3_SH: o_Oper_sel <= `OP_SH;
						`FUNCT3_SB: o_Oper_sel <= `OP_SB;
						default: begin
							o_Port_sel <= `PORT_NOP;
							o_Oper_sel <= `OP_NOP;							
							o_Event    <= 1'b1;
						end
					endcase
				end
				`OP_IMMED : begin // IMMEDIATE ====
					o_Input_sel <= `IN_OP1_OFF;
					o_src1      <= r_RegFile[w_rs1];
					o_offset    <= w_imm_I;
					o_RegFAddr  <= w_rd;						
					case (w_funct3)					
						`FUNCT3_ADDI : begin // ADDI
							o_Port_sel <= `PORT_ARITH;
							o_Oper_sel <= `OP_ADD;
						end
						`FUNCT3_SLTI : begin // SLTI
							o_Port_sel <= `PORT_ARITH;
							o_Oper_sel <= `OP_SLT;
						end
						`FUNCT3_SLTIU: begin // SLTIU
							o_Port_sel <= `PORT_ARITH;
							o_Oper_sel <= `OP_SLTU;
						end
						`FUNCT3_XORI : begin // XORI
							o_Port_sel <= `PORT_LOGIC;
							o_Oper_sel <= `OP_XOR;
						end
						`FUNCT3_ORI  : begin // ORI
							o_Port_sel <= `PORT_LOGIC;
							o_Oper_sel <= `OP_OR;
						end
						`FUNCT3_ANDI : begin // ANDI
							o_Port_sel <= `PORT_LOGIC;
							o_Oper_sel <= `OP_AND;
						end
						`FUNCT3_SLLI : begin // SLLI
							o_Port_sel <= `PORT_SHIFT;
							o_Oper_sel <= `OP_SLL;
						end
						`FUNCT3_SRxI : begin // SRLI or SRAI
							o_Port_sel <= `PORT_SHIFT;
							o_Oper_sel <= (w_funct7)? `FUNCT7_SRLI : `FUNCT7_SRAI;
						end						
						default: begin
							o_Port_sel <= `PORT_NOP;
							o_Oper_sel <= `OP_NOP;							
							o_Event    <= 1'b1;
						end
					endcase
				end
				`OP_REGS  : begin // REGISTERS ====
					o_Input_sel <= `IN_OP1_OP2;
					o_src1      <= r_RegFile[w_rs1];
					o_src2      <= r_RegFile[w_rs2];
					o_RegFAddr  <= w_rd;					
					case (w_funct3)					
						`FUNCT3_ADDSUB: begin // ADD or SUB
							o_Port_sel <= `PORT_ARITH;
							o_Oper_sel <= (w_funct7)? `FUNCT7_ADD : `FUNCT7_SUB;
						end
						`FUNCT3_SLL : begin // SLL
							o_Port_sel <= `PORT_SHIFT;
							o_Oper_sel <= `OP_SLL;
						end
						`FUNCT3_SLT : begin // SLT
							o_Port_sel <= `PORT_ARITH;
							o_Oper_sel <= `OP_SLT;
						end
						`FUNCT3_SLTU: begin // SLTU
							o_Port_sel <= `PORT_ARITH;
							o_Oper_sel <= `OP_SLTU;
						end
						`FUNCT3_XOR : begin // XOR
							o_Port_sel <= `PORT_LOGIC;
							o_Oper_sel <= `OP_XOR;
						end
						`FUNCT3_SRx : begin // SRL or SRA
							o_Port_sel <= `PORT_SHIFT;
							o_Oper_sel <= (w_funct7)? `FUNCT7_SRL : `FUNCT7_SRA;						
						end
						`FUNCT3_OR  : begin // OR
							o_Port_sel <= `PORT_LOGIC;
							o_Oper_sel <= `OP_OR;
						end
						`FUNCT3_AND : begin // AND
							o_Port_sel <= `PORT_LOGIC;
							o_Oper_sel <= `OP_AND;
						end						
						default: begin
							o_Port_sel <= `PORT_NOP;
							o_Oper_sel <= `OP_NOP;							
							o_Event    <= 1'b1;
						end
					endcase
				end
				default:	begin
					o_Port_sel <= `PORT_NOP;
					o_Oper_sel <= `OP_NOP;							
					o_Event    <= 1'b1;					
				end		
			endcase // end of opcodes case structure
		end
	end
endmodule