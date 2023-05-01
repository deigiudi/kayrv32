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
-- Function    : Istruction Decode stage
-- Description : Decodes instruction fetched in the previous block and handles
								 read/writes to the register file
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_RTL_defines.vh"
`include "riscv_opcodes.vh"

module Core_pipID (
	// System
	input wire						  i_Clk,
	input wire						  i_Rstn,
	// From Instruction Fetch module
	input wire [`PCWidth  ] i_IF_PC,  		 // program counter from fetch
	input wire [`PCWidth  ] i_IF_Instr,  	 // istruction data from fetch stage	
	// From Execution module
	input wire              i_EX_rd_wr_En, // write enabler
	input wire [`RegFAddr ] i_EX_rd_Addr,  // destination address
	input wire [`RegFWidth] i_EX_rd_Data,  // destination data
	// From WriteBack module
	input wire [`RegFAddr ] i_WB_wr_En,    // write enabler
	input wire [`RegFAddr ] i_WB_rd_Addr,  // destination address
	input wire [`RegFWidth] i_WB_rd_Data,  // destination data
	// Output
	output reg [`PortSel  ] o_Port_sel,	   // execution port selector
	output reg [`OperSel  ] o_Oper_sel,	   // operation selector
	output reg [`InputSel ] o_Input_sel,   // input selector
	output reg [`RegFAddr ] o_rs1_Addr,    // operand 1 address for the ALU
	output reg [`RegFAddr ] o_rs2_Addr,    // operand 2 address for the ALU
	output reg [`RegFAddr ] o_rd_Addr,     // register destination address
	output reg              o_rd_wr_En,    // destination register writeback enabler
	output reg [`RegFWidth] o_rs1_Data,    // operand 1 data for the ALU
	output reg [`RegFWidth] o_rs2_Data,    // operand 2 data for the ALU
	output reg [`RegFWidth] o_imm1,        // immediate 1 operand
	output reg [`RegFWidth] o_imm2,        // immediate 2 operand
	// Control IO
	input wire						  i_StallEn,	   // Stall enabler
	input wire						  i_FlushEn,	   // Flush enabler
	output reg						  o_Event			   // ECALL or EBREAK detected
	);

	// Decoding the received istruction
	wire [ 6:0] w_opcode = i_IF_Instr[ 6: 0];
	wire [ 2:0] w_funct3 = i_IF_Instr[14:12];
	wire [ 6:0] w_funct7 = i_IF_Instr[31:25];
	wire [ 4:0] w_rs1    = i_IF_Instr[19:15];
	wire [ 4:0] w_rs2    = i_IF_Instr[24:20];
	wire [ 4:0] w_rd     = i_IF_Instr[11: 7];
	wire [11:0] w_imm_I = {{20{i_IF_Instr[31]}}, i_IF_Instr[31:20]};
	wire [19:0] w_imm_U = {i_IF_Instr[31:12], {12{1'b0}}};
	wire [11:0] w_imm_S = {{20{i_IF_Instr[31]}}, i_IF_Instr[31:25], i_IF_Instr[11:7]};
	wire [11:0] w_imm_B = {{20{i_IF_Instr[31]}}, i_IF_Instr[7], i_IF_Instr[30:25], i_IF_Instr[11:8], 1'b0};
	wire [11:0] w_imm_J = {{12{i_IF_Instr[31]}}, i_IF_Instr[19:12], i_IF_Instr[20], i_IF_Instr[30:21], 1'b0};

	// RISC-V RV32I 32 bit register file
	reg  [`RegFWidth] r_RegFile[`RegFDepth];
  reg  [`RegFWidth] r_rs1_data;
  reg  [`RegFWidth] r_rs2_data;

  wire [`RegFWidth] w_rs1_data;
  wire [`RegFWidth] w_rs2_data;
  wire        			w_rs1_Addr_match; 
  wire        			w_rs2_Addr_match;
	
  integer i; // needed to reset to register file

	always @(posedge i_Clk)
	begin
		if (i_Rstn == 1'b0 || i_FlushEn == 1'b1) begin
			// Instruction Decode related
			o_Port_sel  <= `PORT_NOP;
			o_Oper_sel  <= `OP_NOP;
			o_Input_sel <= `IN_RS2_RS1;
			o_rs1_Addr  <=  5'b0;
			o_rs2_Addr  <=  5'b0;
			o_rd_Addr   <=  5'b0;
			o_rs1_Data  <= 32'b0;
			o_rs2_Data  <= 32'b0;
      o_imm1      <= 32'b0; 
      o_imm2      <= 32'b0;
      o_rd_wr_En  <=  1'b0;
			o_Event 		<=  1'b0;
			// Register File related
	    r_rs1_data  <= 32'b0;
	    r_rs2_data  <= 32'b0;
      for(i=1; i<32; i=i+1) begin
        r_RegFile[i] <= 32'b0;
      end 	    
		end else begin
			if (i_StallEn == 1'b1) begin
				o_Port_sel  <= o_Port_sel;
				o_Oper_sel  <= o_Oper_sel;
				o_Input_sel <= o_Input_sel;
				o_rs1_Addr  <= o_rs1_Addr;
				o_rs2_Addr  <= o_rs2_Addr;
				o_rd_Addr   <= o_rd_Addr;
				o_rs1_Data  <= o_rs1_Data;
				o_rs2_Data  <= o_rs2_Data;
		    o_imm1      <= o_imm1; 
		    o_imm2      <= o_imm2;
		    o_rd_wr_En  <= o_rd_wr_En;
			end else begin
				// =====================================================================
				// Istruction Decode logic                                            ==
				// =====================================================================
				// Default values, might be overwritten
				o_Port_sel = `PORT_NOP;
				o_Oper_sel = `OP_NOP;
				o_rs1_Addr =  w_rs1;
				o_rs2_Addr =  w_rs2; 
				o_rd_Addr  =  w_rd;
	      o_imm1     = 32'b0; 
	      o_imm2     = 32'b0;
	      o_rd_wr_En =  1'b0;
				o_Event    =  1'b0;				
				case (w_opcode)	
					// U-type: immediate-immediate
					`OP_LUI   : begin
						o_Port_sel  <= `PORT_ARITH;
						o_Oper_sel  <= `OP_ADD;
						o_Input_sel <= `IN_IMM2_IMM1;
						o_rs1_Addr  <=  5'b0;
						o_rs2_Addr  <=  5'b0;
						o_imm2      <=  w_imm_U;
						o_rd_wr_En  <=  (w_rd == 0)? 0 : 1;
					end
					// U-type: immediate-immediate					
					`OP_AUIPC : begin
						o_Port_sel  <= `PORT_ARITH;
						o_Oper_sel  <= `OP_ADD;
						o_Input_sel <= `IN_IMM2_IMM1;
						o_rs1_Addr  <=  5'b0;
						o_rs2_Addr  <=  5'b0;
						o_imm1      <=  i_IF_PC;
						o_imm2      <=  w_imm_U;
						o_rd_wr_En  <=  (w_rd == 0)? 0 : 1;
					end
					// J-Type:
					`OP_JAL   : begin
						o_Port_sel  <= `PORT_JUMP;
						o_Oper_sel  <= `OP_JAL;
						o_Input_sel <= `IN_IMM2_IMM1;
						o_rs1_Addr  <=  5'b0;
						o_rs2_Addr  <=  5'b0;
						o_imm1      <=  i_IF_PC;
						o_imm2      <=  w_imm_J;
						o_rd_wr_En  <=  (w_rd == 0)? 0 : 1;						
					end
					// I-Type:
					`OP_JALR  : begin
						o_Port_sel  <= `PORT_JUMP;
						o_Oper_sel  <= `OP_JALR;		
						o_Input_sel <= `IN_IMM2_RS1;
						o_rs2_Addr  <=  5'b0;
						o_imm1      <=  i_IF_PC;
						o_imm2      <=  w_imm_I;
						o_rd_wr_En  <=  (w_rd == 0)? 0 : 1;
					end
					// B-Type: register-register
					`OP_BRANCH: begin
						o_Port_sel  <= `PORT_BRANCH;
						o_Input_sel <= `IN_RS2_RS1;
						o_rd_Addr   <=  5'b0;
						o_imm1      <=  i_IF_PC;
						o_imm2      <=  w_imm_B;
						case (w_funct3)
							`FUNCT3_BEQ : o_Oper_sel <= `OP_BEQ;
							`FUNCT3_BNE : o_Oper_sel <= `OP_BNE;
							`FUNCT3_BLT : o_Oper_sel <= `OP_BLT;
							`FUNCT3_BLTU: o_Oper_sel <= `OP_BLTU;
							`FUNCT3_BGE : o_Oper_sel <= `OP_BGE;
							`FUNCT3_BGEU: o_Oper_sel <= `OP_BGEU;
							default: o_Event <= 1'b1;
						endcase
					end
					// I-Type: register-immediate
					`OP_LOAD  : begin
						o_Port_sel  <= `PORT_LOAD;
						o_Input_sel <= `IN_IMM2_RS1;
						o_rs2_Addr  <=  5'b0;
						o_imm2      <=  w_imm_I;
						o_rd_wr_En  <=  (w_rd == 0)? 0 : 1;
						case (w_funct3)						
							`FUNCT3_LW : o_Oper_sel <= `OP_LW;
							`FUNCT3_LH : o_Oper_sel <= `OP_LH;
							`FUNCT3_LHU: o_Oper_sel <= `OP_LHU;
							`FUNCT3_LB : o_Oper_sel <= `OP_LB;
							`FUNCT3_LBU: o_Oper_sel <= `OP_LBU;
							default: o_Event <= 1'b1;
						endcase
					end
					// S-Type: register-immediate
					`OP_STORE : begin
						o_Port_sel  <= `PORT_STORE;
						o_Input_sel <= `IN_IMM2_RS1;
						o_imm2      <= w_imm_S;
						case (w_funct3)
							`FUNCT3_SW: o_Oper_sel <= `OP_SW;
							`FUNCT3_SH: o_Oper_sel <= `OP_SH;
							`FUNCT3_SB: o_Oper_sel <= `OP_SB;
							default: o_Event <= 1'b1;
						endcase
					end
					// I-Type: register-immediate
					`OP_IMMED : begin
						o_Input_sel <= `IN_IMM2_RS1;
						o_rs2_Addr  <=  5'b0;	
						o_imm2      <=  w_imm_I;
						o_rd_wr_En  <=  (w_rd == 0)? 0 : 1;
						case (w_funct3)					
							`FUNCT3_ADDI : begin
								o_Port_sel <= `PORT_ARITH;
								o_Oper_sel <= `OP_ADD;
							end
							`FUNCT3_SLTI : begin
								o_Port_sel <= `PORT_ARITH;
								o_Oper_sel <= `OP_SLT;
							end
							`FUNCT3_SLTIU: begin
								o_Port_sel <= `PORT_ARITH;
								o_Oper_sel <= `OP_SLTU;
							end
							`FUNCT3_XORI : begin
								o_Port_sel <= `PORT_LOGIC;
								o_Oper_sel <= `OP_XOR;
							end
							`FUNCT3_ORI  : begin
								o_Port_sel <= `PORT_LOGIC;
								o_Oper_sel <= `OP_OR;
							end
							`FUNCT3_ANDI : begin
								o_Port_sel <= `PORT_LOGIC;
								o_Oper_sel <= `OP_AND;
							end
							`FUNCT3_SLLI : begin
								o_Port_sel <= `PORT_SHIFT;
								o_Oper_sel <= `OP_SLL;
							end
							`FUNCT3_SRxI : begin
								o_Port_sel <= `PORT_SHIFT;
								o_Oper_sel <= (w_funct7)? `OP_SRA : `OP_SRL;
							end						
							default: o_Event <= 1'b1;
						endcase
					end
	        // R-Type: register-register
					`OP_REGS  : begin
						o_Input_sel <= `IN_IMM2_IMM1;
						o_rd_wr_En  <=  1'b1;
						case (w_funct3)					
							`FUNCT3_ADDSUB: begin
								o_Port_sel <= `PORT_ARITH;
								o_Oper_sel <= (w_funct7)? `OP_SUB : `OP_ADD;
							end
							`FUNCT3_SLL : begin
								o_Port_sel <= `PORT_SHIFT;
								o_Oper_sel <= `OP_SLL;
							end
							`FUNCT3_SLT : begin
								o_Port_sel <= `PORT_ARITH;
								o_Oper_sel <= `OP_SLT;
							end
							`FUNCT3_SLTU: begin
								o_Port_sel <= `PORT_ARITH;
								o_Oper_sel <= `OP_SLTU;
							end
							`FUNCT3_XOR : begin
								o_Port_sel <= `PORT_LOGIC;
								o_Oper_sel <= `OP_XOR;
							end
							`FUNCT3_SRx : begin
								o_Port_sel <= `PORT_SHIFT;
								o_Oper_sel <= (w_funct7)? `OP_SRA : `OP_SRL;						
							end
							`FUNCT3_OR  : begin
								o_Port_sel <= `PORT_LOGIC;
								o_Oper_sel <= `OP_OR;
							end
							`FUNCT3_AND : begin
								o_Port_sel <= `PORT_LOGIC;
								o_Oper_sel <= `OP_AND;
							end						
							default: o_Event <= 1'b1;
						endcase
					end
					default: o_Event <= 1'b1;
				endcase // END of INSTRUCTION DECODE structure


				// =====================================================================
				// Register File logic                                                ==
				// =====================================================================
				// Always force REG0 to zero
        r_RegFile[0] <= 0;

			  // Write to Register File
	      if((i_WB_wr_En) && (i_WB_rd_Addr != 0)) begin
	        r_RegFile[i_WB_rd_Addr] <= i_WB_rd_Data;
	      end

		    // Avoid collision arising when same data needs to be used by the
		    // current instruction and written in the register file
		    o_rs1_Data <= ((o_rs1_Addr == i_WB_rd_Addr) && i_WB_wr_En)
		    							? i_WB_rd_Data : r_RegFile[o_rs1_Addr];

		    o_rs2_Data <= ((o_rs2_Addr == i_WB_rd_Addr) && i_WB_wr_En)
		    							? i_WB_rd_Data : r_RegFile[o_rs2_Addr];
			end
		end
	end
endmodule