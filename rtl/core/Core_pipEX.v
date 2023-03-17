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


module Core_pipEX (
	// System
	input wire 							i_Clk,
	input wire 							i_Rstn,
	// From Decode Stage
	input wire [`PortSel  ] i_ID_Port_sel,    // execution port selector
	input wire [`OperSel  ] i_ID_Oper_sel,    // operation selector
	input wire [`InputSel ] i_ID_Input_sel,   // input selector
	input wire [`RegFAddr ] i_ID_rs1_Addr,    // operand 1 address
	input wire [`RegFAddr ] i_ID_rs2_Addr,    // operand 2 address
	input wire [`RegFAddr ] i_ID_rd_Addr,     // destination address
	input wire              i_ID_rd_wr_en,    // destination register writeback enabler	
	input wire [`RegFWidth] i_ID_rs1_Data,    // operand 1 data
	input wire [`RegFWidth] i_ID_rs2_Data,    // operand 2 data
	input wire [`RegFWidth] i_ID_imm1,        // immediate 1 operand
	input wire [`RegFWidth] i_ID_imm2,        // immediate 2 operand
	// Forwarding
	input wire [`RegFWidth] i_MA_rd_Data,     // data forwarded from memory stage
  input wire [ 1:0      ] i_CT_forward_op1, // forwarding operand 1 control signal
  input wire [ 1:0      ] i_CT_forward_op2, // forwarding operand 2 control signal
	// Output
	output reg [`OperSel  ] o_Oper_sel,	      // operation selector
	output reg [`MemAddr  ] o_rs2_Addr,       // Result address
	output reg [`BusWidth ] o_rs2_Data,       // Computed result data
	output reg              o_rd_wr_en,       // destination register writeback enabler
	output reg [`RegFAddr ] o_rd_Addr,        // register destination address
	output reg [`BusWidth ] o_rd_Data,        // Computed result data
	output reg              o_Branch_En,      // jump enabler
	output reg [`PCWidth  ] o_Branch_Addr,    // jump istruction address			
	// Control IO
	input wire							i_FlushEn,        // Flush enabler		
	output reg              o_Event
	);

	// Input wiring for the ALU
  wire [`RegFWidth] w_op1_data;
  wire [`RegFWidth] w_op2_data;

	always @(posedge i_Clk)
	begin
		if (i_Rstn == 1'b0 || i_FlushEn == 1'b1) begin
			o_Oper_sel    <=  4'b0;
			o_rs2_Addr    <= 32'b0;
			o_rs2_Data    <= 32'b0;
			o_rd_wr_en    <=  1'b0;			
			o_rd_Addr     <=  4'b0;
			o_rd_Data     <= 32'b0;
			o_Branch_En   <=  1'b0;
			o_Branch_Addr <= (i_FlushEn) ? o_Branch_Addr : 32'b0;
			o_Event 		  <=  1'b0;
		end else begin
		  // Signal passtrough
		  o_Oper_sel <= i_ID_Oper_sel;	

			// =======================================================================
	    // ALU source input: op1                                                ==
	    // =======================================================================
      case(i_CT_forward_op1)
        // from previous cycle result
        2'b01:   w_op1_data = o_rd_Data;
        // from read data mem output
        2'b10:   w_op1_data = i_MA_rd_Data;
        // no data hazard
        default: w_op1_data = (i_ID_Input_sel[0]) ? i_ID_imm1 : i_ID_rs1_Data;
      endcase

	    // =======================================================================
	    // ALU source input: op2                                                ==
	    // =======================================================================
      case(i_CT_forward_op2)            
        // from previous cycle result
        2'b01:   w_op2_data = o_rd_Data;
        // from read data mem output
        2'b10:   w_op2_data = i_MA_rd_Data;
        // no data hazard
        default: w_op2_data = (i_ID_Input_sel[1]) ? i_ID_imm2 : i_ID_rs2_Data;      
      endcase

			// =======================================================================
			// Execute logic                                                        ==
			// =======================================================================
		  // Signal passtrough
			o_rs2_Addr  = i_ID_rs2_Addr;
			o_rs2_Data  = i_ID_rs2_Data;
			o_rd_Addr   = i_ID_rd_Addr;
			// Default values, might be overwritten
			o_rd_wr_en  =  1'b1;
			o_rd_Data   = 32'b0;
			o_Branch_En =  1'b0;			
			case (i_ID_Port_sel)
				// NOP ======
				`PORT_NOP    : begin
					o_rd_wr_en <= 1'b0;
					o_rd_Data  <= o_rd_Data;
					end
				// SHIFT =====
				`PORT_SHIFT  : begin
					case (i_ID_Oper_sel)
						`OP_SLL  : o_rd_Data <= w_op1_data << w_op2_data[4:0];
						`OP_SRL  : o_rd_Data <= w_op1_data >> w_op2_data[4:0];
						`OP_SRA  : o_rd_Data <= $signed(w_op1_data) >>> w_op2_data[4:0]; 
						default  : o_Event <= 1'b1;
					endcase
				end
				// LOGIC =====
				`PORT_LOGIC  : begin
					case (i_ID_Oper_sel)
						`OP_AND  : o_rd_Data <= w_op1_data & w_op2_data;
						`OP_OR   : o_rd_Data <= w_op1_data | w_op2_data;												
						`OP_XOR  : o_rd_Data <= w_op1_data ^ w_op2_data;
						default  : o_Event <= 1'b1;
					endcase
				end
				// ARITH =====
				`PORT_ARITH  : begin
					case (i_ID_Oper_sel)
						`OP_ADD  : o_rd_Data <= w_op1_data + w_op2_data;
						`OP_SUB  : o_rd_Data <= w_op1_data - w_op2_data;
						`OP_SLT  : o_rd_Data <= ($signed(w_op1_data) < $signed(w_op2_data)) ? 1:0;
						`OP_SLTU : o_rd_Data <= (w_op1_data < w_op2_data) ? 1:0;
						default  : o_Event <= 1'b1;
					endcase
				end
				// JUMP ======
				`PORT_JUMP   : begin
					o_Branch_En   <= 1'b1;
					o_Branch_Addr <= w_op1_data + w_op2_data;
					case (i_ID_Oper_sel)
						`OP_JAL  : o_rd_Data <= w_op1_data + 32'd4;
						`OP_JALR : o_rd_Data <= w_op1_data + 32'd4;
						default  : o_Event <= 1'b1;
					endcase						
				end
				// BRANCH ====
				`PORT_BRANCH : begin
					o_rd_wr_en    <=  1'b0;
					o_Branch_Addr <= i_ID_imm1 + i_ID_imm2;
					case (i_ID_Oper_sel)
            `OP_BEQ  : o_Branch_En <= (w_op1_data == w_op2_data) ? 1:0;
            `OP_BNE  : o_Branch_En <= (w_op1_data == w_op2_data) ? 0:1;
            `OP_BLT  : o_Branch_En <= ($signed(w_op1_data) < $signed(w_op2_data)) ? 1:0; 
            `OP_BLTU : o_Branch_En <= (w_op1_data < w_op2_data) ? 1:0;
            `OP_BGE  : o_Branch_En <= ($signed(w_op1_data) >= $signed(w_op2_data)) ? 1:0; 
            `OP_BGEU : o_Branch_En <= (w_op1_data >= w_op2_data) ? 1:0;
            default:  o_Event <= 1'b1;
					endcase
				end
				// LOAD ======
				`PORT_LOAD   : begin
					o_rd_Data <= w_op1_data + w_op2_data;
				end
				// STORE =====
				`PORT_STORE  : begin
					o_rd_wr_en <= 1'b1;
					o_rd_Addr  <= w_op1_data + w_op2_data;
					o_rd_Data  <= i_ID_rs2_Data;
				end
				default: o_Event <= 1'b1;
			endcase // END of EXECUTE structure

endmodule