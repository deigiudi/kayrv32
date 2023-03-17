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
-- Function    : Memory Read and Write stage
-- Description : Manages communication with data memory. Data memory should be 
								 dual-port RAM for things to work.
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"

 
module Core_pipMA (
  // System
  input wire             i_Clk,
  input wire             i_Rstn,
  // Input
  input wire [`OperSel ] i_EX_Oper_sel,     // operation selector
  input wire [`BusWidth] i_EX_rs2_data,     // needed by STORE instruction
  input wire             i_EX_rd_wr_en,     // destination address write enabler
  input wire [`RegFAddr] i_EX_rd_Addr,      // register destination address
  input wire [`BusWidth] i_EX_rd_Data,      // used for memory write/read address  
  input wire             i_EX_mem_wr_en,    // enables data mem write
  input wire             i_EX_memtoreg,     // passed through, enables WB to regfile  
  input wire             i_CT_Forward_opM,  // forward write data from MEM/WB  
  // Data Memory
  input wire [`BusWidth] i_dMEM_Data_read,  // Data Memory read data
  output reg [`BusWidth] o_dMEM_Data_write, // Data Memory write data
  output reg [`MemAddr ] o_dMEM_Addr,       // Data Memory address
	output reg             o_dMEM_ReadEn, 		// byte read enable
	output reg             o_dMEM_WriteEn,		// byte read enable
  // Output
  output reg             o_MA_rd_rw_en,     // needed for forwarding/stall logic
  output reg [`RegFAddr] o_MA_rd_addr,      // passthrough of destination address
  output reg             o_MA_memtoreg,    
  output reg [`BusWidth] o_MA_dataout,      // data mem read data
  output reg [`BusWidth] o_MA_aluout,       // passthrough of ALU result
  // Event
	output reg             o_Event            // Misaligned Store
  );

  // registers
  reg [`BusWidth] r_data;
  reg [`OperSel ] r_EX_Oper_sel;			// registered because of loads
  reg [ 1:0     ] r_EX_alu_result;
  reg             r_misaligned_store;

  always@(posedge i_Clk) 
  begin
    if(i_Rstn==1'b0) begin
      r_EX_Oper_sel   <= 0;
      r_EX_alu_result <= 0;    	
      o_MA_rd_rw_en   <= 0;
      o_MA_rd_addr    <= 0;
      o_MA_memtoreg   <= 0;      
      o_MA_aluout     <= 0;
      o_Event         <= 0;
    end else begin
      // Signals passtrough
      o_MA_rd_rw_en   <= i_EX_rd_wr_en;
      o_MA_rd_addr    <= i_EX_rd_Addr;      
      o_MA_memtoreg   <= i_EX_memtoreg;
      o_MA_aluout     <= i_EX_rd_Data;

			// =======================================================================
	    // STORE source input: opM                                              ==
	    // =======================================================================
      if(i_CT_Forward_opM)
          r_data = o_MA_aluout;
      else 
          r_data = i_EX_rs2_data;

			// =======================================================================
			// Handle stores                                                        ==
			// =======================================================================
	    o_dMEM_Data_write  <= (i_EX_mem_wr_en) ? 1 : 0;
	    o_DMEM_Addr        <= {i_EX_rd_Data[31:2], 2'b0};
			// Default values, might be overwritten	    
		  o_DMEM_Data_write   = 32'b0; 
		  r_misaligned_store  = 0;
		  case(i_EX_Oper_sel)
		    `OP_SW: o_DMEM_Data_write <= r_data;
		    `OP_SB: begin
		      case(i_EX_rd_Data[1:0]) 
		        2'b00: o_DMEM_Data_write <= { {24{1'b0}}, r_data[7:0] };
		        2'b01: o_DMEM_Data_write <= { {16{1'b0}}, r_data[7:0], { 8{1'b0}} };
		        2'b10: o_DMEM_Data_write <= { { 8{1'b0}}, r_data[7:0], {16{1'b0}} };
		        2'b11: o_DMEM_Data_write <= { r_data[7:0], {24{1'b0}} };
		      endcase
		    end 
		    `OP_SH: begin
		      case(i_EX_rd_Data[1:0])
		        2'b00: o_DMEM_Data_write <= { {16{1'b0}}, r_data[15:0] };
		        2'b10: o_DMEM_Data_write <= { r_data[15:0], {16{1'b0}} };
		        default: r_misaligned_store <= 1;
		      endcase 
		    end   
		  endcase      
      o_Event <= r_misaligned_store;

			// =======================================================================
			// Handle loads                                                        ==
			// =======================================================================
		  // because loads have one clock cycle of latency, works with registers
      r_EX_Oper_sel   <= i_EX_Oper_sel; 
      r_EX_alu_result <= i_EX_rd_Data[1:0];
	    case(r_EX_Oper_sel)
	      `OP_LB: begin
	        case(r_EX_alu_result)
	          2'b00: o_MA_dataout = {{24{i_DMEM_Data_read[ 7]}}, i_DMEM_Data_read[ 7: 0]}; 
	          2'b01: o_MA_dataout = {{24{i_DMEM_Data_read[15]}}, i_DMEM_Data_read[15: 8]}; 
	          2'b10: o_MA_dataout = {{24{i_DMEM_Data_read[23]}}, i_DMEM_Data_read[23:16]}; 
	          2'b11: o_MA_dataout = {{24{i_DMEM_Data_read[31]}}, i_DMEM_Data_read[31:24]}; 
	        endcase
	      end
	      `OP_LH: begin
	        case(r_EX_alu_result[1])
	          0: o_MA_dataout = {{16{i_DMEM_Data_read[15]}}, i_DMEM_Data_read[15: 0]};
	          1: o_MA_dataout = {{16{i_DMEM_Data_read[31]}}, i_DMEM_Data_read[31:16]};
	        endcase
	      end
	      `OP_LBU: begin
	        case(r_EX_alu_result)
	          2'b00: o_MA_dataout = {{24{1'b0}}, i_DMEM_Data_read[ 7: 0]}; 
	          2'b01: o_MA_dataout = {{24{1'b0}}, i_DMEM_Data_read[15: 8]}; 
	          2'b10: o_MA_dataout = {{24{1'b0}}, i_DMEM_Data_read[23:16]}; 
	          2'b11: o_MA_dataout = {{24{1'b0}}, i_DMEM_Data_read[31:24]}; 
	        endcase
	      end
	      `OP_LHU: begin
	        case(r_EX_alu_result[1])
	          0: o_MA_dataout = {{16{1'b0}}, i_DMEM_Data_read[15: 0]};
	          1: o_MA_dataout = {{16{1'b0}}, i_DMEM_Data_read[31:16]};
	        endcase
	      end
	      `OP_LW:  o_MA_dataout = i_DMEM_Data_read;
	      default: o_MA_dataout = i_DMEM_Data_read;
	    endcase
    end
  end
    
endmodule