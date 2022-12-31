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
-- Function: A file containing the Top Core structure                          -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"

module Core_Top (
	// System
	input wire 			i_Clk,				 // System Clk
	input wire 			i_Rstn,				 // System Reset

	// IBUS
	input wire [31:0] i_iMem_Data,		 // istruction data from memory
	output reg [31:0] o_iMem_Addr,		 // next istruction address
	output reg 			o_iMem_ReadEn,		 // read op enabler

	// DBUS
	input wire [31:0] i_dMem_DataRead,	 // Data Read Bus	
	output reg [31:0] o_dMem_Addr,		 // Address of memory to read or write
	output reg [31:0] o_dMem_DataWrite,  // Data Write Bus
	output reg 			o_dMem_ReadEn,		 // Read Enable
	output reg 			o_dMem_WriteEn,	 // Write Enable	

	// CPU Signals
	input wire 			o_Stall,		 		 // CPU is stalling
	input wire 			o_Interrupt	 	 	 // CPU Interrupt
);

// IF <=> ID
wire [`MemAddr  ] w_toID_PC;

// IF <=> EX
wire              w_JumpEn;
wire [`PCWidth  ] w_JumpAddr;

// ID <=> EX
wire [`MemAddr	 ] w_ID_PC;
wire [`PortSel  ] w_ID_Port_sel;
wire [`OperSel  ] w_ID_Oper_sel;
wire [`InputSel ] w_ID_Input_sel;
wire [`RegFWidth] w_src1;
wire [`RegFWidth] w_src2;
wire [`RegFWidth] w_offset;
wire [`RegFAddr ] w_RegFAddr;
wire              w_ExeEn;
wire [`RegFAddr ] w_ExeAddr;
wire [`RegFWidth] w_ExeData;

// EX <=> MA

// MA <=> WB

// MA <=> IF


Core_pipIF pipIF (
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	.i_StallEn(),
	.i_FlushEn(),
	.i_BranchEn(),
	.i_BranchAddr(),
	.i_JumpEn(w_JumpEn),
	.i_JumpAddr(w_JumpAddr),
	.o_ReadEn(p_iMem_ReadEn_Out),	
	.o_ReadAddr(p_iMem_AddrRead_Out),
	.o_PC(w_toID_PC),	
	.o_Event()
);

Core_pipID pipID (
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	.i_StallEn(),
	.i_FlushEn(),
	.i_PC(w_toID_PC),
	.i_Istr(p_dMem_DataRead_In),
	.i_MemEn(),
	.i_MemAddr(),
	.i_MemData(),
	.i_ExeEn(w_ExeEn),
	.i_ExeAddr(w_ExeAddr),
	.i_ExeData(w_ExeData),
	.o_PC(w_ID_PC),	
	.o_Port_sel(w_ID_Port_sel),
	.o_Oper_sel(w_ID_Oper_sel),
	.o_Input_sel(w_ID_Input_sel),
	.o_RegFAddr(w_RegFAddr),
	.o_src1(w_src1),
	.o_src2(w_src2),
	.o_offset(w_offset),
	.o_StallEn(),
	.o_Exception(),
	.o_Event()	
);

Core_pipEX pipEX (
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),	
	.i_PC(w_ID_PC),	
	.i_Port_sel(w_ID_Port_sel),
	.i_Oper_sel(w_Oper_sel),
	.i_Input_sel(w_ID_Input_sel),
	.i_RegFAddr(w_RegFAddr),
	.i_src1(w_src1),
	.i_src2(w_src2),
	.i_offset(w_offset),
	.o_Oper_sel(w_ID_Oper_sel),
	.o_WriteEn(w_ExeEn),
	.o_WriteAddr(w_ExeAddr),	
	.o_WriteData(w_ExeData),		
	.o_JumpEn(w_ExeAddr),
	.o_JumpAddr(w_ExeData),	
	.o_StallEn(),
	.o_Event()	
);

Core_pipMA pipMA (
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn)	

);

Core_pipWB pipWB (
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),	
	.i_StallEn(),
	.i_WriteEn(),
	.i_WriteAddr(),	
	.i_WriteData(),
	.o_WriteEn(),
	.o_WriteAddr(),	
	.o_WriteData()
);

endmodule
