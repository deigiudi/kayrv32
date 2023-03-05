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
-- Function    : Core Structural architecture
-- Description : A file containing the Core structure 

--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"

module Core_STR (
	// System
	input wire              i_Clk,             // System Clk
	input wire              i_Rstn,            // System Reset

	// IBUS
	output reg 			        o_iMem_ReadEn,
	output reg [`MemAddr  ] o_iMem_Addr,
	input wire [`BusWidth ] i_iMem_Data,

	// DBUS
	input wire [`BusWidth ] i_dMEM_Data_read,
	output reg [`BusWidth ] o_dMEM_Data_write,	
	output reg [`MemAddr  ] o_dMEM_Addr,
	output reg              o_dMEM_ReadEn,
	output reg              o_dMEM_WriteEn,

	// Events
	output reg              o_Interrupt,        // CPU Interrupt
	output reg              o_Stall             // CPU is stalling
);


// =============================================================================
// Wiring                                                                     ==
// =============================================================================
// From IF
wire [`MemAddr  ] w_IF_PC;
wire [`BusWidth ] w_IF_Instruction;
wire   						w_IF_Event;

// From ID
wire [`MemAddr	] w_ID_PC;
wire [`PortSel  ] w_ID_Port_sel;
wire [`OperSel  ] w_ID_Oper_sel;
wire [`InputSel ] w_ID_Input_sel;
wire [`RegFWidth] w_ID_rs1_Addr;
wire [`RegFWidth] w_ID_rs2_Addr;
wire [`RegFAddr ] w_ID_rd_Addr;
wire [`RegFWidth] w_ID_rs1_Data;
wire [`RegFWidth] w_ID_rs2_Data;
wire [`RegFWidth] w_ID_offset;
wire              w_ID_Event;

// From EX
wire [`OperSel  ] w_EX_Oper_sel;
wire              w_EX_rd_Addr;
wire              w_EX_RW_En;
wire [`RegFAddr ] w_EX_Addr;
wire [`RegFWidth] w_EX_Data;
wire              w_EX_BranchEn;
wire [`PCWidth  ] w_EX_BranchAddr;
wire              w_EX_Event;

// From MA
wire              w_MEM_RW_en;
wire              w_MEM_memtoreg;
wire [`RegFAddr ] w_MEM_rd_addr;
wire [`BusWidth ] w_MEM_dataout;
wire [`BusWidth ] w_MEM_aluout;
wire              w_MA_Event;

// From WB
wire              w_WB_RW_en;
wire [`RegFAddr ] w_WB_Addr;
wire [`RegFWidth] o_WB_Data;

// From Branch Generator
wire [`PCWidth  ] w_BG_BranchAddr;


// From Pipeline Controller
wire              w_PipC_StallEn;
wire              w_PipC_FlushEn_IFID;
wire              w_PipC_FlushEn_EX;
wire [`EventBus ]	w_PipC_EventBus;


// =============================================================================
// Istruction Fetch Stage                                                     ==
// =============================================================================
Core_pipIF pipIF (
	// System
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	// Input
	.i_ID_JumpEn(),
	.i_BG_JumpAddr(w_BG_BranchAddr),		
	.i_EX_BranchEn(w_EX_BranchEn),
	.i_EX_BranchAddr(w_EX_BranchAddr),
	.i_IMEM_InstrData(i_iMem_Data),
	// Output
	.o_InstrEn(o_iMem_ReadEn),
	.o_InstrAddr(o_iMem_Addr),
	.o_InstrData(w_IF_Instruction),
	.o_PC(w_IF_PC),
	// Control IO
	.i_StallEn(w_PipC_StallEn),
	.i_FlushEn(w_PipC_FlushEn_IFID),
	.o_Event(w_IF_Event)
);

// =============================================================================
// Istruction Decode Stage                                                    ==
// =============================================================================
Core_pipID pipID (
	// System	
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	// Input
	.i_IF_PC(w_IF_PC),
	.i_IF_Istr(w_IF_Instruction),
	.i_EX_RW_En(w_EX_RW_En),
	.i_EX_Addr(w_EX_Addr),
	.i_EX_Data(w_EX_Data),
	.i_WB_RW_En(w_WB_RW_en),
	.i_WB_Addr(o_WB_Data),
	.i_WB_Data(o_WB_Data),	
	// Output
	.o_PC(w_ID_PC),
	.o_Port_sel(w_ID_Port_sel),
	.o_Oper_sel(w_ID_Oper_sel),
	.o_Input_sel(w_ID_Input_sel),
	.o_rs1_Addr(w_ID_rs1_Addr),
	.o_rs2_Addr(w_ID_rs2_Addr),
	.o_rd_Addr(w_ID_rd_Addr),
	.o_rs1_Data(w_ID_rs1_Data),
	.o_rs2_Data(w_ID_rs2_Data),
	.o_offset(w_ID_offset),
	// Control IO
	.i_StallEn(w_PipC_StallEn),
	.i_FlushEn(w_PipC_FlushEn_IFID),
	.o_Event(w_ID_Event)
);


// =============================================================================
// EXecution Stage                                                            ==
// =============================================================================
Core_pipEX pipEX (
	// System
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	// Input
	.i_ID_PC(w_ID_PC),
	.i_ID_Port_sel(w_ID_Port_sel),
	.i_ID_Oper_sel(w_ID_Oper_sel),
	.i_ID_Input_sel(w_ID_Input_sel),
	.i_ID_rd_Addr(w_ID_rd_Addr),
	.i_ID_rs1_Data(w_ID_rs1_Data),
	.i_ID_rs2_Data(w_ID_rs2_Data),
	.i_ID_offset(w_ID_offset),
	// Output
	.o_Oper_sel(w_EX_Oper_sel),
	.o_rd_Addr(w_EX_rd_Addr),
	.o_RW_En(w_EX_RW_En),
	.o_WriteAddr(w_EX_Addr),
	.o_WriteData(w_EX_Data),
	.o_BranchEn(w_EX_BranchEn),
	.o_BranchAddr(w_EX_BranchAddr),
	// Control IO
	.i_FlushEn(w_PipC_FlushEn_EX),
	.o_Event(w_EX_Event)
);


// =============================================================================
// Memory Access Stage                                                        ==
// =============================================================================
Core_pipMA pipMA (
	// System
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	// Input
	.i_EX_Oper_sel(w_EX_Oper_sel),
  .i_EX_mem_wr_en(),
  .i_EX_memtoreg(),
  .i_EX_alu_result(),
  .i_EX_rs2_data(),
  .i_EX_rd_wr_en(),
  .i_EX_rd_Addr(),
  .i_CTRL_ForwardM(),
  .i_dMEM_Data_read(i_dMEM_Data_read),
	//Output
  .o_dMEM_Data_write(o_dMEM_Data_write),
  .o_dMEM_Addr(o_dMEM_Addr),
	.o_dMEM_ReadEn(o_dMEM_ReadEn),
	.o_dMEM_WriteEn(o_dMEM_WriteEn),
  .o_MEM_RW_en(w_MEM_RW_en),
  .o_MEM_memtoreg(w_MEM_memtoreg),
  .o_MEM_rd_addr(w_MEM_rd_addr),
  .o_MEM_dataout(w_MEM_dataout),
  .o_MEM_aluout(w_MEM_aluout),
	// Control Output
	.o_Event(w_MA_Event)
);


// =============================================================================
// WriteBack Stage                                                            ==
// =============================================================================
Core_pipWB pipWB (
	//Input
  .i_MEM_RW_en(w_MEM_RW_en),  
  .i_MEM_memtoreg(w_MEM_memtoreg),    
  .i_MEM_rd_addr(w_MEM_rd_addr),
  .i_MEM_dataout(w_MEM_dataout),
  .i_MEM_aluout(w_MEM_aluout),
	//Output
	.o_WriteEn(w_WB_RW_en),
	.o_WriteAddr(w_WB_Addr),
	.o_WriteData(o_WB_Data)
);


// =============================================================================
// Branch Generator                                                           ==
// =============================================================================
Core_pipBranchGen pipBranchGen (
	// System
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	// Inputs
	.i_ID_JAL_OperSel(w_ID_Oper_sel[3:2]),
	.i_ID_rs1_Addr(w_ID_rs1_Addr),
	.i_ID_rs1_Data(w_ID_rs1_Data),
	.i_ID_PC(w_ID_PC),
	.i_ID_offset(w_ID_offset),
	.i_EX_rd_Addr(w_EX_rd_Addr),
	.i_EX_rd_wr_en(),
	.i_EX_alu_result(),
	// Output
	.o_Branch_Addr(w_BG_BranchAddr)	
	);


// =============================================================================
// Pipeline Control                                                           ==
// =============================================================================
assign w_PipC_EventBus = {w_IF_Event, w_ID_Event, w_EX_Event, w_MA_Event};
Core_pipControl pipControl (
	// System
	.i_Clk(i_Clk),
	.i_Rstn(i_Rstn),
	.i_EventBus(w_PipC_EventBus),
	// Forwarding
	.i_ID_Input_sel(),
	.i_ID_rs1_Addr(w_ID_rs1_Addr),
	.i_ID_rs2_Addr(w_ID_rs2_Addr),
	.i_EX_rs2_Addr(),	
	.i_ID_rd_Addr(w_ID_rd_Addr),
	.i_EX_rd_Addr(w_EX_rd_Addr),
	.i_MA_rd_Addr(w_MEM_rd_addr),	
	.i_EX_rd_wr_en(),
	.i_MA_rd_wr_en(w_MEM_RW_en),	
	.o_forwardA(),
	.o_forwardB(),
	.o_forwardM(),
	// Hazard detection
	.i_IF_Instr(w_IF_Instruction),
	.i_ID_Load_en(w_ID_Port_sel[4]),
	.i_ID_Jump_en(),
	.i_EX_branch_en(),
	.o_FlushEn_IFID(w_PipC_FlushEn_IFID),
	.o_FlushEn_EX(w_PipC_FlushEn_EX),
	.o_StallEn(w_PipC_StallEn),
	// Event
	.o_Interrupt(o_Interrupt)	
);

endmodule