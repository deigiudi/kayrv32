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
`include "kayrv32_RTL_defines.vh"

module Core_STR (
	// System
	input wire               i_Clk,             // System Clk
	input wire               i_Rstn,            // System Reset

	// IBUS
	output wire              o_iMem_En,
	output wire [`MemAddr  ] o_iMem_Addr,
	input  wire [`BusWidth ] i_iMem_Data,

	// DBUS
	input wire  [`BusWidth ] i_dMEM_Read_Data,
	output wire [`MemAddr  ] o_dMEM_Addr,
	output wire              o_dMEM_ReadEn,
	output wire              o_dMEM_WriteEn,
	output wire [`BusWidth ] o_dMEM_Write_Data,

	// Events
	output wire             o_Interrupt,        // CPU Interrupt
	output wire             o_Stall             // CPU is stalling
);


// =============================================================================
// Wiring                                                                     ==
// =============================================================================
// From IF
wire [`MemAddr  ] w_IF_PC;
wire [`BusWidth ] w_IF_Instruction;
wire   						w_IF_Event;

// From ID
wire [`PortSel  ] w_ID_Port_sel;
wire [`OperSel  ] w_ID_Oper_sel;
wire [`InputSel ] w_ID_Input_sel;
wire [`RegFWidth] w_ID_rs1_Addr;
wire [`RegFWidth] w_ID_rs2_Addr;
wire [`RegFAddr ] w_ID_rd_Addr;
wire [`RegFWidth] w_ID_rs1_Data;
wire [`RegFWidth] w_ID_rs2_Data;
wire [`RegFWidth] w_ID_imm1;
wire [`RegFWidth] w_ID_imm2;
wire              w_ID_rd_wr_En;
wire              w_ID_Event;

// From EX
wire [`OperSel  ] w_EX_Oper_sel;
wire [`RegFAddr ] w_EX_rs2_Addr;
wire [`RegFWidth] w_EX_rs2_Data;
wire              w_EX_rd_wr_En;
wire [`RegFAddr ] w_EX_rd_Addr;
wire [`RegFWidth] w_EX_rd_Data;
wire              w_EX_Branch_En;
wire [`PCWidth  ] w_EX_Branch_Addr;
wire              w_EX_Event;

// From MA
wire              w_MA_memtoreg;
wire              w_MA_rd_rw_en;
wire [`RegFAddr ] w_MA_rd_Addr;
wire [`BusWidth ] w_MA_rd_Data;
wire              w_MA_Event;

// From Pipeline Controller
wire              w_CT_StallEn;
wire              w_CT_FlushEn_IFID;
wire              w_CT_FlushEn_EX;
wire [`EventBus ]	w_CT_EventBus;


// =============================================================================
// Istruction Fetch Stage                                                     ==
// =============================================================================
Core_pipIF pipIF (
	// System
	.i_Clk            (i_Clk),
	.i_Rstn           (i_Rstn),
	// Input
	.i_EX_Branch_En   (w_EX_Branch_En),
	.i_EX_Branch_Addr (w_EX_Branch_Addr),
	// Instruction Memory
	.i_IMEM_Data      (i_iMem_Data),
	.o_IMEM_En        (o_iMem_En),
	.o_IMEM_Addr      (o_iMem_Addr),
	// Output	
	.o_PC             (w_IF_PC),	
	.o_Instr          (w_IF_Instruction),
	// Control IO
	.i_StallEn        (w_CT_StallEn),
	.i_FlushEn        (w_CT_FlushEn_IFID),
	.o_Event          (w_IF_Event)
);

// =============================================================================
// Istruction Decode Stage                                                    ==
// =============================================================================
Core_pipID pipID (
	// System	
	.i_Clk         (i_Clk),
	.i_Rstn        (i_Rstn),
	// Input
	.i_IF_PC       (w_IF_PC),
	.i_IF_Instr    (w_IF_Instruction),
	.i_EX_rd_wr_En (w_EX_rd_wr_En),
	.i_EX_rd_Addr  (w_EX_rd_Addr),
	.i_EX_rd_Data  (w_EX_rd_Data),
	.i_WB_wr_En    (w_WB_rd_wr_En),
	.i_WB_rd_Addr  (w_WB_rd_Addr),
	.i_WB_rd_Data  (w_WB_rd_Data),	
	// Output
	.o_Port_sel    (w_ID_Port_sel),
	.o_Oper_sel    (w_ID_Oper_sel),
	.o_Input_sel   (w_ID_Input_sel),
	.o_rs1_Addr    (w_ID_rs1_Addr),
	.o_rs2_Addr    (w_ID_rs2_Addr),
	.o_rd_Addr     (w_ID_rd_Addr),
	.o_rd_wr_En    (w_ID_rd_wr_En),	
	.o_rs1_Data    (w_ID_rs1_Data),
	.o_rs2_Data    (w_ID_rs2_Data),
	.o_imm1        (w_ID_imm1),
	.o_imm2        (w_ID_imm2),
	// Control IO
	.i_StallEn     (w_CT_StallEn),
	.i_FlushEn     (w_CT_FlushEn_IFID),
	.o_Event       (w_ID_Event)
);


// =============================================================================
// EXecution Stage                                                            ==
// =============================================================================
Core_pipEX pipEX (
	// System
	.i_Clk            (i_Clk),
	.i_Rstn           (i_Rstn),
	// Input
	.i_ID_Port_sel    (w_ID_Port_sel),
	.i_ID_Oper_sel    (w_ID_Oper_sel),
	.i_ID_Input_sel   (w_ID_Input_sel),
	.i_ID_rs1_Addr    (w_ID_rs1_Addr),
	.i_ID_rs2_Addr    (w_ID_rs2_Addr),
	.i_ID_rd_Addr     (w_ID_rd_Addr),	
	.i_ID_rd_wr_en    (w_ID_rd_wr_En),
	.i_ID_rs1_Data    (w_ID_rs1_Data),
	.i_ID_rs2_Data    (w_ID_rs2_Data),
	.i_ID_imm1        (w_ID_imm1),
	.i_ID_imm2        (w_ID_imm2),
	// Forwarding
	.i_MA_rd_Data     (w_MA_rd_Data),
  .i_CT_forward_op1 (w_CT_forward_op1),
  .i_CT_forward_op2 (w_CT_forward_op2),
	// Output
	.o_Oper_sel       (w_EX_Oper_sel),
	.o_rs2_Addr       (w_EX_rs2_Addr),
	.o_rs2_Data       (w_EX_rs2_Data),
	.o_rd_wr_En       (w_EX_rd_wr_En),	
	.o_rd_Addr        (w_EX_rd_Addr),
	.o_rd_Data        (w_EX_rd_Data),
	.o_Branch_En      (w_EX_Branch_En),
	.o_Branch_Addr    (w_EX_Branch_Addr),
	// Control IO
	.i_FlushEn        (w_CT_FlushEn_EX),
	.o_Event          (w_EX_Event)
);


// =============================================================================
// Memory Access Stage                                                        ==
// =============================================================================
Core_pipMA pipMA (
	// System
	.i_Clk             (i_Clk),
	.i_Rstn            (i_Rstn),
	// Input
	.i_EX_Oper_sel     (w_EX_Oper_sel),
  .i_EX_rs2_data     (w_EX_rs2_Data),
  .i_EX_rd_wr_en     (w_EX_rd_wr_En),
  .i_EX_rd_Addr      (w_EX_rd_Addr),
  .i_EX_rd_Data      (w_EX_rd_Data),  
  .i_EX_mem_wr_en    (),
  .i_EX_memtoreg     (),
  .i_CT_Forward_opM  (w_CT_forward_opM),
	// Data Memory
  .i_dMEM_Read_Data  (i_dMEM_Read_Data),
  .o_dMEM_ReadEn     (o_dMEM_ReadEn),
  .o_dMEM_Addr       (o_dMEM_Addr),
	.o_dMEM_WriteEn    (o_dMEM_WriteEn),
  .o_dMEM_Write_Data (o_dMEM_Write_Data),	
	// Output
  .o_memtoreg        (w_MA_memtoreg),	
  .o_rd_rw_en        (w_MA_rd_rw_en),
  .o_rd_Addr         (w_MA_rd_Addr),  
  .o_rd_Data         (w_MA_rd_Data),
	// Control Output
	.o_Event           (w_MA_Event)
);


// =============================================================================
// Pipeline Control                                                           ==
// =============================================================================
assign w_CT_EventBus = {w_IF_Event, w_ID_Event, w_EX_Event, w_MA_Event};
Core_pipControl pipControl (
	// System
	.i_Clk          (i_Clk),
	.i_Rstn         (i_Rstn),
	.i_EventBus     (w_CT_EventBus),
	// Forwarding
	.i_ID_Input_sel (w_ID_Input_sel),
	.i_ID_rs1_Addr  (w_ID_rs1_Addr),
	.i_ID_rs2_Addr  (w_ID_rs2_Addr),
	.i_ID_rd_Addr   (w_ID_rd_Addr),
	.i_EX_rs2_Addr  (w_ID_rs2_Data),
	.i_EX_rd_wr_en  (w_EX_rd_wr_En),
	.i_EX_rd_Addr   (w_EX_rd_Addr),
	.i_MA_rd_wr_en  (w_MA_rd_rw_en),	
	.i_MA_rd_Addr   (w_MA_rd_Addr),
	// Hazard detection
	.i_IF_Instr     (w_IF_Instruction),
	.i_ID_Load_en   (w_ID_Port_sel[4]),
	.i_EX_Branch_en (w_EX_Branch_En),
	// Output
	.o_forward_op1  (w_CT_forward_op1),
	.o_forward_op2  (w_CT_forward_op2),
	.o_forward_opM  (w_CT_forward_opM),	
	.o_FlushEn_IFID (w_CT_FlushEn_IFID),
	.o_FlushEn_EX   (w_CT_FlushEn_EX),
	.o_StallEn      (w_CT_StallEn),
	// Event
	.o_Interrupt    (o_Interrupt)	
);

endmodule