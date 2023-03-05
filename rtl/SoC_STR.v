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
-- Function    : TOP Structural Architecture
-- Description : A file containing the TOP structure

--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps

module SoC_STR (
	input	 sys_Clk,
	input	 sys_Rstn,
	output sys_Stall,
	output sys_Interrupt
);

// ClockWizard <=> ALL
wire w_Clk;

// InstrMem <=> Core
wire              w_Instr_ReadEn;
wire [`MemAddr  ] w_Instr_Addr;
wire [`BusWidth ] w_Instr_Data;

// DataMem <=> Core
wire              w_Data_ReadEn;
wire              w_Data_WriteEn;
wire [`BusWidth ] w_Data_Read;
wire [`BusWidth ] w_Data_Write;
wire [`MemAddr  ] w_Data_Addr;

// --------------
// Instances ----
// --------------
clk_wiz Clk_gen (
	// System
	.clk_in1(sys_Clk),
	.clk_out1(w_Clk),
	.reset(sys_Rst)
	);

Core_STR Core (
	// System
	.i_Clk(w_Clk),												
	.i_Rstn(sys_Rst),											

	// IBUS	
	.o_iMem_En(w_Instr_ReadEn),	
	.o_iMem_Addr(w_Instr_Addr),
	.i_iMem_Data(w_Instr_Data),	

	// DBUS
	.i_dMEM_Data_read(w_Data_Read),	
	.o_dMEM_Data_write(w_Data_Write),
	.o_dMEM_Addr(w_Data_Addr),
	.o_dMEM_ReadEn(w_Data_ReadEn),
	.o_dMEM_WriteEn(w_Data_WriteEn),	

	// CPU Signals
	.o_Interrupt(sys_Interrupt),
	.o_Stall(sys_Stall)
	);

InstrMem #(.BYTESIZE(1024),
			     .ISTRUCTIONS("ROM.mem")) iMem (
	// System
	.i_Clk(w_Clk),

	// Data
	.i_ReadEn(w_Instr_ReadEn),
	.i_ReadAddr(w_Instr_Addr),
	.i_ReadData(w_Instr_Data)
	);

DataMem #(.BYTESIZE(1024)) dMem (
	// System
	.p_Clk(w_Clk),

	// Data
	.p_ReadEn_In(w_Data_ReadEn),
	.p_WriteEn_In(w_Data_WriteEn),
	.p_Addr_In(w_Data_Addr),
	.p_DataWrite_In(w_Data_Write),
	.p_DataRead_Out(w_Data_Read)
	);

endmodule