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
wire [`BusWidth ] w_Read_Data;
wire [`BusWidth ] w_Write_Data;
wire [`MemAddr  ] w_Data_Addr;
wire              w_Read_En;
wire              w_Write_En;

// --------------
// Instances ----
// --------------
clk_wiz Clk_gen (
	// System
	.clk_in1(sys_Clk),
	.clk_out1(w_Clk),
	.reset(sys_Rstn)
	);

Core_STR Core (
	// System
	.i_Clk(w_Clk),												
	.i_Rstn(sys_Rstn),											

	// IBUS	
	.o_iMem_En(w_Instr_ReadEn),	
	.o_iMem_Addr(w_Instr_Addr),
	.i_iMem_Data(w_Instr_Data),	

	// DBUS
	.i_dMEM_Read_Data(w_Read_Data),	
	.o_dMEM_Addr(w_Data_Addr),	
	.o_dMEM_ReadEn(w_Read_En),	
	.o_dMEM_WriteEn(w_Write_En),
	.o_dMEM_Write_Data(w_Write_Data),

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
	.o_ReadData(w_Instr_Data)
	);

DataMem #(.BYTESIZE(1024)) dMem (
	// System
	.i_Clk(w_Clk),

	// Data
	.i_WriteEn(w_Write_En),
	.i_Write_Addr(w_Data_Addr),
	.i_Write_Data(w_Write_Data),
	.i_ReadEn(w_Read_En),	
	.i_Read_Addr(w_Data_Addr),
	.o_Read_Data(w_Read_Data)
	);

endmodule