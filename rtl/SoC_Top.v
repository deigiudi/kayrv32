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
-- Function: A file containing the TOP Architecture                            -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps

module SoC_Top (
	input	 sys_Clk,
	input	 sys_Rstn,
	output sys_Stall,
	output sys_Interrupt
);

// ClockWizard <=> ALL
wire p_Clk;

// InstrMem <=> Core
wire w_InstReadEn;
wire [31:0] w_Instr_Addr;
wire [31:0] w_Instr_Data;

// DataMem <=> Core
wire w_DataRead_En;
wire w_DataWrite_En;
wire [31:0] w_Data_Addr;
wire [31:0] w_Read_Data;
wire [31:0] w_Write_Data;

// --------------
// Instances ----
// --------------
clk_wiz Clk_gen (
	// System
	.clk_in1(sys_Clk),
	.clk_out1(sys_Clk),
	.reset(sys_Rst)
	);

Core_Top Core (
	// System
	.i_Clk(sys_Clk),												
	.i_Rstn(sys_Rst),											

	// IBUS
	.i_iMem_Data(w_Instr_Data),	
	.o_iMem_Addr(w_Instr_Addr),	
	.o_iMem_ReadEn(w_InstReadEn),			

	// DBUS
	.i_dMem_DataRead(w_Read_Data),	
	.o_dMem_Addr(w_Data_Addr),
	.o_dMem_DataWrite(w_Write_Data),
	.o_dMem_ReadEn(w_DataRead_En),
	.o_dMem_WriteEn(w_DataWrite_En),	

	// CPU Signals
	.o_Stall(sys_Stall),
	.o_Interrupt(sys_Interrupt)
	);

InstrMem #(.BYTESIZE(1024),
					 .ISTRUCTIONS("ROM.mem")) iMem (
	// System
	.p_Clk(p_Clk),

	// Data
	.p_ReadEn_In(w_InstReadEn),
	.p_AddrRead_In(w_Instr_Addr),
	.p_DataRead_Out(w_Instr_Data)
	);

DataMem #(.BYTESIZE(1024)) dMem (
	// System
	.p_Clk(p_Clk),

	// Data
	.p_ReadEn_In(w_DataRead_En),
	.p_WriteEn_In(w_DataWrite_En),
	.p_Addr_In(w_Data_Addr),
	.p_DataWrite_In(w_Write_Data),
	.p_DataRead_Out(w_Read_Data)
	);

endmodule