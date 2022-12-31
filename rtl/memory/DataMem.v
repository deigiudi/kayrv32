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
-- Function: A file containing the Data Memory                                 -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
 
module DataMem #(
	parameter BYTESIZE=1024
	)(	
	// System
	input wire 			i_Clk,

	// Data
	input wire 			i_ReadEn,
	input wire 			i_WriteEn,
	input wire [31:0] i_AddrWrite,
	input wire [31:0] i_DataWrite,
	input wire [31:0] i_AddrRead,	
	output reg [31:0] o_AddrRead,
	output reg [31:0] o_DataRead
	);
	
	reg [31:0] r_DataMem[0:BYTESIZE-1]; 	// X words of 32-bit

	/* Data memory logic */
	always @(posedge i_Clk)
	begin
		// Read
		if (i_ReadEn) begin
			o_AddrRead <= i_AddrRead;
			o_DataRead <= r_DataMem[i_AddrRead];
		end else begin
			o_DataRead <= 32'd0;
      end
      
		// Write
		if (i_WriteEn) begin
			r_DataMem[i_AddrWrite] <= i_DataWrite;
		end	
	end	
	
endmodule