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
-- Function: A file containing the Instruction Memory                          -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps 
 
module InstrMem #(
	parameter BYTESIZE = 1024,
   parameter ISTRUCTIONS = ""
	)(		
	// System
	input wire i_Clk,

	// Data
	input wire i_ReadEn,
	input wire [31:0] i_ReadAddr,
	output reg [31:0] o_ReadData
	);
	
	reg  [31:0] r_IstrMem[BYTESIZE-1:0]; // X words of 32-bit
	
	/* Start configuration */
	initial begin
	    $readmemh(ISTRUCTIONS, r_IstrMem);
	end
	
	/* Instruction Memory logic */
	always @(posedge i_Clk)
	begin
		if (i_ReadEn)
			o_ReadData <= r_IstrMem[i_ReadAddr[11:2]];
		else
			o_ReadData <= 32'd0;
	end
endmodule