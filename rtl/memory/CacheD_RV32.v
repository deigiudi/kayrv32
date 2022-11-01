/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Data Cache Unit
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   
 *
 */

`timescale 1ns / 1ps
 
module CacheD_RV32 #(
	parameter MEMSIZE = 8
	)(	
	output reg oStallD,				// DCache hasn't got data needed
	output reg [31:0] oMEMDATA,	// MEMORY DATA OUT
	input [31:0] iMEMDATA,			// MEMORY DATA IN		
	input [31:0] iMEMADDR,			// MEMORY ADDR IN
	input iMEM,							// There is a memory transaction to be performed
	input iRW,							// 1 = Read, 0 = Write
	input iCLK
	);
	
	reg [31:0] dCache[MEMSIZE-1:0]; 	// X words of 32-bit cache
	integer i;

	/* Start configuration */
	initial begin
		oStallD	<= 1'b0;
		oMEMDATA <= 32'd0;
		for (i = 0; i < MEMSIZE; i = i + 1) begin
			dCache[i] = 32'd0;
			end
	end 
	
	/* dCache logic */
	always @(posedge iCLK)
	begin
		if (iMEM) begin
			case (iRW)
				1'b0 :	// It's a WRITE
					dCache[iMEMADDR] <= iMEMDATA;
				1'b1 :	// It's a READ
					oMEMDATA <= dCache[iMEMADDR];
				endcase	
		end
	end	
	
endmodule
