/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Istruction Cache Unit
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
 
module CacheI_RV32 #(
	parameter  MEMSIZE = 8
	)(		
	output reg oStallI,			// iCache hasn't got data needed
	inout [31:0] ioINSTDATA,	// DATA OUT Destination Register	
	input [31:0] iINSTADDR,		// ADDR IN Destination Register
	input iCLK
	);
	
	reg [31:0] iCache[0:MEMSIZE-1]; 	// X words of 32-bit cache
	reg [31:0] INSTDATA;	
	integer i;
	
	assign ioINSTDATA = INSTDATA;
	
	/* Start configuration */
	initial begin
		oStallI	<= 1'b0;
		INSTDATA <= 32'd0;
		for (i = 0; i < MEMSIZE; i = i + 1) begin
			iCache[i] = 32'd0;
			end
	end
	
	/* ICache logic */
	always @(posedge iCLK)
	begin
		INSTDATA <= iCache[iINSTADDR];
	end
endmodule
