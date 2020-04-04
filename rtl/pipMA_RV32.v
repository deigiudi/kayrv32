/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Memory Access Stage Unit
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
`include "DecodedOP.vh"
 
module pipMA_RV32 (	
	output reg [4:0]  oDregADDR,	// ADDR OUT Destination Register	
	output reg [31:0] oDregDATA,	// DATA OUT Destination Register
	input  [4:0]  iDecodedOP,		// Operation to be performed	
	input  [4:0]  iDregADDR,		// ADDR IN Destination Register	
	input  [31:0] iDregDATA,		// DATA IN Destination Register
	input  iStallD,					// DCache hasn't got data needed or Bubble in pipeline
	input  iRW,							// 1 = Read, 0 = Write
	input  iMEM,						// There is a memory transaction to be performed
	input  iCLK,
	input  iRST
	);
	
	always @(posedge iCLK)
	begin
		if (iRST) begin
			oDregADDR <= 5'd0;
			oDregDATA <= 32'd0;
			end
		else begin
			oDregADDR <= iDregADDR;
			case (iMEM)
				1'b1 :	/* Memory operation */
					case (iRW)
						1'b1 : 
							if (!iStallD) begin
								case (iDecodedOP)
									`LB	 :	oDregDATA <= 32'dX;
									`LH	 : oDregDATA <= 32'dX;
									`LBU	 : oDregDATA <= 32'dX;
									`LHU	 : oDregDATA <= 32'dX;
									`LW	 : oDregDATA <= 32'dX;
									default: oDregDATA <= 32'dX;
									endcase
							end else begin
								oDregDATA <= 32'dX;
							end						
						1'b0 : oDregDATA <= 32'dX;
						endcase
 				1'b0 :	/* Passthrough logic */
					oDregDATA <= iDregDATA;
				endcase
			end
		end
	
endmodule
