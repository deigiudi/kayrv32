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
-- Function: KayRV32 Memory stage	 			                   					 -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "riscv_opcodes.vh"
`include "kayrv32_defines.vh"
 
module Core_pipMA (	
	input  iCLK,
	input  iRST,	
	output reg [4:0]  oDregADDR,	// ADDR OUT Destination Register	
	output reg [31:0] oDregDATA,	// DATA OUT Destination Register
	inout  [31:0] ioMEMDATA,		// DATA OUT DCache
	input  [31:0] iDregDATA,		// DATA IN Destination Register
	input  [31:0] iDCacheDATA,		//	DATA IN DCache
	input  [4:0]  iDecodedOP,		// Operation to be performed	
	input  [4:0]  iDregADDR,		// ADDR IN Destination Register	
	input  iStallD,					// DCache hasn't got data needed 
	input  iRW							// 1 = Read, 0 = Write
	);
	
	reg [31:0] MEMDATA;	
		
	always @(posedge iCLK)
   begin
		if (iRST) begin
			oDregADDR <= 5'd0;
			oDregDATA <= 32'd0;
			end
		else begin
			oDregADDR <= iDregADDR;
			oDregDATA <= iDregDATA;
			case (iRW)
				1'b1 : // It's a READ: something to read from DCache
					if (!iStallD) begin
						case (iDecodedOP)									
							`OP_LB  : oDregDATA <= {{24{iDregDATA[7]}}, iDregDATA[7:0]};
							`OP_LH  : oDregDATA <= {{16{iDregDATA[7]}}, iDregDATA[15:0]};
							`OP_LBU : oDregDATA <= {{24{1'b0}}, iDregDATA[7:0]};
							`OP_LHU : oDregDATA <= {{16{1'b0}}, iDregDATA[15:0]};
							`OP_LW  : oDregDATA <= iDregDATA;
							default : oDregDATA <= 32'dX;
						endcase
					end else begin	// Cache hasn't got DATA requested. We have to wait!
						oDregDATA <= iDregDATA;
					end
				1'b0 : // IT's a WRITE: something to write from DCache
					case (iDecodedOP)
						`OP_SB : MEMDATA <= {{24{iDCacheDATA[7]}}, iDCacheDATA[7:0] };
						`OP_SH : MEMDATA <= {{16{iDCacheDATA[15]}}, iDCacheDATA[15:0] };
						`OP_SW : MEMDATA <= iDCacheDATA;
						default: MEMDATA <= 32'dX;
					endcase								
				endcase
			end
		end
	
	assign ioMEMDATA = MEMDATA;
	
endmodule