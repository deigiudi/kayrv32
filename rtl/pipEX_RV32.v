/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Execute Stage Unit
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

module pipEX_RV32 (
	output reg oRW,						// 1 = Read, 0 = Write
	output reg oMEM,						// There is a memory transaction to be performed
	output reg oBRANCH,					// There is a Branch to be taken
	output reg [31:2] oBRANCHADDR,	// Target Address for Program Counter	
	output reg [31:0] oMEMDATA,		// DATA OUT Memory
	output reg [31:0] oMEMADDR,		// ADDR OUT Memory		
	output reg [31:0] oDregDATA,		// DATA OUT Destination Register
	output reg [4:0]  oDregADDR,		// ADDR OUT Destination Register
	output reg [4:0]  oDecodedOP,		// One-Hot  Encoding Type Group to MEM
	output reg [1:0]  oINVALID,		// 10 = istruction not aligned, 01 = invalid instruction
	input [31:0] iAregDATA,				// DATA IN A register
	input [31:0] iBregDATA,				// DATA IN B register
	input [31:0] iIMMDATA,				// DATA IN Immediate
	input [31:0] iPCADDR,				// ADDR IN PC
	input [4:0]  iDregADDR,				// ADDR IN D register
	input [5:0]  iOpType,				// One-Hot Encoding Operation Group
	input [9:0]  iDecodedOP,			// One-Hot Encoding Type Group from FETCH
	input iINSTRAligned,					// Instruction is aligned?
	input iCLK,
	input iRST
	);
	
	
	always @(posedge iCLK)
	begin
		if (iRST) begin
				oMEM <= 1'b0;
				oBRANCH <= 1'b0;
			end
		else begin
			oDecodedOP <= iDecodedOP[4:0];
			oINVALID[0] <= (iINSTRAligned)? 1'b1 : 1'b0;
			case (iOpType)
				6'b100000:	// Load Instruction Group
					begin
						oRW <= 1'b1;
						oMEM <= 1'b1;					
						oBRANCH <= 1'b0;
						oMEMADDR <= iAregDATA + iIMMDATA;
					end
						
				6'b010000:	// Store Instruction Group
					begin
						oRW <= 1'b0;					
						oMEM <= 1'b1;					
						oBRANCH <= 1'b0;
						oMEMADDR <= iAregDATA + iIMMDATA;
						case (iDecodedOP)
							`SB	 : oMEMDATA <= { {24{iBregDATA[7]}}, iBregDATA[7:0] };
							`SH	 : oMEMDATA <= { {16{iBregDATA[15]}}, iBregDATA[15:0] };
							`SW	 : oMEMDATA <= iBregDATA;
							default: oDregDATA <= 32'dX;
							endcase
					end							

				6'b001000:	// Immediate Instruction Group
					begin
						oMEM <= 1'b0;					
						oBRANCH <= 1'b0;
						case (iDecodedOP)				
							`ADDI	 :	oDregDATA <= iAregDATA + iIMMDATA;
							`SLLI	 : if (iIMMDATA[5] == 1'b0) begin
											oINVALID[1] <= 1'b0;											
											oDregDATA <= iAregDATA << iIMMDATA; 
										end else begin
											oINVALID[1] <= 1'b1;
											oDregDATA <= 32'dX;
										end
							`SLTI	 : oDregDATA <= ($signed(iAregDATA) < $signed(iIMMDATA))? 32'd1 : 32'd0;
							`SLTIU :	oDregDATA <= (iAregDATA < iIMMDATA)? 32'd1 : 32'd0;
							`XORI	 :	oDregDATA <= iAregDATA ^ iIMMDATA;							
							`SRLI	 :  if (iIMMDATA[5] == 1'b0) begin
											oINVALID[1] <= 1'b0;											
											oDregDATA <= iAregDATA >> iIMMDATA; 
										end else begin
											oINVALID[1] <= 1'b1;
											oDregDATA <= 32'dX;
										end	 
							`SRAI	 : if (iIMMDATA[5] == 1'b0) begin
											oINVALID[1] <= 1'b0;											
											oDregDATA <= iAregDATA >>> iIMMDATA; 
										end else begin
											oINVALID[1] <= 1'b1;
											oDregDATA <= 32'dX;
										end
							`ORI	 : oDregDATA <= iAregDATA | iIMMDATA;
							`ANDI	 :	oDregDATA <= iAregDATA & iIMMDATA;
							default: oDregDATA <= 32'dX;
							endcase													
					end

				6'b000100:	// Register Instruction Group
					begin
						oMEM <= 1'b0;
						oBRANCH <= 1'b0;
						case (iDecodedOP)								
							`ADD	 : oDregDATA <= iAregDATA + iBregDATA;
							`SUB	 : oDregDATA <= iAregDATA - iBregDATA;
							`SLL	 : oDregDATA <= iAregDATA << iBregDATA;
							`SLT	 : oDregDATA <= ($signed(iAregDATA) < $signed(iBregDATA))? 32'd1 : 32'd0;
							`SLTU	 : oDregDATA <= (iAregDATA < iBregDATA)? 32'd1 : 32'd0;
							`XOR	 : oDregDATA <= iAregDATA ^ iBregDATA;
							`SRL	 : oDregDATA <= iAregDATA >> iBregDATA;
							`SRA	 :	oDregDATA <= iAregDATA >>> iBregDATA;
							`OR	 : oDregDATA <= iAregDATA | iBregDATA;
							`AND	 : oDregDATA <= iAregDATA & iBregDATA;
							default: oDregDATA <= 32'dX;
							endcase
					end							

				6'b000010:	// Branch Instruction Group
					begin
						oMEM <= 1'b0;
						case (iDecodedOP)				
							`BEQ	 : case (iAregDATA == iBregDATA) 
											1'b1: begin 
														oBRANCH <= 1'b1;
														oBRANCHADDR <= iPCADDR + iIMMDATA;
													end
											1'b0: begin 
														oBRANCH <= 1'b0;
														oBRANCHADDR <= 32'd0;
													end											
										endcase
							`BNE	 : case (iAregDATA != iBregDATA) 
											1'b1: begin 
														oBRANCH <= 1'b1;
														oBRANCHADDR <= iPCADDR + iIMMDATA;
													end
											1'b0: begin 
														oBRANCH <= 1'b0;
														oBRANCHADDR <= 32'd0;
													end											
										endcase
							`BLT	 : case ($signed(iAregDATA) < $signed(iBregDATA)) 
											1'b1: begin 
														oBRANCH <= 1'b1;
														oBRANCHADDR <= iPCADDR + iIMMDATA;
													end
											1'b0: begin 
														oBRANCH <= 1'b0;
														oBRANCHADDR <= 32'd0;
													end											
										endcase
							`BGE	 : case ($signed(iAregDATA) > $signed(iBregDATA)) 
											1'b1: begin 
														oBRANCH <= 1'b1;
														oBRANCHADDR <= iPCADDR + iIMMDATA;
													end
											1'b0: begin 
														oBRANCH <= 1'b0;
														oBRANCHADDR <= 32'd0;
													end											
										endcase
							`BLTU  : case (iAregDATA < iBregDATA) 
											1'b1: begin 
														oBRANCH <= 1'b1;
														oBRANCHADDR <= iPCADDR + iIMMDATA;
													end
											1'b0: begin 
														oBRANCH <= 1'b0;
														oBRANCHADDR <= 32'd0;
													end											
										endcase
							`BGEU	 : case (iAregDATA < iBregDATA) 
											1'b1: begin 
														oBRANCH <= 1'b1;
														oBRANCHADDR <= iPCADDR + iIMMDATA;
													end
											1'b0: begin 
														oBRANCH <= 1'b0;
														oBRANCHADDR <= 32'd0;
													end											
										endcase
							default: oDregDATA <= 32'dX;
							endcase
					end

				6'b000001:	// Mixed Instruction Group
					begin
						oMEM <= 1'b0;
						case (iDecodedOP)				
							`AUIPC : begin					
											oBRANCH <= 1'b1;
											oDregDATA <= iPCADDR + iIMMDATA;
										end
							`LUI	 : begin					
											oBRANCH <= 1'b0;
											oDregDATA <= iIMMDATA << 12;
										end
							`JAL	 : begin					
											oBRANCH <= 1'b1;
											oDregDATA <= iAregDATA + 4;
											oBRANCHADDR <= iPCADDR + iIMMDATA;
										end
							`JALR	 : begin
											oBRANCH <= 1'b1;												
											oDregDATA <= iAregDATA +4;
											oBRANCHADDR <= (iPCADDR + iIMMDATA)&~32'b1;
										end
							default: oDregDATA <= 32'dX;
							endcase
					end

				default:
					begin
						oBRANCH <= 1'b0;
						oDregDATA <= 32'dX;
					end
				endcase
		end
	end

endmodule
