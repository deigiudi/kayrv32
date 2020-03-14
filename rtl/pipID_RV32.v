/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Istruction Decode Stage Unit
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

`include "DecodedOP.vh"

module pipID_RV32 (
	output reg [31:0] oAregDATA,	// DATA OUT A Register
	output reg [31:0] oBregDATA,	// DATA OUT B Register
	output reg [31:0] oIMMDATA,	// DATA OUT Immediate
	output reg [31:0] oPCADDR,	// ADDR OUT PC
	output reg [5:0]  oDecodedOP,	// Mixed Encoding Operation Type
	output reg [5:0]  oOpType,		// One-Hot Encoding Operation Group
	output reg [4:0]  oDregADDR,	// ADDR D Register		
	output reg oINSTRAligned,			// Instruction is aligned?
	input [31:0] iPCADDR,			// PC Address
	input [31:0] iCacheDATA,		// Instruction from ICache
	input [31:0] iDregDATA,			// DATA D Register
	input [4:0]  iDregADDR,			// ADDR D Register	
	input iCLK,
	input iRST
	);	

	wire [31:0] idu_DATA = iCacheDATA[31:0];
	wire [6:0] idu_f7 = idu_DATA[31:25];
	wire [2:0] idu_f3 = idu_DATA[14:12];
	wire [6:2] idu_op = idu_DATA[6:2];
	
	reg [31:0] idu_RegBANK [31:0];	// RISC-V RV32I 32 bit register bank
	
	/* Hazard detection logic */
	// wire idu_hazard = (idu_rd != 5'd0) && (idu_ra == idu_rd || idu_rb == idu_rd);
	// wire exu_hazard = (exu_rd != 5'd0) && (idu_ra == exu_rd || idu_rb == exu_rd);
	// wire hazard     = idu_hazard || exu_hazard;

	/* Output Updater */
	always @(posedge iCLK)
	begin
      oPCADDR <= iPCADDR;	
		oINSTRAligned <= (idu_DATA[1:0] == 2'b11)? 1'b1 : 1'b0;
	end
	
	/* Destination DATA Updater */
	always @(posedge iCLK)
	begin
      idu_RegBANK[0] <= 1'b0;	
		idu_RegBANK[iDregADDR] <= iDregDATA;
	end
	
	/* Decoder cases structure */
	always @(posedge iCLK)
	begin
		if (iRST) begin	
			oDecodedOP <= 6'd0;
			oOpType	  <= 6'd0;
			end
		else begin
			case (idu_op)		
				5'b00000 :	// Load Instruction Group
					begin
						oOpType <= 6'b100000;
						oAregDATA <= idu_RegBANK[idu_DATA[19:15]];
						oBregDATA <= idu_RegBANK[0];
						oIMMDATA <= { {20{idu_DATA[31]}}, idu_DATA[31:20] };
						oDregADDR <= idu_DATA[11:7];
						case (idu_f3)
							3'b000 : oDecodedOP <= `LB;
							3'b001 : oDecodedOP <= `LH;
							3'b010 : oDecodedOP <= `LW;
							3'b100 : oDecodedOP <= `LBU;
							3'b101 : oDecodedOP <= `LHU;
							default: oDecodedOP <= `NOTSUP;
							endcase
					end
					
				5'b01000 :	// Store Instruction Group
					begin				
						oOpType <= 6'b010000;
						oAregDATA <= idu_RegBANK[idu_DATA[19:15]];
						oBregDATA <= idu_RegBANK[idu_DATA[24:20]];
						oIMMDATA <= { {20{idu_DATA[31]}}, idu_DATA[31:25], idu_DATA[11:7] };
						oDregADDR <= 6'b000000;
						case (idu_f3)					
							3'b000  : oDecodedOP <= `SB;
							3'b001  : oDecodedOP <= `SH;
							3'b010  : oDecodedOP <= `SW;
							default : oDecodedOP <= `NOTSUP;
							endcase
					end					
									
				5'b00100 :	// ALU Immediate Instruction Group
					begin
						oOpType <= 6'b001000;
						oAregDATA <= idu_RegBANK[idu_DATA[19:15]];
						oBregDATA <= idu_RegBANK[0];
						oIMMDATA <= { {20{idu_DATA[31]}}, idu_DATA[31:20] };									
						oDregADDR <= idu_DATA[11:7];
						case (idu_f3)
							3'b000 : oDecodedOP <= `ADDI;
							3'b001 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`SLLI;
									default	  : oDecodedOP <= `NOTSUP;
									endcase				
							3'b010 : oDecodedOP <= `SLTI;
							3'b011 : oDecodedOP <= `SLTIU;					
							3'b100 : oDecodedOP <= `XORI;
							3'b101 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`SRLI;
									7'b0100000 : oDecodedOP <=	`SRAI;
									default	  : oDecodedOP <= `NOTSUP;
									endcase							
							3'b110 : oDecodedOP <= `ORI;
							3'b111 : oDecodedOP <= `ANDI;
							endcase
					end
					
				5'b01100 :	// ALU Register Instruction Group
					begin
						oOpType <= 6'b000100;
						oAregDATA <= idu_RegBANK[idu_DATA[19:15]];
						oBregDATA <= idu_RegBANK[idu_DATA[24:20]];
						oIMMDATA <= idu_RegBANK[0];
						oDregADDR <= idu_DATA[11:7];
						case (idu_f3)
							3'b000 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`ADD;
									7'b0100000 : oDecodedOP <=	`SUB;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							3'b001 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`SLL;
									default	  : oDecodedOP <= `NOTSUP;
									endcase					
							3'b010 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`SLT;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							3'b011 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`SLTU;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							3'b100 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`XOR;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							3'b101 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`SRL;
									7'b0100000 : oDecodedOP <=	`SRA;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							3'b110 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`OR;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							3'b111 : 
								case (idu_f7)
									7'b0000000 : oDecodedOP <=	`AND;
									default	  : oDecodedOP <= `NOTSUP;
									endcase
							endcase
					end					
					
				5'b11000 :	// Branch Instruction Group
					begin
						oOpType <= 6'b000010;
						oAregDATA <= idu_RegBANK[idu_DATA[19:15]];
						oBregDATA <= idu_RegBANK[idu_DATA[24:20]];
						oIMMDATA <= { {20{idu_DATA[31]}}, idu_DATA[7], idu_DATA[30:25], idu_DATA[11:8], 1'b0 };
						oDregADDR <= 6'b000000;
						case (idu_f3)
							3'b000 : oDecodedOP <= `BEQ;
							3'b001 : oDecodedOP <= `BNE;
							3'b100 : oDecodedOP <= `BLT;
							3'b101 : oDecodedOP <= `BGE;
							3'b110 : oDecodedOP <= `BLTU;
							3'b111 : oDecodedOP <= `BGEU;
							default: oDecodedOP <= `NOTSUP;
							endcase
					end					
										
				5'b00101 :	// ALU Immediate Instruction
					begin
						oOpType <= 6'b000001;
						oAregDATA <= iPCADDR;
						oBregDATA <= idu_RegBANK[0];
						oIMMDATA <= { idu_DATA[31:12], 12'h0 };
						oDregADDR <= idu_DATA[11:7];
						oDecodedOP <= `AUIPC;
					end
										
				5'b01101 :	// ALU Immediate Instruction
					begin
						oOpType <= 6'b000001;
						oAregDATA <= idu_RegBANK[0];
						oBregDATA <= idu_RegBANK[0];
						oIMMDATA <= { idu_DATA[31:12], 12'h0 };
						oDregADDR <= idu_DATA[11:7];
						oDecodedOP <= `LUI;
					end
										
				5'b11011 :	// Jump Instruction
					begin
						oOpType <= 6'b000001;
						oAregDATA <= idu_RegBANK[0];
						oBregDATA <= idu_RegBANK[0];
						oIMMDATA <= { {12{idu_DATA[31]}}, idu_DATA[19:12], idu_DATA[20], idu_DATA[30:21], 1'b0 };
						oDregADDR <= idu_DATA[11:7];
						oDecodedOP <= `JAL;
					end

				5'b11001 :	// Jump Instruction
					begin
						oOpType <= 6'b000001;
						oAregDATA <= idu_RegBANK[0];
						oBregDATA <= idu_RegBANK[0];
						oIMMDATA <= { {20{idu_DATA[31]}}, idu_DATA[31:20] };
						oDregADDR <= idu_DATA[11:7];						
						case (idu_f3)
							3'b000  : oDecodedOP <=	`JALR;					
							default : oDecodedOP <= `NOTSUP;
							endcase
					end	

				default : 
					begin
						oOpType <= 6'd0;	
						oDecodedOP <= `NOTSUP;
					end
				
			endcase		
		end	
	end
endmodule
