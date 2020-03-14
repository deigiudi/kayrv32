/*
 *  SasRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the CPU Architecture
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

module Core_RV32 (
	output oRW,						// 1 = Read, 0 = Write
	output oMEM,					// There is a memory transaction to be performed	
	output [31:0] oCacheADDR,	// Data Address to read from ICache
	output [31:0] oMEMADDR,		// ADDR OUT Memory	
	input  [31:0] iMEMDATA,		// DATA IN Memory
	input  [31:0] iCacheDATA,	// Instruction from ICache	
	input  iStallI,				// ICache hasn't got data needed or Bubble in pipeline	
	input  iStallD,				// DCache hasn't got data needed or Bubble in pipeline		
	input	 iCLK,
	input	 iRST
);

// IF <=> EX
wire [31:0] BranchADDR;
wire ItsBRANCH;

// ID <=> EX
wire [31:0] AregDATA;
wire [31:0] BregDATA;
wire [31:0] IMMDATA;
wire [31:0] PCADDR;
wire  [4:0] DregADDRint1;
wire	[5:0] DecodedOP;
wire  [5:0] OpType;
wire INSTRAligned;

// ID <=> WB
wire [31:0] DregDATA;
wire  [4:0] DregADDR;

// EX <=> MA
wire [31:0] DregDATA1;
wire  [4:0] DregADDRint2;

// MA <= WB
wire [31:0] DregDATA2;
wire  [4:0] DregADDRint3;


pipIF_RV32 pipIF (
	.oPCADDR(oCacheADDR),			// Data Address to read from ICache
	.iBranchADDR(BranchADDR),		// pipEX says in which Address to jump in
	.iBRANCH(ItsBRANCH),				// pipEX says there is a Branch to be taken		
	.iStallI(iStallI),				// ICache hasn't got data needed or Bubble in pipeline
	.iStallD(iStallD),				// DCache hasn't got data needed or Bubble in pipeline	
	.iCLK(iCLK),
	.iRST(iRST)	
);

pipID_RV32 pipID (
	.oAregDATA(AregDATA),			// DATA OUT A Register
	.oBregDATA(BregDATA),			// DATA OUT B Register
	.oIMMDATA(IMMDATA),				// DATA OUT Immediate
	.oPCADDR(PCADDR),					// ADDR OUT PC
	.oDecodedOP(DecodedOP),			// Mixed Encoding Operation Type
	.oOpType(OpType),					// One-Hot Encoding Operation Group
	.oDregADDR(DregADDRint1),		// ADDR D Register		
	.oINSTRAligned(INSTRAligned),	// Instruction is aligned?
	.iPCADDR(oPCADDR),				// PC Address
	.iCacheDATA(iCacheDATA),		// Instruction from ICache
	.iDregDATA(DregDATA),			// DATA D Register
	.iDregADDR(DregADDR),			// ADDR D Register
	.iCLK(iCLK),
	.iRST(iRST)
);

pipEX_RV32 pipEX (
	.oRW(oRW),							// 1 = Read, 0 = Write
	.oMEM(oMEM),						// There is a memory transaction to be performed
	.oBRANCH(ItsBRANCH),				// There is a Branch to be taken
	.oBRANCHADDR(BranchADDR),		// Target Address for Program Counter	
	.oMEMDATA(iMEMDATA),				// DATA OUT Memory
	.oMEMADDR(oMEMADDR),				// ADDR OUT Memory
	.oDregDATA(DregDATA1),			// DATA OUT Destination Register	
	.oDregADDR(DregADDRint2),		// ADDR OUT Destination Register
	.oINVALID(),						// 10 = istruction not aligned, 01 = invalid instruction
	.iAregDATA(AregDATA),			// DATA IN A register
	.iBregDATA(BregDATA),			// DATA IN B register
	.iIMMDATA(IMMDATA),				// DATA IN Immediate
	.iPCADDR(PCADDR),					// ADDR IN PC
	.iDregADDR(DregADDRint1),		// ADDR IN D register
	.iOpType(OpType),					// One-Hot Encoding Operation Group
	.iDecodedOP(DecodedOP),			// Operation to be performed
	.iINSTRAligned(INSTRAligned), // Instruction is aligned?
	.iCLK(iCLK),
	.iRST(iRST)
);

pipMA_RV32 pipMA (
	.oDregADDR(DregADDRint3),		// ADDR OUT Destination Register	
	.oDregDATA(DregDATA2),			// DATA OUT Destination Register
	.iDregADDR(DregADDRint2),		// ADDR IN Destination Register
	.iDregDATA(DregDATA1),			// DATA IN Destination Register
	.iRW(oRW),							// 1 = Read, 0 = Write
	.iMEM(oMEM),			 			// There is a memory transaction to be performed
	.iCLK(iCLK),
	.iRST(iRST)
);

pipWB_RV32 pipWB (
	.oDregADDR(DregADDR),			// ADDR OUT Destination Register	
	.oDregDATA(DregDATA),			// DATA OUT Destination Register
	.iDregADDR(DregADDRint3),		// ADDR IN Destination Register	
	.iDregDATA(DregDATA2),			// DATA IN Destination Register
	.iCLK(iCLK),	
	.iRST(iRST)
);

endmodule
