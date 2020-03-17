/*
 *  SasRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the TOP Architecture
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

module TOP_RV32 (
	output [31:0] oGPIO_OUT,
	input  [31:0] iGPIO_IN,
	output oUART_TX,
	input	 iUART_RX,
	input	 iCLK,
	input	 iRST
);

// ICache <=> Core
wire [31:0] PCADDR;
wire [31:0] PCDATA;
wire ICacheStall;

// DCache <=> Core
wire RW;
wire MEM;
wire DCacheStall;
wire [31:0] MEMADDR;
wire [31:0] MEMDATA;

CacheCTRL CacheCTRL (	
	.iCLK(iCLK),
	.iRST(iRST)
	);

CacheI_RV32 #(
	.MEMSIZE(4)
	) CacheI (	
	.oStallI(ICacheStall),	// iCache hasn't got data needed
	.oPCDATA(PCDATA),			// DATA OUT Destination Register
	.iPCADDR(PCADDR),			// ADDR IN Destination Register
	.iCLK(iCLK),
	.iRST(iRST)
	);
	
CacheD_RV32 #(
	.MEMSIZE(4)
	) CacheD (
	.oStallD(DCacheStall),	// DCache hasn't got data needed
	.ioMEMDATA(MEMDATA),		// DATA OUT Memory
	.iMEMADDR(MEMADDR),		// ADDR IN MEMORY
	.iRW(RW),					// 1 = Read, 0 = Write
	.iMEM(MEM),					// There is a memory transaction to be performed		
	.iCLK(iCLK),
	.iRST(iRST)
	);
	
Core_RV32 Core (
	.oRW(RW),					// 1 = Read, 0 = Write
	.oMEM(MEM),					// There is a memory transaction to be performed	
	.oCacheADDR(PCADDR),		// Data Address to read from ICache
	.oMEMADDR(MEMADDR),		// ADDR OUT Memory
	.iCacheDATA(PCDATA),		// Instruction from ICache	
	.iMEMDATA(MEMDATA),		// DATA IN Memory
	.iStallI(ICacheStall),	// ICache hasn't got data needed = Bubble in pipeline
	.iStallD(DCacheStall),	// DCache hasn't got data needed = Bubble in pipeline		
	.iCLK(iCLK),
	.iRST(iRST)
);

endmodule
