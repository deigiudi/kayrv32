/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Istruction Fetch Stage Unit
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

module pipIF_RV32 (
	output reg [31:0] oPCADDR,	// Data Address to read from ICache
	input  [31:0] iBranchADDR,	// Branch Address	to be jumped in
	input  iBRANCH,				// There is a Branch to be taken		
	input  iStallI,				// ICache hasn't got data needed or Bubble in pipeline	
	input  iCLK,
	input  iRST 
	);
	
	reg [31:2] reg_PC;
	wire stall;
	
	/* PC logic */
	always @(posedge iCLK)
	begin
		if (iRST) begin
			reg_PC <= 30'd0;
		   end 
		else begin
			case ({iStallI, iBRANCH})
				2'b00 : // No branch, no stall in pipeline
					reg_PC <= reg_PC + 30'd1;			
				2'b01 : // There is a branch in pipeline to be taken
					reg_PC <= iBranchADDR[31:2];
				2'b10 : // There is a stall in pipeline
					reg_PC <= reg_PC;
				2'b11 : // ERROR, it shouldn't happen
					reg_PC <= 30'd0;					
			endcase					
			oPCADDR = { reg_PC, 2'b00 };	
		   end
	end
 
endmodule
