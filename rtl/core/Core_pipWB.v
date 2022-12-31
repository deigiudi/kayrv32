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
-- Function: KayRV32 Writeback stage                                              -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"
 
module Core_pipWB (  
   // System
   input wire              i_Clk,
   input wire              i_Rstn,  
   // Control input
   input wire              i_StallEn,   // Stall enabler
   // From Memory Stage
   input wire              i_WriteEn,   
   input wire [`RegFAddr ] i_WriteAddr,
   input wire [`RegFWidth] i_WriteData,
   // To Register File
   output reg              o_WriteEn,   
   output reg [`RegFAddr ] o_WriteAddr,
   output reg [`RegFWidth] o_WriteData 
);

   always @(posedge i_Clk) 
   begin
      if (i_Rstn==1'b0 || i_StallEn) begin
         o_WriteEn   <= 0;
         o_WriteAddr <= 0;
         o_WriteData <= 0;
      end else begin
         o_WriteEn   <= i_WriteEn;         
         o_WriteAddr <= i_WriteAddr;
         o_WriteData <= i_WriteData;
      end
   end

endmodule