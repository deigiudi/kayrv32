/*
--------------------------------------------------------------------------------
-- COPYRIGHT (c) 2023, Alessandro Dei Giudici <alessandro.deig@live.it>
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
-- Project     : KayRV32
-- Function    : Writeback stage
-- Description : A combinatorial control block to select data either to be 
                 written in the register file from data memory or ALU output
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"
 
module Core_pipWB (
   // From Memory Stage
   input wire              i_MEM_RW_en,  
   input wire              i_MEM_memtoreg,    
   input wire [`RegFAddr ] i_MEM_rd_addr,
   input wire [`BusWidth ] i_MEM_dataout,
   input wire [`BusWidth ] i_MEM_aluout,
   // To Register File
   output reg              o_WriteEn,   
   output reg [`RegFAddr ] o_WriteAddr,
   output reg [`RegFWidth] o_WriteData 
);

   always @*
   begin
     o_WriteEn   = i_MEM_RW_en;      
     o_WriteAddr = i_MEM_rd_addr;
     o_WriteData = (i_MEM_memtoreg) ? i_MEM_data : i_MEM_aluout;
   end

endmodule