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
  // System
  input wire             i_Clk,
  input wire             i_Rstn,
  // From Memory Stage
  input wire              i_MA_rd_rw_en,
  input wire [`RegFAddr ] i_MA_rd_addr,
  input wire              i_MA_memtoreg,
  input wire [`BusWidth ] i_MA_dataout,
  input wire [`BusWidth ] i_MA_aluout,
  // To Register File
  output reg              o_rd_wr_En,   
  output reg [`RegFAddr ] o_rd_Addr,
  output reg [`RegFWidth] o_rd_Data 
);

  always@(posedge i_Clk) 
  begin
    if(i_Rstn==1'b0) begin
      o_rd_wr_En <=  1'b0;
      o_rd_Addr  <= 32'b0;
      o_rd_Data  <= 32'b0;
    end else begin    
      o_rd_wr_En <= i_MA_rd_rw_en;      
      o_rd_Addr  <= i_MA_rd_addr;
      o_rd_Data  <= (i_MA_memtoreg) ? i_MA_dataout : i_MA_aluout;
    end
  end

endmodule