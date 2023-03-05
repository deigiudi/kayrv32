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
-- Function    : Branch Generator
-- Description : Computes either a PC-relative or a register-offset target 
                 based on the control signals generated in the decode stage
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"

module Core_pipBranchGen (
  // System
  input wire               i_Clk,
  input wire               i_Rstn,
  // Input
  input  wire [ 1:0      ] i_ID_JAL_OperSel,
  input  wire [`RegFAddr ] i_ID_rs1_Addr,
  input  wire [`RegFWidth] i_ID_rs1_Data,
  input  wire [`MemAddr  ] i_ID_PC,
  input  wire [`RegFWidth] i_ID_offset,    
  input  wire [`RegFWidth] i_EX_rd_Addr,
  input  wire              i_EX_rd_wr_en,
  input  wire [`RegFWidth] i_EX_alu_result
  // Output
  output reg  [`MemAddr  ] o_Branch_Addr,
  );

  reg  [`MemAddr] r_data;

  // If the two statements below are true:
  // - there is an instruction in EX that writes to rs1
  // - the jump target is a register offset (JALR)
  // then forward the register data from the EX alu result.
  always @(posedge i_Clk)
  begin
    if (i_Rstn==1'b0) begin  // Reset is active
      o_Branch_Addr = 0;
    end else begin
      if (i_ID_JAL_OperSel[1] && (i_EX_rd_Addr==i_ID_rs1_Addr) && i_EX_rd_wr_en)
        r_data = i_EX_alu_result;
      end else begin
        r_data = i_ID_rs1_Data;
      end

      o_Branch_Addr = 0;
      case(i_ID_JAL_OperSel)
          2'b01: o_Branch_Addr = r_data  + $signed(i_ID_offset);
          2'b10: o_Branch_Addr = i_ID_PC + $signed(i_ID_offset);
      endcase
    end
  end
    
endmodule