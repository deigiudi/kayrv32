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
-- Function    : Pipeline Controller
-- Description : Handles 2 functions: operands and memory data forwarding and
                 control and data load hazards detection
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"
`include "riscv_opcodes.vh"

module Core_pipControl (
  // System
  input wire             i_Clk,
  input wire             i_Rstn,
  input wire [`EventBus] i_EventBus,   

  // Forwarding
  input wire [1:0      ] i_ID_Input_sel,
  input wire [`RegFAddr] i_ID_rs1_Addr,
  input wire [`RegFAddr] i_ID_rs2_Addr,
  input wire [`RegFAddr] i_EX_rs2_Addr,
  input wire [`RegFAddr] i_ID_rd_Addr,
  input wire [`RegFAddr] i_EX_rd_Addr,
  input wire [`RegFAddr] i_MA_rd_Addr,
  input wire             i_EX_rd_wr_en,
  input wire             i_MA_rd_wr_en,
  output reg [1:0      ] o_forwardA,       // forwarding control ALU operand1
  output reg [1:0      ] o_forwardB,       // forwarding control ALU operand2
  output reg             o_forwardM,       // forwarding control for data mem writes   

  // Hazard detection
  input wire [31:0     ] i_IF_Instr,       // used to check for Data hazard
  input wire             i_ID_Load_en,     // is an ID instrn a load?   
  input wire             i_ID_Jump_en,     // TODO
  input wire             i_EX_Branch_en,   // TODO
  output wire            o_FlushEn_IFID,   // flush IF and ID in case of branch or jump taken
  output reg             o_FlushEn_EX,     // flush EX if branch taken   
  output reg             o_StallEn,        // pipeline stall signal

  // Event
  output reg             o_Interrupt       // error event notifier
  );



endmodule