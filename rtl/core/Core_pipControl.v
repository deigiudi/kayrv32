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
`include "kayrv32_RTL_defines.vh"
`include "riscv_opcodes.vh"

module Core_pipControl (
  // System
  input wire              i_Clk,
  input wire              i_Rstn,
  input wire [`EventBus ] i_EventBus,   

  // Forwarding
  input wire [1:0       ] i_ID_Input_sel,
  input wire [`RegFAddr ] i_ID_rs1_Addr,
  input wire [`RegFAddr ] i_ID_rs2_Addr,
  input wire [`RegFAddr ] i_EX_rs2_Addr,
  input wire [`RegFAddr ] i_ID_rd_Addr,
  input wire [`RegFAddr ] i_EX_rd_Addr,
  input wire [`RegFAddr ] i_MA_rd_Addr,
  input wire              i_EX_rd_wr_en,
  input wire              i_MA_rd_wr_en,
  output reg [1:0       ] o_forward_op1,  // forwarding control ALU operand1
  output reg [1:0       ] o_forward_op2,  // forwarding control ALU operand2
  output reg              o_forward_opM,  // forwarding control for data mem writes   

  // Hazard detection
  input wire [31:0      ] i_IF_Instr,     // used to check for Data hazard
  input wire              i_ID_Load_en,   // is an ID instrn a load?   
  input wire              i_ID_Jump_en,   // TODO
  input wire              i_EX_Branch_en, // TODO
  output reg              o_FlushEn_IFID, // flush IF and ID in case of branch or jump taken
  output reg              o_FlushEn_EX,   // flush EX if branch taken   
  output reg              o_StallEn,      // pipeline stall signal

  // Event
  output reg              o_Interrupt     // error event notifier
  );

  // Needed for Data Load Hazard logic
  wire [6:0] w_opcode = i_IF_Instr[6:0];
  // Needed for Control Hazard logic
  reg        r_IF_ID_Flush1;
  reg        r_IF_ID_Flush2;

  // Shared logic gate computation
  wire       w_EX_cond = (i_EX_rd_wr_en == 1'b1) && (i_EX_rd_Addr != 0);
  wire       w_MA_cond = (i_MA_rd_wr_en == 1'b1) && (i_MA_rd_Addr != 0);  

  always @(posedge i_Clk)
  begin
    if (i_Rstn==1'b0) begin
      r_IF_ID_Flush2 <= 0;
      o_forward_op1  <= 0;
      o_forward_op2  <= 0;
      o_forward_opM  <= 0;
      o_FlushEn_IFID <= 0;
      o_FlushEn_EX   <= 0;
      o_StallEn      <= 0;
      o_Interrupt    <= 0;
    end else begin
      // OR reduction logic of Events
      o_Interrupt <= |i_EventBus;

      // =======================================================================    
      // FORWARDING related logic                                             ==
      // =======================================================================
      // Check i_ID_Input_sel : is an ALU operand going to be an immediate?
      //  -> YES: no hazard
      //  -> NO:  perform checks
      // =======================================================================
      // ALU forwarding logic: op1 ==
      // EX HAZARD: forward result from EXECUTE stage
      if ((i_ID_Input_sel[1] != 1) && w_EX_cond && (i_EX_rd_Addr == i_ID_rs1_Addr)) begin
        o_forward_op1 = 2'b10;
      end else
      // WB hazard: forward result from MEMORY ACCESS stage
      if ((i_ID_Input_sel[0] != 1) && w_MA_cond && (i_MA_rd_Addr == i_ID_rs2_Addr) &&
         ~(w_EX_cond && (i_EX_rd_Addr == i_ID_rs2_Addr))) begin
        o_forward_op1 = 2'b01; 
      end else begin
      // NO Forwarding
        o_forward_op1 = 2'b0;                 
      end

      // =======================================================================
      // ALU forwarding logic: op2 ==
      // EX HAZARD: forward result from EXECUTE stage
      if ((i_ID_Input_sel[0] != 1) && w_EX_cond && (i_EX_rd_Addr == i_ID_rs2_Addr)) begin
         o_forward_op2 = 2'b10;
      end else 
      // WB hazard: forward result from MEMORY ACCESS stage
      if ((i_ID_Input_sel[0] != 1) && w_MA_cond && (i_MA_rd_Addr == i_ID_rs2_Addr) &&
         ~(w_EX_cond && (i_EX_rd_Addr == i_ID_rs2_Addr))) begin
        o_forward_op2 = 2'b01;
      // NO Forwarding
      end else begin
        o_forward_op2 = 2'b0;         
      end

      // =======================================================================
      // MEM forwarding logic: opM
      // MEM hazard: forward from previous value
      if (w_MA_cond && (i_EX_rs2_Addr == i_MA_rd_Addr))
        o_forward_opM = 1;
      // No Forwarding
      else
        o_forward_opM = 0;
      end


      // =======================================================================    
      // DATA LOAD HAZARD related logic                                       ==
      // =======================================================================
      //  Checks for the following conditions:
      //  1) is a load instruction present in ID pipeline reg?
      //  2a) does the next instruction (in IF pipeline reg) read any registers?
      //  2b) are any of the registers to be read dependent on the load?
      //   
      //  if all true, stall the pipeline.
      //  
      //  IF_Rs1_addr    = Instruction[19:15];
      //  IF_Rs2_addr    = Instruction[24:20];
      if (i_ID_Load_en == 1) begin
        if (((w_opcode == `OP_REGS) || (w_opcode == `OP_BRANCH) || (w_opcode == `OP_STORE)) &&
            ((i_ID_rd_Addr == i_IF_Instr[19:15]) || (i_ID_rd_Addr == i_IF_Instr[24:20]))) begin
           o_StallEn = 1;
        end else 
        if (((w_opcode == `OP_IMMED) ||(w_opcode == `OP_LOAD)) && 
             (i_IF_Instr[19:15] == i_ID_rd_Addr)) begin
          o_StallEn = 1;
        end else begin 
          o_StallEn = 0;   
        end
      end else begin 
        o_StallEn = 0;
      end


      // =======================================================================    
      // CONTROL HAZARD related logic                                         ==
      // =======================================================================
      //  -> if a branch or jump is taken, the pipeline needs to be flushed for two cycles.
      //     -> a flush is defined as setting all control signals to 0, effectively
      //        replacing whatever instructions were being processed with NOP
      //  -> it is unknown whether a branch or jump is taken until the end of EX stage.
      //  -> if a branch is taken, IF, ID, and EX need to be flushed.
      //  -> if a jump is taken, only IF and ID need to be flushed.    
      o_FlushEn_IFID = (r_IF_ID_Flush1 || r_IF_ID_Flush2);

      if((i_EX_Branch_en) || (i_ID_Jump_en)) begin
        r_IF_ID_Flush1 = 1;
      end else begin
        r_IF_ID_Flush1 = 0;
      end

      r_IF_ID_Flush2 = r_IF_ID_Flush1;

      o_FlushEn_EX <= (i_EX_Branch_en) ? 1:0;
    end
endmodule