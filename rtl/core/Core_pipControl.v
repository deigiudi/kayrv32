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
  output reg [1:0       ] o_forwardA,     // forwarding control ALU operand1
  output reg [1:0       ] o_forwardB,     // forwarding control ALU operand2
  output reg              o_forwardM,     // forwarding control for data mem writes   

  // Hazard detection
  input wire [31:0      ] i_IF_Instr,     // used to check for Data hazard
  input wire              i_ID_Load_en,   // is an ID instrn a load?   
  input wire              i_ID_Jump_en,   // TODO
  input wire              i_EX_Branch_en, // TODO
  output wire             o_FlushEn_IFID, // flush IF and ID in case of branch or jump taken
  output reg              o_FlushEn_EX,   // flush EX if branch taken   
  output reg              o_StallEn,      // pipeline stall signal

  // Event
  output reg              o_Interrupt     // error event notifier
  );

  // ALU operand forwarding: 
  // 1) no forwarding, operand will come from regfile
  // 2) forward operand from prior ALU result 
  // 3) forward operand from MEM stage output
  //
  // Check i_ID_Input_sel : is an ALU operand going to be an immediate?
  //  -> YES: don't forward
  //  -> NO:  perform checks
  //    
  // ForwardA_o Truth Table:
  // ---------------------
  //  00 -> ALU op1 comes from register file  (no hazard)
  //  10 -> ALU op1 forwarded from ALU result (EX hazard)
  //  01 -> ALU op1 forwarded from data memory or earlier ALU result (MEM hazard)
  //    
  // ForwardB_o Truth Table:
  // ---------------------
  //  00 -> ALU op2 comes from register file
  //  10 -> ALU op2 forwarded from ALU result (EX hazard)
  //  01 -> ALU op2 forwarded from data memory or earlier ALU result (MEM hazard)
  //     
  // Forward A combinatorial logic
  always@* begin

  // EX HAZARD
  // -> forward from EX_ALU_result to op1
  if ((i_EX_rd_wr_en == 1'b1) &&     
      (i_EX_rd_Addr  != 0   ) &&
      (i_EX_rd_Addr  == i_ID_rs1_Addr) &&
      (i_ID_Input_sel[1] != 1)
     )
     o_forwardA = 2'b10; 
  else 

  // WB hazard 
  // -> forward from WB_Rd_data to op1
  if ( (i_MA_rd_wr_en == 1'b1) &&
      (i_MA_rd_Addr       != 0   ) &&
      ~ ( (i_EX_rd_wr_en  == 1'b1) &&
          (i_EX_rd_Addr        != 0)    &&
          (i_EX_rd_Addr        == i_ID_rs1_Addr)) &&
      (i_MA_rd_Addr       == i_ID_rs1_Addr)&&
      (i_ID_Input_sel[1] != 1)         
     )
     o_forwardA = 2'b01; 
  else
     o_forwardA = 2'b0;                 
  end
   
  always@* begin
     // EX HAZARD
     // -> forward from EX_ALU_result to op2
     if ( (i_EX_rd_wr_en == 1'b1) &&     
          (i_EX_rd_Addr       != 0   ) &&
          (i_EX_rd_Addr       == i_ID_rs2_Addr) &&
          (i_ID_Input_sel[0] != 1)
         )
         o_forwardB = 2'b10;
     else 

     // WB hazard 
     // -> forward from WB_Rd_data to op2
     if ( (i_MA_rd_wr_en == 1'b1) &&
          (i_MA_rd_Addr       != 0   ) &&
          ~ ( (i_EX_rd_wr_en  == 1'b1) && (i_EX_rd_Addr != 0) && (i_EX_rd_Addr == i_ID_rs2_Addr))
          && (i_MA_rd_Addr == i_ID_rs2_Addr) &&
          (i_ID_Input_sel[0] != 1)
        ) 
         o_forwardB = 2'b01;
     else
         o_forwardB = 2'b0;         
  end


  //  Memory write forwarding:
  //  1) no forwarding, write value of rs2 to data mem
  //  2) forward data from prior MEM stage output 
  always@* begin
       if ( 
           (i_MA_rd_wr_en == 1'b1) &&
           (i_EX_rs2_Addr == i_MA_rd_Addr) &&
           (i_MA_rd_Addr != 0) 
          )
           o_forwardM = 1; // forward from MEM_ALU_result
       else
          o_forwardM = 0;
  end


  // Data Load Hazard Detection
  //     
  //   Checks for the following conditions:
  //   1) is a load instruction present in ID pipeline reg?
  //   2a) does the next instruction (in IF pipeline reg) read any registers?
  //   2b) are any of the registers to be read dependent on the load?
  //   
  //   if all true, stall the pipeline.
  //   
  //   IF_Rs1_addr    = Instruction[19:15];
  //   IF_Rs2_addr    = Instruction[24:20];
  wire [6:0] opcode = i_IF_Instr[6:0]; // internal 
  always@* begin
    if(i_ID_Load_en == 1) begin
      if(((opcode == `OP_REGS) || (opcode == `OP_BRANCH) || (opcode == `OP_STORE) ) &&
         ((i_ID_rd_Addr == i_IF_Instr[19:15]) || (i_ID_rd_Addr == i_IF_Instr[24:20]) )
       )     
      begin
         o_StallEn = 1;
      end else if( ( (opcode == `OP_IMMED) ||(opcode == `OP_LOAD) )  &&
                  ( (i_ID_rd_Addr == i_IF_Instr[19:15]) )
                )
      begin
         o_StallEn = 1;
      end     
      else o_StallEn = 0;   
      end
      else o_StallEn = 0;
  end


  // Control Hazard Detection:
  //     
  //   -> if a branch or jump is taken, the pipeline needs to be flushed for two cycles.
  //       -> a flush is defined as setting all control signals to 0, effectively
  //          replacing whatever instructions were being processed with NOP
  // 
  //   -> it is unknown whether a branch or jump is taken until the end of EX stage.
  //   -> if a branch is taken, IF, ID, and EX need to be flushed.
  //   -> if a jump is taken, only IF and ID need to be flushed.    
  reg IF_ID_Flush1; // internal 1
  reg IF_ID_Flush2; // internal 2

  assign o_FlushEn_IFID = (IF_ID_Flush1 || IF_ID_Flush2);

  always@* begin
      if((i_EX_Branch_en) || (i_ID_Jump_en)) 
       IF_ID_Flush1 = 1;
      else                                      
       IF_ID_Flush1 = 0;
  end

  always@(posedge i_Clk) begin
      if(i_Rstn == 1'b0) 
       IF_ID_Flush2 <= 0;
      else                 
       IF_ID_Flush2 <= IF_ID_Flush1;
      
  end

  // Flush EX if a branch is taken
  always@* begin
    o_FlushEn_EX = (i_EX_Branch_en) ? 1:0;
  end

  // Control logic
  always @(posedge i_Clk)
  begin
    if (i_Rstn==1'b0) begin  // Reset is active
       o_Interrupt <= 0;
    end else begin
       o_Interrupt <= |i_EventBus;
    end
  end


    // If the two statements below are true:
  // - there is an instruction in EX that writes to rs1
  // - the jump target is a register offset (JALR)
  // then forward the register data from the EX alu result.
  if (i_ID_JAL_OperSel[1] && (i_EX_rd_Addr==i_ID_rs1_Addr) && i_EX_rd_wr_en) begin
    r_data <= i_EX_alu_result;
  end else begin
    r_data <= i_ID_rs1_Data;
  end   

endmodule