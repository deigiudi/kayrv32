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
-- Function    : Single Instruction simple testbench
-- Description : A simple testbench to test single isolated instructions

--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_Test_defines.vh"

// Select test to run: [1-35]
`define TEST_TO_RUN 0

module testbench();
  // System
  reg         tb_Clk = 0;
  reg         tb_Rstn;
  // IBUS
  wire [31:0] tb_iMem_Addr;
  reg  [31:0] tb_iMem_Data;
  // DBUS
  reg  [31:0] tb_dMem_ReadData;
  wire        tb_dMem_ReadEn;
  wire [31:0] tb_dMem_Addr;
  wire        tb_dMem_WriteEn;
  wire [31:0] tb_dMem_WriteData;
  wire        DMEM_rst;
  //Events
  wire        tb_Exception;
  wire        tb_Stall;  


  Core_STR KayRV32_Core(
  .i_Clk             (tb_Clk),
  .i_Rstn            (tb_Rstn),
  .o_iMem_En         (1'b1),
  .o_iMem_Addr       (tb_iMem_Addr),
  .i_iMem_Data       (tb_iMem_Data),
  .i_dMEM_Read_Data  (tb_dMem_ReadData),
  .o_dMEM_ReadEn     (tb_dMem_ReadEn),  
  .o_dMEM_Addr       (tb_dMem_Addr),
  .o_dMEM_WriteEn    (tb_dMem_WriteEn),
  .o_dMEM_Write_Data (tb_dMem_WriteData),
  .o_Interrupt       (tb_Exception),
  .o_Stall           (tb_Stall)
  );

  always#(10) tb_Clk = ~tb_Clk;     


  // dump vcd?
  initial begin
    if($test$plusargs("vcd")) begin
       $dumpfile("testbench.vcd");
       $dumpvars(0, testbench);
    end
  end

  // ROM SIMULATION
  parameter ROM_DEPTH = 32'hFFF;
  reg [31:0] ROM [0:ROM_DEPTH];
  integer i;


  // $readmemh loads program data into consecutive addresses, however 
  // RISC-V uses byte-addressable memory (i.e. a word at every fourth address)
  // A workaround is to ignore the lower two bits of the address.
  // Do this for both program data and data memory. 

  // Note that program memory and data memory are loaded from the same .hex file.
  // Data memory begins at 0x2000, this can be changed by editing /Scripts/memgen.sh and
  // changing the -Tdata parameter of riscv32-unknown-elf-ld.
  always@(posedge tb_Clk, negedge tb_Rstn) 
  begin
    if(tb_Rstn == 1'b0) begin
      tb_iMem_Data     <= 0;
      tb_dMem_ReadData <= 0;
    end else begin
      tb_iMem_Data     <= ROM[tb_iMem_Addr[31:2]];
      tb_dMem_ReadData <= ROM[tb_dMem_Addr[31:2]];
      ROM[tb_dMem_Addr[31:2]] <= tb_dMem_WriteData;
    end
  end

  // ===========================================================================
  // SIMULATION TASKS                                                         ==
  // ===========================================================================
  task LOAD_TEST;
  input integer TESTID;
    begin
      tb_Rstn = 0;
      #10;
      for (i=0; i<= ROM_DEPTH; i=i+1) begin
        ROM[i] = 0;
      end

      case(TESTID)
        // Jumps [1:2]
        `J_JAL:   $readmemh("mem/hex/jal.S.hex"  ,ROM);
        `J_JALR:  $readmemh("mem/hex/jalr.S.hex" ,ROM);

        // Branches [3:8]
        `B_BEQ:   $readmemh("mem/hex/beq.S.hex"  ,ROM);
        `B_BNE:   $readmemh("mem/hex/bne.S.hex"  ,ROM);
        `B_BLT:   $readmemh("mem/hex/blt.S.hex"  ,ROM);
        `B_BGE:   $readmemh("mem/hex/bge.S.hex"  ,ROM);
        `B_BLTU:  $readmemh("mem/hex/bltu.S.hex" ,ROM);
        `B_BGEU:  $readmemh("mem/hex/bgeu.S.hex" ,ROM);

        // Loads [9:13]
        `L_LB:    $readmemh("mem/hex/lb.S.hex"   ,ROM);
        `L_LH:    $readmemh("mem/hex/lh.S.hex"   ,ROM);
        `L_LW:    $readmemh("mem/hex/lw.S.hex"   ,ROM);
        `L_LBU:   $readmemh("mem/hex/lbu.S.hex"  ,ROM);
        `L_LHU:   $readmemh("mem/hex/lhu.S.hex"  ,ROM);
        
        // Stores [14:16]
        `S_SB:    $readmemh("mem/hex/sb.S.hex"   ,ROM);
        `S_SH:    $readmemh("mem/hex/sh.S.hex"   ,ROM);
        `S_SW:    $readmemh("mem/hex/sw.S.hex"   ,ROM);

        // Register-Immediate [17:24]
        `I_ADDI:  $readmemh("mem/hex/addi.S.hex" ,ROM);
        `I_ANDI:  $readmemh("mem/hex/andi.S.hex" ,ROM);
        `I_ORI:   $readmemh("mem/hex/ori.S.hex"  ,ROM);
        `I_XORI:  $readmemh("mem/hex/xori.S.hex" ,ROM);
        `I_SLTI:  $readmemh("mem/hex/slti.S.hex" ,ROM);
        `I_SLLI:  $readmemh("mem/hex/slli.S.hex" ,ROM);
        `I_SRLI:  $readmemh("mem/hex/srli.S.hex" ,ROM);
        `I_SRAI:  $readmemh("mem/hex/srai.S.hex" ,ROM);

        // Upper Immediate [25:26]
        `U_LUI:   $readmemh("mem/hex/lui.S.hex"  ,ROM);
        `U_AUIPC: $readmemh("mem/hex/auipc.S.hex",ROM);

        // Register-Register [27:35] 
        `R_ADD:   $readmemh("mem/hex/add.S.hex"  ,ROM);
        `R_SUB:   $readmemh("mem/hex/sub.S.hex"  ,ROM);
        `R_AND:   $readmemh("mem/hex/and.S.hex"  ,ROM);
        `R_OR:    $readmemh("mem/hex/or.S.hex"   ,ROM);
        `R_XOR:   $readmemh("mem/hex/xor.S.hex"  ,ROM);
        `R_SLT:   $readmemh("mem/hex/slt.S.hex"  ,ROM);
        `R_SLTU:  $readmemh("mem/hex/sltu.S.hex" ,ROM);
        `R_SLL:   $readmemh("mem/hex/sll.S.hex"  ,ROM);
        `R_SRL:   $readmemh("mem/hex/srl.S.hex"  ,ROM);
        `R_SRA:   $readmemh("mem/hex/sra.S.hex"  ,ROM);                
      endcase
    end
  endtask // LOAD_TEST

  // ===========================================================================  
  integer t;
  task EVAL_TEST;
    input integer TESTID;
    begin
      tb_Rstn = 1;
      for(t=0; t<=1000000; t=t+1) begin
        @(posedge tb_Clk) begin
          // Check if test pass
          // pass condition: GP=1 , A7=93, A0=0
          if((UUT.id_stage_i.regfile_i.regfile_data[3] == 1)   &&   
             (UUT.id_stage_i.regfile_i.regfile_data[17] == 93) && 
             (UUT.id_stage_i.regfile_i.regfile_data[10] == 0))    
          begin
            $display("TEST PASSED; ID: %0d", TESTID);
            t=1000000;
          end else if (tb_Exception == 1) begin
            $display("EXCEPTION ASSERTED, TEST FAILED");
            $display("FAILED TEST ID: %0d", TESTID);
            $finish;
          end else if (t==999999) begin
            $display("TEST FAILED: TIMED OUT");
            $display("FAILED TEST ID: %0d", TESTID);
            $finish;
          end
        end
      end
    end
  endtask // EVAL_TEST


  // ===========================================================================
  // TESTBENCH                                                                ==
  // ===========================================================================
  integer j;
  initial begin
    tb_Rstn = 0;
    #100;

    if($test$plusargs("runall")) begin
      $display("Running all tests.");
      for (j=`R_ADD; j<=`S_SW; j=j+1) begin
        $display("***********************************");
        $display("Running Test ID: %0d", j);
        LOAD_TEST(j);
        #100;
        EVAL_TEST(j);
      end
      $display("");
      $display("***********************************");
      $display("ALL TESTS PASSED !");
      $finish;
    end else begin
      $display("Running Test ID: %0d", `TEST_TO_RUN);
      LOAD_TEST(`TEST_TO_RUN);
      EVAL_TEST(`TEST_TO_RUN);
      $display("***********************************");
      $display("TEST PASSED !");
      $finish;
    end
  end
endmodule