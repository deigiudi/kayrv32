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
-- Project : KayRV32
-- Function: Shared hardware definitions in the RTL
--------------------------------------------------------------------------------
*/

// ===========================
// DEFINE                   ==
// ===========================
`define PCWidth           31:0
`define MemAddr           31:0
`define BusWidth          31:0
`define RegFAddr           4:0
`define RegFWidth         31:0
`define RegFDepth         31:0
`define OffWidth          12:0
`define EventBus           2:0


//============================
`define PortSel      5:0
//-------------------PortSel--
`define PORT_NOP     6'b000000
`define PORT_SHIFT   6'b000001
`define PORT_LOGIC   6'b000010
`define PORT_ARITH   6'b000011
`define PORT_JUMP    6'b000100
`define PORT_BRANCH  6'b001000
`define PORT_LOAD    6'b010000
`define PORT_STORE   6'b100000


//============================
`define OperSel        3:0
//------NOP------------OpSel--
`define OP_NOP         4'b0000
//------SHIFT----------OpSel--
`define OP_SLL         4'b0001
`define OP_SRL         4'b0010
`define OP_SRA         4'b0100
//------LOGIC----------OpSel--
`define OP_AND         4'b0001
`define OP_OR          4'b0010
`define OP_XOR         4'b0100
//------ARITH----------OpSel--
`define OP_ADD         4'b0001 
`define OP_SUB         4'b0010
`define OP_SLT         4'b0100
`define OP_SLTU        4'b1000
//------JUMP-----------OpSel--
`define OP_JAL         4'b0100
`define OP_JALR        4'b1000
//------BRANCH---------OpSel--
`define OP_BEQ         4'b0010
`define OP_BNE         4'b0011
`define OP_BLT         4'b0100
`define OP_BLTU        4'b0101
`define OP_BGE         4'b1000
`define OP_BGEU        4'b1001
//------LOAD-----------OpSel--
`define OP_LW          4'b0001
`define OP_LH          4'b0100
`define OP_LHU         4'b0110
`define OP_LB          4'b1000
`define OP_LBU         4'b1010
//------STORE----------OpSel--
`define OP_SW          4'b0001
`define OP_SH          4'b0100
`define OP_SB          4'b1000


//============================
`define InputSel         0:0
//--------------------InputSel
`define IN_RS2_RS1       2'b00
`define IN_IMM2_RS1      2'b10
`define IN_IMM2_IMM1     2'b11