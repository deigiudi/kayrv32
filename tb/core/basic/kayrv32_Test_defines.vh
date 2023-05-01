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
-- Project : KayRV32 Basic Testbench
-- Function: Shared hardware definitions in testbench
--------------------------------------------------------------------------------
*/

// ===============
// JUMPS        ==
// ===============
`define J_JAL    1
`define J_JALR   2

// ===============
// BRANCHES     ==
// ===============
`define B_BEQ    3
`define B_BNE    4
`define B_BLT    5
`define B_BGE    6
`define B_BLTU   7
`define B_BGEU   8

// ===============
// LOADS        ==
// ===============
`define L_LB     9
`define L_LH    10
`define L_LW    11
`define L_LBU   12
`define L_LHU   13

// ===============
// STORES       ==
// ===============
`define S_SB    14
`define S_SH    15
`define S_SW    16

// ===============
// Immediate    ==
// ===============
`define I_ADDI  17
`define I_ANDI  18
`define I_ORI   19
`define I_XORI  20
`define I_SLTI  21
`define I_SLLI  22
`define I_SRLI  23
`define I_SRAI  24

// ===============
// Upper Immediate
// ===============
`define U_LUI   25
`define U_AUIPC 26

// ===============
// Register based
// ===============
`define R_ADD   27
`define R_SUB   28
`define R_AND   29
`define R_OR    30
`define R_XOR   31
`define R_SLT   32
`define R_SLTU  32
`define R_SLL   33
`define R_SRL   34
`define R_SRA   35