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
-- Function: RISC-V instruction opcodes, funct3, funct7 fields
--------------------------------------------------------------------------------
*/

// =====instruction opcodes ==
`define OP_LUI      7'b0110111
`define OP_AUIPC    7'b0010111
`define OP_JAL      7'b1101111
`define OP_JALR     7'b1100111
`define OP_BRANCH   7'b1100011
`define OP_LOAD     7'b0000011
`define OP_STORE    7'b0100011
`define OP_IMMED    7'b0010011
`define OP_REGS     7'b0110011
`define OP_FENCE    7'b0001111
`define OP_MISC     7'b1110011
// ===========================


// =====================funct3
// JALR
`define FUNCT3_JALR     3'b000
// BRANCH
`define FUNCT3_BEQ      3'b000
`define FUNCT3_BNE      3'b001
`define FUNCT3_BLT      3'b100
`define FUNCT3_BGE      3'b101
`define FUNCT3_BLTU     3'b110
`define FUNCT3_BGEU     3'b111
// LOAD
`define FUNCT3_LB       3'b000
`define FUNCT3_LH       3'b001
`define FUNCT3_LW       3'b010
`define FUNCT3_LBU      3'b100
`define FUNCT3_LHU      3'b101
// STORE
`define FUNCT3_SB       3'b000
`define FUNCT3_SH       3'b001
`define FUNCT3_SW       3'b010
// IMMEDIATE
`define FUNCT3_ADDI     3'b000
`define FUNCT3_SLTI     3'b010
`define FUNCT3_SLTIU    3'b011
`define FUNCT3_XORI     3'b100
`define FUNCT3_ORI      3'b110
`define FUNCT3_ANDI     3'b111
`define FUNCT3_SLLI     3'b001
`define FUNCT3_SRxI     3'b101
// ALL REGISTERS
`define FUNCT3_ADDSUB   3'b000
`define FUNCT3_SLL      3'b001
`define FUNCT3_SLT      3'b010
`define FUNCT3_SLTU     3'b011
`define FUNCT3_XOR      3'b100
`define FUNCT3_SRx      3'b101
`define FUNCT3_OR       3'b110
`define FUNCT3_AND      3'b111
// FENCE
`define FUNCT3_FENCE    3'b000
`define FUNCT3_FENCEI   3'b001
// ENVIRONMENT
`define FUNCT3_ENVIRON  3'b000
// CONTROL STATUS REGISTERS
`define FUNCT3_CSRRW    3'b001
`define FUNCT3_CSSRS    3'b010
`define FUNCT3_CSSRC    3'b011
`define FUNCT3_CSSRWI   3'b101
`define FUNCT3_CSSRSI   3'b110
`define FUNCT3_CSSRCI   3'b111
// ===========================


//==================funct7====
// PORT BASED
`define FUNCT7_ADD  7'b0000000
`define FUNCT7_SUB  7'b0100000
`define FUNCT7_XOR  7'b0000000
`define FUNCT7_OR   7'b0000000
`define FUNCT7_AND  7'b0000000
// SHIFT
`define FUNCT7_SLL  7'b0000000
`define FUNCT7_SLLI 7'b0000000
`define FUNCT7_SLT  7'b0000000
`define FUNCT7_SLTU 7'b0000000
`define FUNCT7_SRL  7'b0000000
`define FUNCT7_SRA  7'b0100000
// ===========================