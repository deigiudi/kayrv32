/*
--------------------------------------------------------------------------------
-- COPYRIGHT (c) 2022, Alessandro Dei Giudici <alessandro.deig@live.it>
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
-- Project : KayRV32                                                           -
-- Function: KayRV32 Pipeline Controller                                       -
--------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps
`include "kayrv32_defines.vh"

module Core_pipControl (
   // System
   input wire             i_Clk,
   input wire             i_Rstn,

   // Event
   input wire             i_EventBus[`EventBus],
   output reg             o_Interrupt // error event notifier
   );

   // Control logic
   always @(posedge i_Clk)
   begin
      if (i_Rstn==1'b0) begin  // Reset is active
         o_Interrupt <= 0;
      end else begin
         o_Interrupt <= |i_EventBus;
      end
   end
endmodule