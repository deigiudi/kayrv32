/*
 *  BenchRV32I -- A simple RISC-V (RV32I) Processor Core
 *  Copyright (c) 2020, Alessandro Dei Giudici <alessandro.deig@live.it>
 *  
 *  A file containing the definitions of the Cache Controller Unit
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   
 *
 */
 
`timescale 1ns / 1ps
 
module CacheCTRL #(
	parameter Nbit = 128,			// number of bit to serialize
	parameter CLK_DIV = 100			// oCLK = iCLK/(2*CLK_DIV)
	)(
	output reg [N-1:0] oDATApar,	// DATA read from memory
	output reg oTXend,				// TX data completed, iDATApar available
	output reg oCLK,
	output reg oSS,
	output reg oMOSI,
	input [N-1:0] iDATApar,			// DATA write to memory
	input iTXstart,					// start TX on serial line
	input iMISO,
	input iCLK,
	input iRST
	);

	wire [CLK_DIV*2:0] r_counter_clock;	// from 0 to [CLK_DIV*2:0] 
	wire [N-1:0] r_tx_data, r_rx_data;	// data to sent and received
	wire [N:0] r_counter_data;				// from 0 to N
	wire r_sclk_rise, r_sclk_fall, r_counter_clock_ena, w_tc_counter_data;
	wire r_tx_start;		// start TX on serial line
	
	reg r_st_present, w_st_next;
	parameter ST_RESET = 2'b11, 
				 ST_TX_RX = 2'b01, 
				 ST_END	 = 2'b10;

	assign w_tc_counter_data = (r_counter_data>0)? 1'b0 : 1'b1;

	always @(posedge iCLK,iRST)
	begin: p_state
		if(iRST)
			r_st_present <= ST_RESET;
		else
			r_st_present <= w_st_next;
	end

	always @(posedge r_st_present, w_tc_counter_data, r_tx_start, r_sclk_rise, r_sclk_fall)
	begin: p_comb
		case (r_st_present)
			ST_TX_RX :
				if ((w_tc_counter_data= 1'd1) & (r_sclk_rise=1'b1)) 
					w_st_next <= ST_END;
				else
					w_st_next <= ST_TX_RX;
		 	ST_END : 
				if(r_sclk_fall)
					w_st_next <= ST_RESET;  
				else
				  w_st_next <= ST_END;  
		 	default: // ST_RESET
				if(r_tx_start)
					w_st_next <= ST_TX_RX;
				else
					w_st_next <= ST_RESET;
		endcase
	end
	
	always @(posedge iCLK,iRST)
	begin: p_state_out
		if(iRST) begin
			r_tx_start <= 1'b0;
			r_tx_data  <= (others=>'0');
			r_rx_data  <= (others=>'0');
			oTXend     <= 1'b0;
			iDATApar   <= (others=>'0'); 
			r_counter_data <= N-1;
			r_counter_clock_ena <= 1'b0;
			oCLK  <= 1'b1;
			oSS   <= 1'b1;
			oMOSI <= 1'b1;
			end
		else begin
			r_tx_start           <= iTXstart;
			case (r_st_present)
				ST_TX_RX :
					oTXend             <= '0';
					r_counter_clock_ena  <= '1';
					if(r_sclk_rise='1') begin
						oCLK               <= '1';
						r_rx_data            <= r_rx_data(N-2 downto 0)&iMISO;
						if(r_counter_data>0)
							r_counter_data       <= r_counter_data - 1;
						end
					else if(r_sclk_fall='1') begin
						oCLK               <= '0';
						oMOSI               <= r_tx_data(N-1);
						r_tx_data            <= r_tx_data(N-2 downto 0)&'1';
						end
					oSS                 <= '0';
				ST_END :
					oTXend             <= r_sclk_fall;
					iDATApar      <= r_rx_data;
					r_counter_data       <= N-1;
					r_counter_clock_ena  <= '1';
					oSS                 <= '0';

				default: // ST_RESET
					r_tx_data            <= oDATApar;
					oTXend             <= '0';
					r_counter_data       <= N-1;
					r_counter_clock_ena  <= '0';
					oCLK               <= '1';
					oSS                 <= '1';
					oMOSI               <= '1';
				endcase
			end
	end

	always @(posedge iCLK,iRST)
	begin: p_counter_clock
		if(iRST) begin
			r_counter_clock            <= 0;
			r_sclk_rise                <= '0';
			r_sclk_fall                <= '0';
			end
		else begin
			if(r_counter_clock_ena='1') begin	// sclk = '1' by default 
				if(r_counter_clock=CLK_DIV-1) then  -- firse edge = fall
					r_counter_clock            <= r_counter_clock + 1;
					r_sclk_rise                <= '0';
					r_sclk_fall                <= '1';
				else if(r_counter_clock=(CLK_DIV*2)-1) begin
					r_counter_clock            <= 0;
					r_sclk_rise                <= '1';
					r_sclk_fall                <= '0';
					end
				else begin
					r_counter_clock            <= r_counter_clock + 1;
					r_sclk_rise                <= '0';
					r_sclk_fall                <= '0';
					end
			else begin
				r_counter_clock            <= 0;
				r_sclk_rise                <= '0';
				r_sclk_fall                <= '0';
			end
		end
	end

endmodule
