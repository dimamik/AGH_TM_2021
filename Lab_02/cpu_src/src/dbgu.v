`timescale 1ns / 1ps
/*
* The MIT License (MIT)
* Copyright (c) 2021 Ada Brzoza
* Permission is hereby granted, free of charge, to any person obtaining 
* a copy of this software and associated documentation files (the "Software"), 
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
* THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


module dbgu(
	input clk_in,
	input clk_ser_in,
	input reset_n,
	input trig,
	output s_out,
	output reg bsy,
	output reg [7:0] p_out,
	input [3:0] dbg_r0,
	input [3:0] dbg_r1,
	input [3:0] dbg_pc,
	input [2:0] dbg_istate,
	input [7:0] dbg_mem_dr,
	input [7:0] dbg_mem_dw,
	input [3:0] dbg_mem_a,
	
	output reg [3:0] mem_a,
	input [7:0] mem_d,
	output mem_clk,
	
	output [4:0] uart_state,
	output [7:0] uart_buf,
	output [2:0] dbg_acntr
	);
	
	parameter LAST_STATE = 38 + 16*3;
	reg [7:0] state;
	reg tx_chr_start;
	wire tx_chr_bsy;
	reg [2:0] acntr;
	wire startpulse;
	
	assign mem_clk = clk_in;
	assign dbg_acntr = acntr;
	
	pulsegen gen1 (
		.reset_n(reset_n),
		.clk(clk_in),
		.trig(trig),
		.pulse(startpulse)
	);
	
	//assign startpulse = trig;
	
	uart_tx u1(
		.data(p_out),
		.reset_n(reset_n),
		.start(tx_chr_start),
		.bitclk(clk_ser_in),
		.bsy(tx_chr_bsy),
		.txline(s_out),
		.dbg_state(uart_state),
		.dbg_buf(uart_buf)
	);
	
	always@(negedge clk_in, negedge reset_n)
	begin
		if(reset_n == 0)
		begin
			bsy <= 0;
			p_out <= 0;
			state <= 0;
			tx_chr_start <= 0;
			mem_a <= 0;
			acntr <= 0;
		end
		else
		begin
			if( state )
			begin
				if( tx_chr_bsy == 0 )
				begin
					if(tx_chr_start == 0)
					begin
						case(state)
							1: p_out <= 4'hD;
							2: p_out <= 4'hA;
							3: p_out <= "c";
							4: p_out <= "=";
							5: p_out <= (dbg_istate < 10 ? (dbg_istate + "0") : (dbg_istate - 10 + "A"));
							6: p_out <= " ";
							7: p_out <= "r";
							8: p_out <= "0";
							9: p_out <= "=";
							10: p_out <= (dbg_r0 < 10 ? (dbg_r0 + "0") : (dbg_r0 - 10 + "A"));
							11: p_out <= " ";
							12: p_out <= "r";
							13: p_out <= "1";
							14: p_out <= "=";
							15: p_out <= (dbg_r1 < 10 ? (dbg_r1 + "0") : (dbg_r1 - 10 + "A"));
							16: p_out <= " ";
							17: p_out <= "p";
							18: p_out <= "c";
							19: p_out <= "=";
							20: p_out <= (dbg_pc < 10 ? (dbg_pc + "0") : (dbg_pc - 10 + "A"));
							21: p_out <= " ";
							22: p_out <= "A";
							23: p_out <= "=";
							24: p_out <= (dbg_mem_a < 10 ? (dbg_mem_a + "0") : (dbg_mem_a - 10 + "A"));
							25: p_out <= " ";
							26: p_out <= "R";
							27: p_out <= "=";
							28: p_out <= (dbg_mem_dr[7:4] < 10 ? (dbg_mem_dr[7:4] + "0") : (dbg_mem_dr[7:4] - 10 + "A"));
							29: p_out <= (dbg_mem_dr[3:0] < 10 ? (dbg_mem_dr[3:0] + "0") : (dbg_mem_dr[3:0] - 10 + "A"));
							30: p_out <= " ";
							31: p_out <= "W";
							32: p_out <= "=";
							33: p_out <= (dbg_mem_dw[7:4] < 10 ? (dbg_mem_dw[7:4] + "0") : (dbg_mem_dw[7:4] - 10 + "A"));
							34: p_out <= (dbg_mem_dw[3:0] < 10 ? (dbg_mem_dw[3:0] + "0") : (dbg_mem_dw[3:0] - 10 + "A"));
							35: p_out <= " ";
							36: p_out <= "M";
							37: p_out <= ":";
							38:	//to co tutaj wpisac do LAST_STATE
							begin
								p_out <= " ";
								mem_a <= 0;
								acntr <= 0;
							end
							
							default:
							begin
								case(acntr)
									0: p_out <= (mem_d[7:4] < 10 ? (mem_d[7:4] + "0") : (mem_d[7:4] - 10 + "A"));
									1: p_out <= (mem_d[3:0] < 10 ? (mem_d[3:0] + "0") : (mem_d[3:0] - 10 + "A")); 
									2: begin p_out <= " "; mem_a <= mem_a + 1; end
								endcase
								
								if(acntr < 2) acntr <= acntr + 1; else acntr <= 0;
							end
							
						endcase
						
						if( state <= LAST_STATE )
						begin
							state <= state + 1;
							tx_chr_start <= 1;
						end
						else
						begin
							state <= 0;
							bsy <= 0;
							p_out <= 0;
						end
						
					end	//if(tx_chr_start == 0)
				end
				else	//if( tx_chr_bsy == 0 )
				begin
					tx_chr_start <= 0;
				end
			end
			else	//if( state )
			begin
				if(startpulse)
				begin
					state <= 1;
					bsy <= 1;
				end
			end
		end
	end
	
endmodule
