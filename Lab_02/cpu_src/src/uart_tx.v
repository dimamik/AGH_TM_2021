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


module uart_tx(
	input start,
	input [7:0] data,
	input bitclk,
	input reset_n,
	output reg bsy,
	output reg txline,
	output [7:0] dbg_buf,
	output [4:0] dbg_state
	);
	
	reg [4:0] state;
	reg [8:0] txbuf;
	
	assign dbg_buf = txbuf;
	assign dbg_state = state;
	
	always@(posedge bitclk, negedge reset_n)
	begin
		if(reset_n==0)
		begin
			txbuf <= 0;
			bsy <= 0;
			txline <= 1;
			state <= 0;
		end
		else
		begin
			if(start && (bsy==0))
			begin
				txbuf[7:0] <= data[7:0];
				state <= 1;
				txline <= 0;
				bsy <= 1;
			end
			
			if( state > 0 )
			begin
				if(state < 9)
				begin
					state <= state + 1;
					txline <= txbuf[0];
					txbuf[7:0] <= {1'b0, txbuf[7:1]};
				end
				else
				begin
					case(state)
						9:
						begin
							state <= 10;
							txline <= 1;
						end
						
						10: state <= 11;
						
						11: state <= 12;
						
						12: state <= 13;
						
						13:
						begin
							state <= 0;
							bsy <= 0;
						end
					endcase
				end
			end
			
		end
	end
	
endmodule
