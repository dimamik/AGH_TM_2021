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


module prescaler (
	input clkin,
	output reg clkout_a,
	output reg clkout_b,
	output reg clkout_c
);

parameter F_IN = 125000000;
parameter F_OUT_A = 1000000;
parameter F_OUT_B = 115200;
parameter F_OUT_C = 1;

reg [31:0] counter_a;
reg [31:0] counter_b;
reg [31:0] counter_c;

//section A

always@(posedge clkin)
	if( counter_a < ((F_IN) / F_OUT_A) )
		counter_a <= counter_a+1;
	else
		counter_a <= 0;

always@(posedge clkin)
	if( counter_a < (((F_IN) / F_OUT_A) / 2))
		clkout_a <= 0;
	else
		clkout_a <= 1;

//section B

always@(posedge clkin)
	if( counter_b < ((F_IN) / F_OUT_B) )
		counter_b <= counter_b+1;
	else
		counter_b <= 0;

always@(posedge clkin)
	if( counter_b < (((F_IN) / F_OUT_B) / 2))
		clkout_b <= 0;
	else
		clkout_b <= 1;

//section C

always@(posedge clkin)
	if( counter_c < ((F_IN) / F_OUT_C) )
		counter_c <= counter_c+1;
	else
		counter_c <= 0;

always@(posedge clkin)
	if( counter_c < (((F_IN) / F_OUT_C) / 2))
		clkout_c <= 0;
	else
		clkout_c <= 1;


endmodule
