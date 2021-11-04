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


module pulsegen(
    input clk,
    input reset_n,
    input trig,
    output reg pulse
    );
    
	reg [32:0] cntr;
	reg prevstate;
    
    always@(posedge clk,negedge reset_n)
    if(reset_n == 0)
    begin
    	cntr <= 0;
    	pulse <= 0;
    end
    else
    begin
    	prevstate <= trig;
    	
    	if( (prevstate==0) && trig )
    	begin
    		pulse <= 1;
    		cntr <= 0;
    	end
    	else
    	begin
    		if(cntr < 100) cntr <= cntr + 1;
    		else
    		begin
    			cntr <= 0;
    			pulse <= 0;
    		end
    	end
    end
    
    
endmodule
