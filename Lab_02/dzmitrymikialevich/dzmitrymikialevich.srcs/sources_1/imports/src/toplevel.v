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


module toplevel(
	input CLK_IN,
	output [3:0] LD,
	input [3:0] SW,
	input [5:0] BTN,
	output [7:0] je
	);

wire clk_ser;	//should be 115200 Hz
wire clk_main;
wire clk_slow;
wire reset_n;
wire ser_out;

wire btn1_d;
wire btn2_d;
wire btn3_d;

wire [7:0] mem_port_b_rd;
wire [3:0] mem_port_b_addr;
wire [7:0] mem_port_a_rd;
wire [7:0] mem_port_a_wr;
wire [3:0] mem_port_a_addr;
wire mem_port_a_we;

wire clk_manual;
wire clk_cycle;
wire clk_cpu;
wire dbg_trig;
wire dbg_tx_bsy;

wire [2:0] dbg_state;
wire [3:0] dbg_r0;
wire [3:0] dbg_r1;
wire [3:0] dbg_pc;



assign je[7] = ser_out;
assign je[6] = clk_ser;
assign je[5] = clk_main;

assign reset_n = ~BTN[0];

prescaler p1 (
	.clkin(CLK_IN),
	.clkout_a(clk_main),
	.clkout_b(clk_ser),
	.clkout_c(clk_slow)
);

debouncer d1 (.clk(clk_main), .in(BTN[1]), .out(btn1_d));

debouncer d2 (.clk(clk_main),.in(BTN[2]),.out(btn2_d));

debouncer d3 (.clk(clk_main),.in(BTN[3]),.out(btn3_d));

clocknx pgen1(
	.reset_n(reset_n),
	.clk(clk_main),
	.in(btn2_d),
	.out(clk_cycle)
);

assign clk_manual = (SW[0]) ? btn2_d : clk_cycle;	//reczny generator zegara: po 1 cykl / po 4 cykle
assign clk_cpu = (SW[1]) ? clk_slow : clk_manual;	//taktowanie CPU: auto / manual
assign dbg_trig = (SW[2]) ? clk_cpu : btn1_d;		//wyswietlenie debug info: auto / na BTN1

assign LD[0] = clk_cpu;
assign LD[1] = dbg_tx_bsy;
assign LD[2] = btn2_d;
assign LD[3] = 0;


mem_wrapper mem1(
	.BRAM_PORTA_addr(mem_port_a_addr),
	.BRAM_PORTA_clk(~clk_cpu),
	.BRAM_PORTA_din(mem_port_a_wr),
	.BRAM_PORTA_dout(mem_port_a_rd),
	.BRAM_PORTA_we(mem_port_a_we),
	
	.BRAM_PORTB_addr(mem_port_b_addr),
	.BRAM_PORTB_clk(clk_main),
	.BRAM_PORTB_din(0),
	.BRAM_PORTB_dout(mem_port_b_rd),
	.BRAM_PORTB_we(0)
);


cpu cpu1(
	.clk(clk_cpu),
	.reset_n(reset_n),
	
	//interfejs pamieci
	.mem_address(mem_port_a_addr),
	.mem_data_r(mem_port_a_rd),
	.mem_data_w(mem_port_a_wr),
	.mem_we(mem_port_a_we),
	
	//interfejs do debuggera
	.dbg_state(dbg_state),
	.dbg_r0(dbg_r0),
	.dbg_r1(dbg_r1),
	.dbg_pc(dbg_pc)
);


dbgu dbgu1(
		.clk_in(clk_main),
		.clk_ser_in(clk_ser),
	    .reset_n(reset_n),
	    .trig(dbg_trig),
	    .s_out(ser_out),
	    .bsy(dbg_tx_bsy),
	    .p_out(),
	    .dbg_r0(dbg_r0),
	    .dbg_r1(dbg_r1),
	    .dbg_pc(dbg_pc),
	    .dbg_istate(dbg_state),
	    .dbg_mem_a(mem_port_a_addr),
	    .dbg_mem_dr(mem_port_a_rd),
	    .dbg_mem_dw(mem_port_a_wr),
		
	    .mem_a(mem_port_b_addr),
	    .mem_d(mem_port_b_rd),
	    .mem_clk()
	);
	
	
endmodule
