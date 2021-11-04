//Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2016.4 (lin64) Build 1756540 Mon Jan 23 19:11:19 MST 2017
//Date        : Thu Nov  4 11:53:11 2021
//Host        : UTCLAB-L00 running 64-bit Linux Mint 19 Tara
//Command     : generate_target mem.bd
//Design      : mem
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CORE_GENERATION_INFO = "mem,IP_Integrator,{x_ipVendor=xilinx.com,x_ipLibrary=BlockDiagram,x_ipName=mem,x_ipVersion=1.00.a,x_ipLanguage=VERILOG,numBlks=1,numReposBlks=1,numNonXlnxBlks=0,numHierBlks=0,maxHierDepth=0,numSysgenBlks=0,numHlsBlks=0,numHdlrefBlks=0,numPkgbdBlks=0,bdsource=USER,synth_mode=OOC_per_IP}" *) (* HW_HANDOFF = "mem.hwdef" *) 
module mem
   (BRAM_PORTA_addr,
    BRAM_PORTA_clk,
    BRAM_PORTA_din,
    BRAM_PORTA_dout,
    BRAM_PORTA_we,
    BRAM_PORTB_addr,
    BRAM_PORTB_clk,
    BRAM_PORTB_din,
    BRAM_PORTB_dout,
    BRAM_PORTB_we);
  input [3:0]BRAM_PORTA_addr;
  input BRAM_PORTA_clk;
  input [7:0]BRAM_PORTA_din;
  output [7:0]BRAM_PORTA_dout;
  input [0:0]BRAM_PORTA_we;
  input [3:0]BRAM_PORTB_addr;
  input BRAM_PORTB_clk;
  input [7:0]BRAM_PORTB_din;
  output [7:0]BRAM_PORTB_dout;
  input [0:0]BRAM_PORTB_we;

  wire [3:0]BRAM_PORTA_1_ADDR;
  wire BRAM_PORTA_1_CLK;
  wire [7:0]BRAM_PORTA_1_DIN;
  wire [7:0]BRAM_PORTA_1_DOUT;
  wire [0:0]BRAM_PORTA_1_WE;
  wire [3:0]BRAM_PORTB_1_ADDR;
  wire BRAM_PORTB_1_CLK;
  wire [7:0]BRAM_PORTB_1_DIN;
  wire [7:0]BRAM_PORTB_1_DOUT;
  wire [0:0]BRAM_PORTB_1_WE;

  assign BRAM_PORTA_1_ADDR = BRAM_PORTA_addr[3:0];
  assign BRAM_PORTA_1_CLK = BRAM_PORTA_clk;
  assign BRAM_PORTA_1_DIN = BRAM_PORTA_din[7:0];
  assign BRAM_PORTA_1_WE = BRAM_PORTA_we[0];
  assign BRAM_PORTA_dout[7:0] = BRAM_PORTA_1_DOUT;
  assign BRAM_PORTB_1_ADDR = BRAM_PORTB_addr[3:0];
  assign BRAM_PORTB_1_CLK = BRAM_PORTB_clk;
  assign BRAM_PORTB_1_DIN = BRAM_PORTB_din[7:0];
  assign BRAM_PORTB_1_WE = BRAM_PORTB_we[0];
  assign BRAM_PORTB_dout[7:0] = BRAM_PORTB_1_DOUT;
  mem_blk_mem_gen_0_0 blk_mem_gen_0
       (.addra(BRAM_PORTA_1_ADDR),
        .addrb(BRAM_PORTB_1_ADDR),
        .clka(BRAM_PORTA_1_CLK),
        .clkb(BRAM_PORTB_1_CLK),
        .dina(BRAM_PORTA_1_DIN),
        .dinb(BRAM_PORTB_1_DIN),
        .douta(BRAM_PORTA_1_DOUT),
        .doutb(BRAM_PORTB_1_DOUT),
        .wea(BRAM_PORTA_1_WE),
        .web(BRAM_PORTB_1_WE));
endmodule
