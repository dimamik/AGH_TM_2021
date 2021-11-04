//Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2016.4 (lin64) Build 1756540 Mon Jan 23 19:11:19 MST 2017
//Date        : Thu Nov  4 11:53:11 2021
//Host        : UTCLAB-L00 running 64-bit Linux Mint 19 Tara
//Command     : generate_target mem_wrapper.bd
//Design      : mem_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module mem_wrapper
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

  wire [3:0]BRAM_PORTA_addr;
  wire BRAM_PORTA_clk;
  wire [7:0]BRAM_PORTA_din;
  wire [7:0]BRAM_PORTA_dout;
  wire [0:0]BRAM_PORTA_we;
  wire [3:0]BRAM_PORTB_addr;
  wire BRAM_PORTB_clk;
  wire [7:0]BRAM_PORTB_din;
  wire [7:0]BRAM_PORTB_dout;
  wire [0:0]BRAM_PORTB_we;

  mem mem_i
       (.BRAM_PORTA_addr(BRAM_PORTA_addr),
        .BRAM_PORTA_clk(BRAM_PORTA_clk),
        .BRAM_PORTA_din(BRAM_PORTA_din),
        .BRAM_PORTA_dout(BRAM_PORTA_dout),
        .BRAM_PORTA_we(BRAM_PORTA_we),
        .BRAM_PORTB_addr(BRAM_PORTB_addr),
        .BRAM_PORTB_clk(BRAM_PORTB_clk),
        .BRAM_PORTB_din(BRAM_PORTB_din),
        .BRAM_PORTB_dout(BRAM_PORTB_dout),
        .BRAM_PORTB_we(BRAM_PORTB_we));
endmodule
