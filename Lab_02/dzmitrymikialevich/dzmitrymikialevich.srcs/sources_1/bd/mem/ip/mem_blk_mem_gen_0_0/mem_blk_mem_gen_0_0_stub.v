// Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2016.4 (lin64) Build 1756540 Mon Jan 23 19:11:19 MST 2017
// Date        : Thu Nov  4 11:54:01 2021
// Host        : UTCLAB-L00 running 64-bit Linux Mint 19 Tara
// Command     : write_verilog -force -mode synth_stub
//               /home/student/Dzmitry_Mikialevich/dzmitrymikialevich/dzmitrymikialevich.srcs/sources_1/bd/mem/ip/mem_blk_mem_gen_0_0/mem_blk_mem_gen_0_0_stub.v
// Design      : mem_blk_mem_gen_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z010clg400-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "blk_mem_gen_v8_3_5,Vivado 2016.4" *)
module mem_blk_mem_gen_0_0(clka, wea, addra, dina, douta, clkb, web, addrb, dinb, 
  doutb)
/* synthesis syn_black_box black_box_pad_pin="clka,wea[0:0],addra[3:0],dina[7:0],douta[7:0],clkb,web[0:0],addrb[3:0],dinb[7:0],doutb[7:0]" */;
  input clka;
  input [0:0]wea;
  input [3:0]addra;
  input [7:0]dina;
  output [7:0]douta;
  input clkb;
  input [0:0]web;
  input [3:0]addrb;
  input [7:0]dinb;
  output [7:0]doutb;
endmodule
