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

module cpu (
	//wejscia kontrolne
	input clk,
	input reset_n,
	
	//interfejs pamieci
	output reg [3:0] mem_address,
	input [7:0] mem_data_r,
	output reg [7:0] mem_data_w,
	output reg mem_we,
	
	//interfejs do debugowania
	output [1:0] dbg_state,
	output [3:0] dbg_r0,
	output [3:0] dbg_r1,
	output [3:0] dbg_pc
);


reg [1:0] state;	//przechowuje aktualny stan mikroprocesora
reg [3:0] pc;		//rejestr PC - licznik programu
reg [7:0] opcode;	//rejestr do tymczasowego przechowywania kodu instrukcji pobranego z pamieci

reg [3:0] r0;		//rejestr r0
reg [3:0] r1;		//rejestr r1

//przypisania zapewniajace podglad wewnetrznych sygnalow
assign dbg_state = state;
assign dbg_r0 = r0;
assign dbg_r1 = r1;

assign dbg_pc = pc;


//glowna maszyna stanow mikroprocesora:

always@(posedge clk, negedge reset_n)
if(reset_n == 0)	//po wykryciu aktywnosci na sygnale reset_n...
begin
	state <= 0;		//zerujemy podstawowe rejestry mikroprocesora
	pc <= 0;
	r0 <= 0;
	r1 <= 0;
	mem_we <= 0;	//zerujemy sygnaly zezwolenia na zapis do pamieci operacyjnej
end
else
begin
	case(state)
		0:
		begin
			state <= 1;			//przechodzimy do nastepnego stanu
			mem_we <= 0;		//wylaczamy sygnal zezwolenia na zapis do pamieci
								//(jesli byl uaktywniony w poprzednim cyklu, a jesli nie, to pozostaje wyzerowany)
			mem_address <= pc;	//na magistrali adresowej wystawiamy adres rozkazu do pobrania
		end
		
		1:
		begin
			state <= 2;
			opcode <= mem_data_r;	//odczytana dana z pamieci traktujemy jako kod operacji
			pc <= pc + 1;			//zwiekszamy zawartosc PC o 1
		end
		
		2:
		begin
			state <= 3;
			if(opcode[7]==0)					//jesli bit 7 ma wartosc 0, to operacja arytmetyczna lub skok
			begin
				case(opcode[6:4])				//analizujemy bity 6, 5 i 4...
					3'b000:						//jesli wszystkie sa ustawione na 000...
					begin
												//wykonujemy skok pod adres dany stala natychmiastowa
						pc <= opcode[3:0];		//zaladuj do PC wartosc znaleziona w bitach 3:0 kodu instrukcji
					end
					
					3'b010:						//jesli bity 6:4 maja wartosc 010...
					begin
						r0 <= r0 + r1;			//wykonujemy dodawanie: add r0, r1 (r0 = r0 + r1)
					end
				endcase
			end
			else									//gdy bit 7 != 0, to kod operacji stanowi instrukcje load, store lub move
			begin
				if(opcode[6])						//bit 6 == 1 -> instrukcja load/store
				begin
					mem_address <= opcode[3:0];		//...wtedy adres znajduje sie w bitach 3:0
					if(opcode[5])					//jesli bit 5 jest ustawiony, to mamy do czynienia z zapisem (store)
					begin
						if(opcode[4])				//bitem 4 wybieramy, ktory rejestr zapisac (r0 gdy 0 lub r1 gdy 1)
							mem_data_w <= r1;		//wystawiamy na magistrale zapisu do pamieci zawartosc r1
						else
							mem_data_w <= r0;		//wystawiamy na magistrale zapisu do pamieci zawartosc r0
					end
				end
				else								//jesli bit 6 ma wartosc 0...
				begin								//to mamy do czynienia z instrukcjami wpisywania stalej natychmiastowej lub move
					case(opcode[5:4])				//analizujemy bity 5 i 4
						2'b00: r0 <= opcode[3:0];	//imm to r0 (stala natychmiastowa do r0)
						2'b01: r1 <= opcode[3:0];	//imm to r1 (stala natychmiastowa do r1)
						2'b10: r0 <= r1;			//mov r0, r1 (kopiowanie r0 = r1)
						2'b11: r1 <= r0;			//mov r1, r0 (kopiowanie r1 = r0)	
					endcase
				end
			end
		end
		
		3:			//kontyuacja operacji zapisu lub odczytu
		begin
			state <= 0;
			//obsluga zapisu do pamieci
			if(opcode[7:5]==3'b111)
				mem_we <= 1;			//wystawiamy aktywny sygnal zezwolenia na zapis do pamieci (mem_address byl ustawiony wczesniej)
			
			//obsluga odczytu z pamieci
			if(opcode[7:5]==3'b110)		//jesli byla operacja odczytu
			begin
				if(opcode[4]==0)		//wybieramy rejestr
					r0 <= mem_data_r;	//wpisujemy odczytana z pamieci liczbe do rejestru r0
				else
					r1 <= mem_data_r;	//wpisujemy odczytana z pamieci liczbe do rejestru r1
			end
		end
		
	endcase
end

endmodule

