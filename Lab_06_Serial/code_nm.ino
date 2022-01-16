#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "lcd.h" // include LCD library

// TODO Fix BAUD rate to that in the task
// #define BAUD 9600                              // define baud
#define BAUD 2400
#define F_CPU 16000000UL                       // set the CPU clock
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1) // set baudrate value for UBRR

// function to initialize UART
void uart_init(void)
{
    UBRRH = (BAUDRATE >> 8);
    UBRRL = BAUDRATE;                                    // set baud rate
    UCSRB |= (1 << TXEN) | (1 << RXEN);                  // enable receiver and transmitter

    // TODO Set the frame format to 8 data bits, no parity, 1 stop bit ?? This is not for sure :)
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // 8bit data format
}


// function to send data - NOT REQUIRED FOR THIS PROGRAM IMPLEMENTATION
void uart_transmit(unsigned char data)
{
    while (!(UCSRA & (1 << UDRE)))
        ;       // wait while register is free
    UDR = data; // load data in the register
}

// function to receive data
unsigned char uart_recieve(void)
{
    while (!(UCSRA) & (1 << RXC))
        ;       // wait while data is being received
    return UDR; // return 8-bit data
}

// main function: entry point of program

setup()
{

    uart_init(); // initialize UART
}

// TODO For 3.5
// This part is a little bit tricky,
// But we need by hand to set digitalWrite to LOW and High to transmit the data
// Instead of using 1000 ms we need to use BAUDRATE for this concrete implementation
// setup_v2(){
//     pinMode(4, OUTPUT);
// }

// loop_v2(){
//     digitalWrite(4, HIGH); // sets the digital pin 13 on
//     delay(1000);            // waits for a second
//     digitalWrite(4, LOW);  // sets the digital pin 13 off
//     delay(1000);            // waits for a second
// }

loop()
{
    // TODO There could fail, play with the data to send
    uart_transmit('5');
    _delay_ms(100); // wait before next attempt
}
