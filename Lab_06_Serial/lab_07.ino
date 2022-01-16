#include "avr/io.h"

#define FOSC 1843200 // Clock Speed
#define BAUD 9600
#define BAUD 2400
#define MYUBRR (FOSC / 16 / BAUD - 1)
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1)

void main(void)
{
  // TODO Our code there
  USART_Init(MYUBRR);
} // main

// 5 to 8 Data Bits

void USART_Init(unsigned int ubrr)
{
  /* Set baud rate */
  UBRRH = (unsigned char)(ubrr >> 8);
  UBRRL = (unsigned char)ubrr;
  /* Enable receiver and transmitter */
  UCSRB = (1 << RXEN) | (1 << TXEN);
  /* Set frame format: 8data, 2stop bit */
  // UCSRC = (1 << USBS) | (3 << UCSZ0);
  UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
} // USART_Init

void USART_Transmit_v1(unsigned char data)
{
  /* Wait for empty transmit buffer */
  while (!(UCSRnA & (1 << UDREn)))
    ;
  /* Put data into buffer, sends the data */
  UDRn = data;
}
unsigned char USART_Receive_v1(void)
{
  /* Wait for data to be received */
  while (!(UCSRnA & (1 << RXCn)))
    ;
  /* Get and return received data from buffer */
  return UDRn;
}

// This is for 9 Data bits

void USART_Transmit_v2(unsigned int data)
{
  /* Wait for empty transmit buffer */
  while (!(UCSRnA & (1 << UDREn))) )
;
  /* Copy 9th bit to TXB8 */
  UCSRnB &= ~(1 << TXB8);
  if (data & 0x0100)
    UCSRnB |= (1 << TXB8);
  /* Put data into buffer, sends the data */
  UDRn = data;
}

unsigned int USART_Receive_v2(void)
{
  unsigned char status, resh, resl;
  /* Wait for data to be received */
  while (!(UCSRnA & (1 << RXCn)))
    ;
  /* Get status and 9th bit, then data */
  /* from buffer */
  status = UCSRnA;
  resh = UCSRnB;
  resl = UDRn;
  /* If error, return -1 */
  if (status & (1 << FEn) | (1 << DORn) | (1 << UPEn))
    return -1;
  /* Filter the 9th bit, then return */
  resh = (resh >> 1) & 0x01;
  return ((resh << 8) | resl);
}

// This is for all
void USART_Flush(void)
{
  unsigned char dummy;
  while (UCSRnA & (1 << RXCn))
    dummy = UDRn;
}

void setup()
{
}

void loop()
{
}
