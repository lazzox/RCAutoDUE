/* 
 * File:   uart.h
 * Author: Sirius-PC
 *
 * Created on February 6, 2017, 9:40 PM
 */

#ifndef UART_H
#define	UART_H


#include <pic16f689.h>

#define _XTAL_FREQ 20000000

char UART_Init(const long int baudrate)
{
	unsigned int x;
	x = (_XTAL_FREQ - baudrate*64)/(baudrate*64);
	if(x>255)
	{
		x = (_XTAL_FREQ - baudrate*16)/(baudrate*16);
		BRGH = 1;
	}
	if(x<256)
	{
        SPBRG = x;
        SYNC = 0;
        SPEN = 1;
        TRISB7 = 1;
        TRISB5 = 1;
        CREN = 1;
        TXEN = 1;
        RCIE = 1;
        return 1;
	}
	return 0;
}

char UART_Init115200()
{
    //@ 20 MHz
    BRGH = 1;
    SPBRG = 10;
    SYNC = 0;
    SPEN = 1;
    TRISB7 = 1;
    TRISB5 = 1;
    CREN = 1;
    TXEN = 1;
    RCIE = 1;
    PEIE = 1;
    return 1;
}

char UART_Init9600(){
    /* @ 4 mhz
    SPBRG = 25; // Calculated for baudrate of 9600 
    BRGH = 1;
    SYNC = 0;
    SPEN = 1; // SPEN = 1 -> Serial port enabled (configures RX/DT and TX/CK pins as serial port pins) 
    TRISB7 = 1;
    TRISB5 = 1;
    CREN = 1;
    TXEN = 1;
    
    RCIE = 1;
    PEIE = 1;
    */
    
    SPBRG = 10; // Calculated for baudrate of 9600 
    BRGH = 1;
    SYNC = 0;
    SPEN = 1; // SPEN = 1 -> Serial port enabled (configures RX/DT and TX/CK pins as serial port pins) 
    TRISB7 = 1;
    TRISB5 = 1;
    CREN = 1;
    TXEN = 1;
    
    RCIE = 1;
    PEIE = 1;
}

char UART_TX_Empty()
{
  return TRMT;
}

char UART_Data_Ready()
{
   return RCIF;
}
char UART_Read()
{
 
  while(!RCIF);
  return RCREG;
}

void UART_Read_Text(char *Output, unsigned int length)
{
	unsigned int i;
	for(int i=0;i<length;i++)
		Output[i] = UART_Read();
}

void UART_Write(char data)
{
  while(!TRMT);
  TXREG = data;
}

void UART_Write_Text(char *text)
{
  int i;
  for(i=0;text[i]!='\0';i++)
	  UART_Write(text[i]);
}


#endif	/* UART_H */

