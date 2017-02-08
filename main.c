/*
 * File:   main.c
 * Author: Sirius-PC
 *
 * Created on May 24, 2016, 10:04 PM
 */

#include <pic16f689.h>
#include <stdint.h>
#include <xc.h>
#include "uart.h"

#define DEBUG_ON

// BEGIN CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON      // Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
//END CONFIG

/* Define XTAL_FREQ in uart.h */

#define ENA PORTCbits.RC1
#define ENB PORTCbits.RC6
#define IN1 PORTCbits.RC0
#define IN2 PORTCbits.RC2
#define IN3 PORTCbits.RC7
#define IN4 PORTCbits.RC3
#define LED1 PORTAbits.RA2


#define TMR0_RLDVAL 56

uint16_t i = 0;
uint8_t newData = 0;
uint8_t receivedData[4] = '0';
/** receivedData bytes meaning
 * 
 * [0] - S (beginning of transmision)
 * [1] - commands [1/2]
 *      [bit 4] - movement [0 - OFF; 1 - ON]  
 *      [bit 0] - way [0 - FORWARD; 1 - BACKWARD]
 * [2] - commands [2/2]
 *      [bit 4] - stearing [0 - NO; 1 - YES]  
 *      [bit 0] - side [0 - LEFT; 1 - RIGHT]
 * [3] - T (termination of transmision)
 * 
 * Example of moving forward and stearing right:
 * 
 * [0] - 'S'
 * [1] - 0x10 (hexadecimal)
 * [2] - 0x11 (hexadecimal)
 * [3] - 'T'
 * 
 **/

void interrupt   tc_int  (void){        // interrupt function 
    if (T0IF){ /* 800 uS interrupt */
        /* End of PWM period - reload TMR0 & TMR1 */
        T0IF = 0;
        TMR0 = TMR0_RLDVAL;
        
        /* Reload TMR1 */
        TMR1IF = 0;
        TMR1H = 0xFf;
        TMR1L = 0x38;
        TMR1ON = 1;
        //i++;
        ENA = 1;
        //ENB = 1;
    }
    else if (TMR1IF){ /* 400 uS interrupt */
        TMR1IF = 0;
        TMR1ON = 0;
        
        /* End of duty cycle */
        ENA = 0;
        //ENB = 0;
    } else if(RCIF){
        i = 0;
        while(RCIF)
        {
           receivedData[i] = RCREG; 
           i++;
        }
        newData = 1;
        UART_Write(i);
        i=0;
    }
    
}

void setHBridge(){
    if(receivedData[0] == 'S' && receivedData[3] == 'T')
    {
        if((receivedData[1] >> 4) ) {
            /* There is movement */
            ENA = 0;
            if(receivedData[1] && 0x01){ 
                /* Move forward */ 
                IN1 = 0;
                IN2 = 1;
            }else {
                /* Move backwards */
                IN1 = 0;
                IN2 = 1;
            }
            ENA = 1;
        }else {
            /* There is no movement */
            ENA = 0;
        }
        
         if((receivedData[2] >> 4) ) {
            /* There is stearing */
            ENB = 0;
            if(receivedData[1] && 0x01){ 
                /* Stear left */ 
                IN3 = 0;
                IN4 = 1;
            }else {
                /* Stear right */
                IN3 = 0;
                IN4 = 1;
            }
            ENB = 1;
        }else {
            /* There is no movement */
            ENB = 0;
        }
        
        
    }
#ifdef DEBUG_ON
    else {
        /* DATA RECEIVED NOT GOOD SEND DATA BACK */
        UART_Write_Text("ERROR1");
    }
#endif
}

//Timer1
//Prescaler 1:1; TMR1 Preload = 65036; Actual Interrupt Time : 500 us
void InitTimer1(){
  T1CON = 0x01;
  TMR1IF = 0;
  TMR1H	 = 0xFF;
  TMR1L	 = 0x38;
  TMR1IE = 1;
  INTCON |= 0xC0;
}


//Timer0
//Prescaler 1:4; TMR0 Preload = 56; Actual Interrupt Time : 800 us
void InitTimer0(){
  OPTION_REG = 0x81;
  TMR0 = TMR0_RLDVAL;
  INTCON = 0xA0;
}
 


void main(void) {
    
    //************SET PINS DIGITAL/AD-OFF**************
    ANSEL = 0b00000000; //All I/O pins are configured as digital
   // ANSELH = 0;//
    ADCON0bits.ADON = 0;  //disable ADCON
    
    //************SET PINS AS OUTPUT/INPUT**************
    TRISA = 0x00;   //PORTA All Outputs
    TRISB = 0x00;   //PORTB ALL Outputs
    TRISC = 0x00;   //PORTC All Outputs
  
    PORTA = 0x00;   //PORTA = 1;
    PORTB = 0x00;   //PORTB = 1;
    PORTC = 0x00;   //PORTC = 1;
    LED1 = 1;
    __delay_ms(5000);
    LED1 = 0;
    //************INIT TIMER1**************
    InitTimer0();
    InitTimer1();
    
    //T1CON	 = 0x01;
    //PIR1bits.TMR1IF = 0;
    //TMR1H	 = 0x3C;
    //TMR1L	 = 0xB0;
    
    //PIE1bits.TMR1IE = 1; /* Enable Timer1 Interrupt */
    //INTCONbits.PEIE = 1;
   // GIE = 1; //Global interrupt Enable
    
    /************INIT UART****************/
    //UART_Init(9600);
  
    GIE = 1; /* Global interrupt enabled */
    /************TEST AREA***************/
    
    ENA = 0;
    ENB = 0;
    IN1 = 1;
    IN2 = 0;
    IN3 = 1;
    IN4 = 0;
  
    while(1){
      
    }
    
    while(0){
        if(i>=5000){
            i =0;
            LED1 = ~LED1;
        }
    }
    
    while(1){
        ENA = 1;
        ENB = 1;
        __delay_us(400);
        ENA = 0;
        ENB = 1;
        __delay_us(400);
    }
    

    while(1){
        if(newData){
            setHBridge();
            newData = 0;
        }
    } 
}
