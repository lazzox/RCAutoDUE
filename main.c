/*
 * File:   main.c
 * Author: L. Vukasovic
 *
 * Created on May 24, 2016, 10:04 PM
 */

//#include <pic16f689.h>
#include <stdint.h>
#include <xc.h>
#include "uart.h"

//#define DEBUG_ON

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

// Timer0 - 10ms
#define TMR0_RLDVAL 61


// Timer1 //0x5038 - 9ms //0x63C0 - 8 ms  //0x8ad0 - 6 ms //0xB1E0 - 4MS
#define TMR1_RLDVAL_HI 0x8a
#define TMR1_RLDVAL_LO 0xd0

static volatile uint16_t i = 0;
static volatile uint16_t j = 0;
static volatile uint8_t tmpChar = 0;
static volatile uint8_t newData = 0;
static volatile uint8_t receivedData[21] = '0';
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
    if (T0IF){ /* 10 ms interrupt */
        /* End of PWM period - reload TMR0 & TMR1 */
        T0IF = 0;
        TMR0 = TMR0_RLDVAL;
        
        
        /* Reload TMR1 */
        TMR1IF = 0;
        TMR1H = TMR1_RLDVAL_HI;
        TMR1L = TMR1_RLDVAL_LO;
        TMR1ON = 1;
        i++;
        ENA = 1;
        
    }
    else if (TMR1IF){ /* 6 ms interrupt */
        TMR1IF = 0;
        TMR1ON = 0;
        
        /* End of duty cycle */
        ENA = 0;
       
    } else if(RCIF){
        while(RCIF){
            tmpChar = RCREG; 

            if(tmpChar == 'S' && newData == 0){
                j = 1;
                receivedData[0] = tmpChar;
            }else if(tmpChar == 'T' && j == 3){
                receivedData[3] = tmpChar;
                j=0;
                i=0;
                newData=15; /* Reduced the command interval for 150 ms*/
            }
            else{
                receivedData[j] = RCREG;
                j++;
            }
             /* Protection if junk is received */
            if (j > 4) j = 0;
        }
    }
    
    /* Protection if nothing is received for 15 * 10ms
     * stop the motors */
    if(i > 15) {
        i = 0;
        receivedData[0] = 'S';
        receivedData[1] = 0x00;
        receivedData[2] = 0x00;
        receivedData[3] = 'T';
        newData=1;
        
    }
           
}

void setHBridge(){
    if(receivedData[0] == 'S' && receivedData[3] == 'T')
    {
        if((receivedData[1] >> 4) ) {
            /* There is movement */
           
            if(receivedData[1] & 0x01){ 
                /* Move forward */ 
                #ifdef DEBUG_ON
                UART_Write('F');
                #endif
                //ENA = 0;
                IN1 = 0;
                IN2 = 1;
               // ENA = 1;
            }else {
                /* Move backwards */
                #ifdef DEBUG_ON
                UART_Write('B');
                #endif
                //ENA = 0;
                IN1 = 1;
                IN2 = 0;
               // ENA = 1;
            }
           
        }else {
            /* There is no movement */
            IN1 = 0;
            IN2 = 0;
            ENA = 0;
        }
        
         if((receivedData[2] >> 4) ) {
            /* There is stearing */
            ENB = 0;
            if(receivedData[2] & 0x01){ 
                /* Stear left */
                #ifdef DEBUG_ON
                UART_Write('L');
                #endif
                IN3 = 0;
                IN4 = 1;
            }else {
                /* Stear right */
                #ifdef DEBUG_ON
                UART_Write('R');
                #endif
                IN3 = 1;
                IN4 = 0;
            }
            ENB = 1;
        }else {
            /* There is no movement */
            IN3 = 0;
            IN4 = 0;
            ENB = 0;
        }
        
        // Detect OK+LOST or OK+CONNECT
    }else if(receivedData[0] == 'O' && receivedData[1] == 'K'){
        if(receivedData[3] == 'L'){
            //OK+LOST received
            receivedData[0] = 'S';
            receivedData[1] = 0x00;
            receivedData[2] = 0x00;
            receivedData[3] = 'T';
            newData=1;
            LED1 = 0;
        }
        else if(receivedData[3] == 'C'){
            //OK+CONN received
            LED1 = 1;
            
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
  TMR1H	 = TMR1_RLDVAL_HI;
  TMR1L	 = TMR1_RLDVAL_LO;
  TMR1IE = 1;
  INTCON |= 0xC0;
}


//Timer0
//Prescaler 1:256; TMR0 Preload = 61; Actual Interrupt Time : 9.984 ms
void InitTimer0(){
  OPTION_REG = 0x87;
  TMR0 = TMR0_RLDVAL;
  INTCON |= 0xA0;
}
 


void main(void) {
    
    //************SET PINS DIGITAL/AD-OFF**************
    ANSEL = 0b00000000; //All I/O pins are configured as digital
    ANSELH = 0;//
    ADCON0bits.ADON = 0;  //disable ADCON
    
    //************SET PINS AS OUTPUT/INPUT**************
    TRISA = 0x00;   //PORTA All Outputs
    TRISB = 0x00;   //PORTB ALL Outputs
    TRISC = 0x00;   //PORTC All Outputs
  
    PORTA = 0x00;   //PORTA = 1;
    PORTB = 0x00;   //PORTB = 1;
    PORTC = 0x00;   //PORTC = 1;
    LED1 = 1;
    __delay_ms(100);
    LED1 = 0;
    //************INIT TIMER1**************
    InitTimer0();
    InitTimer1();
  
    /************INIT UART****************/
    UART_Init115200();
    GIE = 1; /* Global interrupt enabled */
    /************TEST AREA***************/
    
    ENA = 0;
    ENB = 0;
    IN3 = 0;
    IN4 = 0;       
   
     while(1){
         if(newData){ /* Received new command */
#ifdef DEBUG_ON
           // UART_Write(receivedData[0]);
           // UART_Write(receivedData[1]);
            //UART_Write(receivedData[2]);
            //UART_Write(receivedData[3]);
            //UART_Write(receivedData[4]);
#endif
            setHBridge();
            if(!newData) newData--;
         }
     }
}
