/*
 * File:   sequencer.c
 * Author: Luke
 *
 * Created on 24. Januar 2017, 00:41
 */


#include <xc.h>
#include <stdint.h>


// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)




#define _XTAL_FREQ 8000000

void main(void) 
{
    uint8_t cntr;
    
    
    
    TRISAbits.TRISA4 = 0;
    PORTAbits.RA4 = 1; //PTT LED on
    
    TRISAbits.TRISA5 = 0;
    PORTAbits.RA5 = 1; //PWR LED on
    
    PORTA=0xFF;
    
    PORTC = 0x00;
    TRISCbits.TRISC5 = 0; //Relay 1
    TRISCbits.TRISC4 = 0; //Relay 2
    TRISCbits.TRISC3 = 0; //Relay 3
    /*
    while(1)
    {
        PORTC = cntr;
        ++cntr;
        __delay_ms(500);
    }
    */
    
    return;
}
