/*
 * File:   sequencer.c
 * Author: Luke
 *
 * Created on 24. Januar 2017, 00:41
 */


#include <xc.h>
#include <stdint.h>

// PIC16F18325 Configuration Bit Settings
// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = ON        // Watchdog Timer Enable bits (WDT enabled, SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = HIGH      // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.7V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = ALL        // User NVM self-write protection bits (0000h to 1FFFh write protected, no addresses may be modified)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = ON          // User NVM Program Memory Code Protection bit (User NVM code protection enabled)
#pragma config CPD = ON         // Data NVM Memory Code Protection bit (Data NVM code protection enabled)

#define _XTAL_FREQ 1000000
#define ADC_VALUE_STARTUP 890
#define STARTUP_COUNT 8

#define MINIMUM_DELAY 70
#define AVERAGE_COUNT 32
#define AVERAGE_SHIFT 7

#define PPTLED_TRIS TRISAbits.TRISA4
#define PPTLED_MASK 0b00010000
#define PPTLED_PORT PORTA
#define PPTLED_VARIABLE portA

#define PWRLED_TRIS TRISAbits.TRISA5
#define PWRLED_MASK 0b00100000
#define PWRLED_PORT PORTA
#define PWRLED_VARIABLE portA

#define RELAY1_TRIS TRISCbits.TRISC5
#define RELAY1_MASK 0b00100000
#define RELAY1_PORT PORTC
#define RELAY1_VARIABLE portC

#define RELAY2_TRIS TRISCbits.TRISC4
#define RELAY2_MASK 0b00010000
#define RELAY2_PORT PORTC
#define RELAY2_VARIABLE portC

#define RELAY3_TRIS TRISCbits.TRISC3
#define RELAY3_MASK 0b00001000
#define RELAY3_PORT PORTC
#define RELAY3_VARIABLE portC

#define PPTSENSE_TRIS TRISCbits.TRISC1
#define PPTSENSE_ANSEL ANSELCbits.ANSC1
#define PPTSENSE_PIN PORTCbits.RC1

#define VINSENSE_TRIS TRISCbits.TRISC0
#define VINSENSE_ANSEL ANSELCbits.ANSC0

#define SPEEDSENSE_TRIS TRISAbits.TRISA2
#define SPEEDSENSE_ANSEL ANSELAbits.ANSA2

typedef enum Status 
{
    STATUS_STARTUP,
    STATUS_RX_MODE,
    STATUS_PREAMP_OFF,
    STATUS_RELAY_ON,
    STATUS_TX_MODE,
    STATUS_AMPLIFIER_OFF,
    STATUS_RELAY_OFF          
} status_t;

typedef enum adc_channel
{
    ADC_CHANNEL_INPUTVOLTAGE,
    ADC_CHANNEL_SPEED
} adc_channel_t;

uint8_t portA;
uint8_t portB;
uint8_t portC;
status_t status;
uint16_t adc_value;
uint8_t adc_count;

uint16_t interrupt_count;
uint16_t startup_count;
uint16_t adc_sum;
uint16_t delay;


static void init(void);

static void adc_init(void);
static void adc_set_channel(adc_channel_t channel);
static inline void adc_start_conversion(void);
static uint16_t adc_get_result(void);

static void timer_init(void);
static void ioc_init(void);

static void startup(void);
static void run(void);

static inline void pwrled_on(void);
static inline void pwrled_off(void);
static inline void pwrled_toggle(void);
static inline void pptled_on(void);
static inline void pptled_off(void);
static inline void relay1_on(void);
static inline void relay1_off(void);
static inline void relay2_on(void);
static inline void relay2_off(void);
static inline void relay3_on(void);
static inline void relay3_off(void);

void interrupt ISR(void)
{ 
    //Timer Interrupt
    if(PIR1bits.TMR2IF)
    {
        if(status==STATUS_STARTUP)
        {
            startup();
        }
        else
        {
            run();
        }
        //Clear interrupt flag
        PIR1bits.TMR2IF = 0;
    }
    
    //Interrupt on Change
    if(IOCCFbits.IOCCF1)
    {
        interrupt_count = 0;
        if(PPTSENSE_PIN)
        {
            pptled_off();
            switch(status)
            {
                case STATUS_PREAMP_OFF:
                    relay1_on();
                    status = STATUS_RX_MODE;
                    break;
                case STATUS_RELAY_ON:
                    relay2_off();
                    status = STATUS_RELAY_OFF;
                    break;
                case STATUS_TX_MODE:
                    relay3_off();
                    status = STATUS_AMPLIFIER_OFF;
                    break;
            }
        }
        else
        {
            pptled_on();
            switch(status)
            {
                case STATUS_AMPLIFIER_OFF:
                    relay3_on();
                    status = STATUS_TX_MODE;
                    break;
                case STATUS_RELAY_OFF:
                    relay2_on();
                    status = STATUS_RELAY_ON;
                    break;
                case STATUS_RX_MODE:
                    relay1_off();
                    status = STATUS_PREAMP_OFF;
                    break;
            }
        }
        //Clear interrupt flag
        IOCCFbits.IOCCF1 = 0;
    }
    
    
}

static inline void pwrled_on(void)
{
    PWRLED_VARIABLE |= PWRLED_MASK;
    PWRLED_PORT = PWRLED_VARIABLE;
}

static inline void pwrled_off(void)
{
    PWRLED_VARIABLE &= ~PWRLED_MASK;
    PWRLED_PORT = PWRLED_VARIABLE;
}

static inline void pwrled_toggle(void)
{
    PWRLED_VARIABLE ^= PWRLED_MASK;
    PWRLED_PORT = PWRLED_VARIABLE;
}

static inline void pptled_on(void)
{
    PPTLED_VARIABLE |= PPTLED_MASK;
    PPTLED_PORT = PPTLED_VARIABLE;
}

static inline void pptled_off(void)
{
    PPTLED_VARIABLE &= ~PPTLED_MASK;
    PPTLED_PORT = PPTLED_VARIABLE;
}

static inline void relay1_on(void)
{
    RELAY1_VARIABLE |= RELAY1_MASK;
    RELAY1_PORT = RELAY1_VARIABLE;
}

static inline void relay1_off(void)
{
    RELAY1_VARIABLE &= ~RELAY1_MASK;
    RELAY1_PORT = RELAY1_VARIABLE;
}

static inline void relay2_on(void)
{
    RELAY2_VARIABLE |= RELAY2_MASK;
    RELAY2_PORT = RELAY2_VARIABLE;
}

static inline void relay2_off(void)
{
    RELAY2_VARIABLE &= ~RELAY2_MASK;
    RELAY2_PORT = RELAY2_VARIABLE;
}

static inline void relay3_on(void)
{
    RELAY3_VARIABLE |= RELAY3_MASK;
    RELAY3_PORT = RELAY3_VARIABLE;
}

static inline void relay3_off(void)
{
    RELAY3_VARIABLE &= ~RELAY3_MASK;
    RELAY3_PORT = RELAY3_VARIABLE;
}

static void init(void)
{
    portA = 0x00;
    portB = 0x00;
    portC = 0x00;
    status = STATUS_STARTUP;
    
    
    interrupt_count = 0;
    startup_count = 0;
    adc_value = 0;
    adc_sum = 0;
    adc_count = 0;
    delay = 200;
    
    //Digital outputs
    PPTLED_TRIS = 0;
    PWRLED_TRIS = 0;
    RELAY1_TRIS = 0;
    RELAY2_TRIS = 0;
    RELAY3_TRIS = 0;
    
    //Digital inputs
    PPTSENSE_TRIS = 1;
    PPTSENSE_ANSEL = 0;
    
    //Analog inputs
    VINSENSE_TRIS = 1;
    VINSENSE_ANSEL = 1;
    
    //vin
    TRISCbits.TRISC0 = 1;
    ANSELCbits.ANSC0 = 1;
    
    //Pot
    TRISAbits.TRISA2 = 1;
    ANSELAbits.ANSA2 = 1;
    
    //Configure watchdog timer (WDT)
    // 1 second timeout period
    WDTCONbits.WDTPS4 = 0;
    WDTCONbits.WDTPS3 = 1;
    WDTCONbits.WDTPS2 = 0;
    WDTCONbits.WDTPS1 = 1;
    WDTCONbits.WDTPS0 = 0;
    //Clear WDT
    CLRWDT();
    
    adc_init();
    adc_set_channel(ADC_CHANNEL_INPUTVOLTAGE);
    adc_start_conversion();
    
    timer_init();
}

static void adc_init(void)
{
    //Enable fixed voltage reference
    FVRCONbits.FVREN = 1;
    //Reference voltage 4.096V
    FVRCONbits.ADFVR = 0b11;
    //Turn ADC on
    ADCON0bits.ADON = 1;
    //Conversion Clock = fosc/2
    ADCON1bits.ADCS = 0b000;
    //Negative reference = Ground
    ADCON1bits.ADNREF = 0;
    //Output format right-justified
    ADCON1bits.ADFM = 1;
}

static void adc_set_channel(adc_channel_t channel)
{
    switch(channel)
    {
        case ADC_CHANNEL_INPUTVOLTAGE:
            //ANC0 (pin 10) as source
            ADCON0bits.CHS = 0b010000;
            //Positive reference = Fixed Voltage Reference
            ADCON1bits.ADPREF = 0b11;
            break;
        case ADC_CHANNEL_SPEED:
            //ANA2 (pin 11) as source
            ADCON0bits.CHS = 0b000010;
            //Positive reference = VDD
            ADCON1bits.ADPREF = 0b00;
            break;
    }
}

static inline void adc_start_conversion(void)
{
    //Start a conversion
    ADCON0bits.GO = 1;
}

static uint16_t adc_get_result(void)
{
    uint16_t adc_value;
    //Wait for the result to be ready
    while(ADCON0bits.GO_nDONE)
    {
        __delay_us(25);
    }
    //Read result of ADC conversion
    adc_value = ADRESH;
    adc_value <<= 8;
    adc_value |= ADRESL;
    return adc_value;
}

static void timer_init(void)
{
    //Prescaler = 16
    T2CONbits.T2CKPS = 0b10;
    //Postscaler = 1
    T2CONbits.T2OUTPS = 0b0000;
    // 1 = 1MHz / 4 / 16  = 64us
    //Period = 122 -> 1.024ms
    PR2 = 16;
    //Zero timer 2
    TMR2 = 0x00;
    //Clear Interrupt flag
    PIR1bits.TMR2IF = 0;
    //Enable general interrupts
    INTCONbits.GIE = 1;
    //Enable peripheral interrupts
    INTCONbits.PEIE = 1;
    //Enable timer2 interrupts
    PIE1bits.TMR2IE = 1;
    //Turn timer 2 on
    T2CONbits.TMR2ON = 1;    
}

static void ioc_init(void)
{
    IOCCNbits.IOCCN1 = 1;
    IOCCPbits.IOCCP1 = 1;
    IOCCFbits.IOCCF1 = 0;
    
    //Enable general interrupts
    //INTCONbits.GIE = 1;
    //Enable peripheral interrupts
    //INTCONbits.PEIE = 1;
    //Enable interrupt-on-change
    PIE0bits.IOCIE = 1;
}


void main(void) 
{
    uint8_t cntr;
  
    init();
    
    while(1)
    {
        __delay_ms(50);
        CLRWDT(); 
    }

    return;
}

static void startup(void)
{
    ++interrupt_count;
    adc_value = adc_get_result();
    if(adc_value<ADC_VALUE_STARTUP)
    {
        startup_count = 0;
    }
    if(interrupt_count==100)
    {
        pwrled_on();
        pptled_off();
    }
    if(interrupt_count==200)
    {
        ++startup_count;
        interrupt_count = 0;
        if(startup_count==STARTUP_COUNT)
        {
            adc_set_channel(ADC_CHANNEL_SPEED);
            startup_count = 0;
            if(PPTSENSE_PIN)
            {
                pptled_off();
                relay1_on();
                status = STATUS_RX_MODE;
                
            }
            else
            {
                pptled_on();
                relay2_on();
                status = STATUS_RELAY_ON;
            }
            ioc_init();
        }  
        else
        {
            pwrled_off();
            if(adc_value<ADC_VALUE_STARTUP)
            {
                pptled_on();
            }
        }    
    }
    adc_start_conversion(); 
}

static void run(void)
{
    //Measure pot and calculate delay
    adc_sum += adc_get_result();
    adc_start_conversion();
    ++adc_count;
    if(adc_count==AVERAGE_COUNT)
    {
       delay = adc_sum >> AVERAGE_SHIFT;
       delay += MINIMUM_DELAY;
       adc_sum = 0;
       adc_count = 0;
    }
    
    ++interrupt_count;
    
    if(interrupt_count==delay)
    {
        switch(status)
        {
            case STATUS_AMPLIFIER_OFF:
                relay2_off();
                status = STATUS_RELAY_OFF;
                break;
            case STATUS_RELAY_OFF:
                relay1_on();
                status = STATUS_RX_MODE;
                break;
            case STATUS_PREAMP_OFF:
                relay2_on();
                status = STATUS_RELAY_ON;
                break;
            case STATUS_RELAY_ON:
                relay3_on();
                status = STATUS_TX_MODE;
                break;
        }
        interrupt_count = 0;
    }
    

}