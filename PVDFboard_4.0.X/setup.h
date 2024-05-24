#ifndef SETUP_H
#define	SETUP_H


// MUST MATCH in config_bits.h
#define FOSC    8000000UL     // internal crystal FRC 8MHz  
#define XTFREQ  8000000UL     // internal crystal FRC 8MHz 
#define _XTAL_FREQ 8000000UL
#define PLL_MULT 4          // PLL multiplier if - must match FNOSC = FRCPLL 
#define CLKDIV_VAL 1        // postscaler value  - must match CLKDIVbits.RCDIV = 0b000;  
#define BAUDRATE 115200 

#define FCY (FOSC*PLL_MULT)/(2*CLKDIV_VAL) //Instruction Cycle Frequency


#define BRGH_SET // BRGH = 1;

#ifdef BRGH_SET
#define BRGVAL (FCY/(BAUDRATE*4))-1 // datasheet pg 150
#else
#define BRGVAL (FCY/(BAUDRATE*16))-1 // datasheet pg 150
#endif

#include <xc.h> // include processor files - each processor file is guarded.

#define BIT0  0b0000000000000001
#define BIT1  0b0000000000000010
#define BIT2  0b0000000000000100
#define BIT3  0b0000000000001000
#define BIT4  0b0000000000010000
#define BIT5  0b0000000000100000
#define BIT6  0b0000000001000000
#define BIT7  0b0000000010000000
#define BIT8  0b0000000100000000
#define BIT9  0b0000001000000000
#define BIT10 0b0000010000000000
#define BIT11 0b0000100000000000
#define BIT12 0b0001000000000000
#define BIT13 0b0010000000000000
#define BIT14 0b0100000000000000
#define BIT15 0b1000000000000000
    
/*****************************************************/
/*************   IO digitali     ********************/
/****************************************************/

#define IO_PIN_TRIS                      TRISAbits.TRISA6
#define IO_PIN                           PORTAbits.RA6

//SPI
#define TRIS_SCK                         TRISBbits.TRISB12
#define TRIS_SDI                         TRISBbits.TRISB14
#define TRIS_SDO                         TRISBbits.TRISB13
 
    
//ADC
#define ADC_FLAG                    IFS0bits.AD1IF
#define ADC_INT_ENABLE              IEC0bits.AD1IE
#define ADC_ENABLE                  AD1CON1bits.ADON
    
/****************************************************/
/*******************   TIMER1    ********************/
/****************************************************/
// TIMER 1 @ 1KHz 
#define TMR1_FLAG                   IFS0bits.T1IF
#define TMR1_INT_ENABLE             IEC0bits.T1IE
#define TMR1_ENABLE                 T1CONbits.TON   
    
#define TMR2_FLAG                   IFS0bits.T2IF
#define TMR2_INT_ENABLE             IEC0bits.T2IE
#define TMR2_ENABLE                 T2CONbits.TON

/****************************************************/
/*******************   PINS    ********************/
/****************************************************/
// MOTOR PORTS
#define MOT1             LATAbits.LATA4      // Motor 1 activation
#define MOT2             LATAbits.LATA6      // Motor 2 activation

// LED
#define REDLED           PORTBbits.RB12      // RED LED (RGB)
#define GREENLED         PORTBbits.RB13      // GREEN LED (RGB)
#define BLUELED          PORTBbits.RB14      // BLUE LED (RGB)

// BLUETOOTH
#define BT_WAKEUP        PORTBbits.RB9     
#define BT_RESET         PORTBbits.RB8     

#define DEBUG_PIN        LATAbits.LATA1
  
void init_mcu(); 
void tmr1_init();
void tmr2_init();
void init_spi(void);
void init_uart(void);
void GIE_OFF(void);
void Save_EEPROM (int data, int address);
void init_ADC(void);
void send_uart (unsigned char Message);
void send_uart2 (unsigned char msgS);
#endif	