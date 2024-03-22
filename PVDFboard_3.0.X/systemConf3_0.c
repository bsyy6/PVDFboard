
/*
 * File:   systemConf.c
 * Author: rebeccabaldi
 *
 * Created on June 8, 2020, 7:02 PM
 */


// FBS
#pragma config BWRP = OFF               // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF                // Boot segment Protect (No boot flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Flash Write Protect (General segment may be written)
#pragma config GSS0 = OFF               // General Segment Code Protect (No Protection)

// FOSCSEL
#pragma config FNOSC = FRCDIV            // Oscillator Select (8MHz FRC with Postscaler (FRCDIV)))
#pragma config SOSCSRC = DIG            // SOSC Source Type (Digital Mode for use with external clock on SCLKI)
#pragma config LPRCSEL = HP             // LPRC Power and Accuracy (High Power/High Accuracy)
#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-speed Start-up enabled))

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode (Primary oscillator disabled)
#pragma config OSCIOFNC = ON            // CLKO Pin I/O Function (Port I/O enabled (CLKO disabled))
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range (Primary Oscillator/External Clock frequency >8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switching and Fail-safe Clock Monitor Enabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128            // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = SWON            // Watchdog Timer Enable bits (WDT controlled with SWDTEN bit setting)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected (windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default SCL1/SDA1 Pins for I2C1)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset at 1.8V)
#pragma config MCLRE = ON               // MCLR Pin Enable bit (RA5 input disabled; MCLR enabled)

// FICD
#pragma config ICS = PGx1               // ICD Pin Placement Select (EMUC/EMUD share PGC1/PGD1)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.   

#include <p24F16KL401.h>
#include "xc.h"
#include "HeaderPVDF.h"

#define BRGVAL ((FCY/BAUDRATE)/4)-1 // Baud Rate definition

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Initializes the registers of  the micro. Everything is switched   //
//          off except what is needed                                         //
////////////////////////////////////////////////////////////////////////////////
void init_mcu(){
    
    RCON = 0x00;             // Reset configuration register: reset all flag
    
    //Clear all Interrupt configuration register
    INTCON1 = 0x0000;       // First nesting bit : enable, other flag error  
    INTCON2 = 0x0000;       // All disabled, what is needed will be modified in the proper functions
    //INTCON1 = 0x8000; // MSB at 1 to disable nesting, the other bits are cleared
    INTCON1bits.NSTDIS = 1; //disable nesting
    //SRbits.IPL = 0b010;
    
    //Disable all interrupts
    IEC0 = 0x0000;                 
    IEC1 = 0x0000;                  
    IEC2 = 0x0000;                  
    IEC3 = 0x0000;            
    IEC4 = 0x0000;

    //Clear all interrupt flags
    IFS0 = 0x0000;
    IFS1 = 0x0000;
    IFS2 = 0x0000;
    IFS3 = 0x0000;
    IFS4 = 0x0000;
    IFS5 = 0x0000;
   
    //Enable peripheral
    PMD1 = 0xff;            // Everything disabled (would be on ar reset)
    PMD2 = 0xff;            // Everything disabled (would be on ar reset)
    PMD3 = 0xff;            // Everything disabled (would be on ar reset)
    PMD4 = 0xff;            // Everything disabled (would be on ar reset)
    
    //Power Management
    PMD1bits.T1MD = 0;      // Enable Timer1
    PMD1bits.T2MD = 0;      // Enable Timer2
    PMD3bits.SPI2MD = 0;    // Enable SPI2
    PMD1bits.SPI1MD = 0;    // Enable SPI1
    PMD1bits.U1MD = 0;      // Enable UART1
    PMD1bits.U2MD = 0;      // Enable UART2
    PMD4bits.EEMD = 0;
        
    //Clock initialization
    CLKDIVbits.RCDIV = 0b000;       // Post-scaler = 1, default would be 2
    
    //Data EEPROM table page
    
    
    //LED RGB
    ANSBbits.ANSB12 = 0;
    ANSBbits.ANSB13 = 0;
    ANSBbits.ANSB14 = 0;
    TRISBbits.TRISB12 = 0;   // LED R
    TRISBbits.TRISB13 = 0;   // LED G
    TRISBbits.TRISB14 = 0;   // LED B
    
    //Define motor output ports
    TRISAbits.TRISA4 = 0;    // MOT1
    TRISAbits.TRISA6 = 0;    // MOT2
    
    //Bluetooth configuration
    TRISBbits.TRISB8 = 0;   // BT_RESET (output)
    TRISBbits.TRISB9 = 0;   // BT_WAKEUP (output)
    TRISBbits.TRISB4 = 1;   // BT_BUSY (input)
    TRISBbits.TRISB15 = 0;  // SW1 (output)
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Initializes the registers of Timer1 and switches off the timer    //
//          It must be switched on in the main                                //
////////////////////////////////////////////////////////////////////////////////
void tmr1_init(){ // 16 bit Timer
    
    T1CONbits.TON = 0;              // Timer switched off
    T1CONbits.TGATE = 0;            // Disable TGate

    T1CONbits.TCKPS = 0b00;         // Prescale 1:1
    T1CONbits.TCS = 0;              // Internal Clock

    TMR1 = 0x0000;                  // Clear timer register
    PR1 = 4000;                      // CLK = FOSC/2 = 250 KHz   
    
    IPC0bits.T1IP0 = 1;           // PRIORITY = 3
    IPC0bits.T1IP1 = 1;
    IPC0bits.T1IP2 = 0;
    
    TMR1_FLAG = 0;                  // Clear Flag
    TMR1_INT_ENABLE = 0;            // Interrupt disabled (must be enabled in main)
    TMR1_ENABLE = 0;                // Timer switched off
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Initializes the registers of Timer2 and switches off the timer    //
//          It must be switched on in the main                                //
////////////////////////////////////////////////////////////////////////////////
void tmr2_init(){ // 8 bit Timer --> not used
    
    T2CONbits.T2OUTPS = 0b1001;     // 1:1 Postscaler
    T2CONbits.T2CKPS = 0b10;       

    TMR2 = 0x00;                    // Clear timer register
    PR2 = 250;                      // CLK = FOSC/2 = 250 KHz   
    
    IPC1bits.T2IP0 = 0;             //PRIORITY = 2
    IPC1bits.T2IP1 = 1;
    IPC1bits.T2IP2 = 0;
    
    TMR2_FLAG = 0;                  // Clear Flag
    TMR2_INT_ENABLE = 0;            // Interrupt disabled (must be enabled in main)
    TMR2_ENABLE = 0;
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Initializes the registers of SPI1 and  enables the SPI module     //
////////////////////////////////////////////////////////////////////////////////
//void init_spi(void){
//    SSP1CON1bits.SSPEN  = 0; // Disable SPI module
//    //SSP1STAT Register Settings
//    SSP1STATbits.SMP = 0; 			// Input data is sampled at the middle of data output time
//    SSP1STATbits.CKE = 1; 			// Serial output data changes on transition from
//                                    // Idle clock state to active clock state
//    //SSP1CON1 Register Settings
//    SSP1CON1bits.WCOL = 0;          // Must wait to write in BUF that the transmission is end
//    SSP1CON1bits.CKP  = 0; 			// Idle state for clock is a low level;
//    SSP1CON1bits.SSPM = 0b0000;     // Master Mode enabled, Clock set like global clock
//    
//    //PADCFG1  Register Settings    
//    PADCFG1bits.SDO2DIS = 1;        //The SPI output data (SDO2) of MSSP2 to the pin is disabled
//    PADCFG1bits.SCK2DIS = 1;        //The SPI clock (SCK2) of MSSP2 to the pin is disabled
//    PADCFG1bits.SDO1DIS = 0;        //The SPI output data (SDO1) of MSSP1 is output to the pin
//    PADCFG1bits.SCK1DIS = 0;        //The SPI clock (SCK1) of MSSP1 is output to the pin
    
    //TRIS_SDI = 1;
    
    // Disable all the analog ports
    //ANSBbits.ANSB14 = 0;    //SDI
    //ANSBbits.ANSB4 = 0;     // SS2
    //ANSBbits.ANSB12 = 0;    // SCK1
    //ANSBbits.ANSB2 = 0;     // SS3
    // ANSB8 unimplemented - SS1
    // ANSB7 unimplemented - SS4
    //SSP1CON1bits.SSPEN  = 1;        // Enable SPI module
//}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Initializes the registers of UART1 and enables UART1 and          //
//          the transmission                                                   //
////////////////////////////////////////////////////////////////////////////////
void init_uart(void){
    //Communication on serial port 1 (serial cable)
    U1MODEbits.UARTEN = 0;          // UART disabled
    U1MODEbits.STSEL = 0;           // 1-Stop bit
    U1MODEbits.PDSEL = 0b00;        // No Parity, 8-Data bits --> 8,N,1
    U1MODEbits.ABAUD = 0;           // Auto-Baud disabled
    U1MODEbits.UEN = 0b00;          // Enable only UTX and URX
    U1MODEbits.BRGH = 1;            // Fast Speed Mode 
    
    U1BRG = 15;                      // Set Baud Rate to 62500 (=8 IF 115200)
    
    ANSBbits.ANSB2 = 0;             // HERE LIED AN ERROR!! RX pin is shared with AN4 = RB2. We must set as digital to make it work.
    U1STAbits.UTXISEL0 = 0;         //Interrupt when the last character is shifted out 
    U1STAbits.UTXISEL1 = 1;         //of the Transmit Shift Register; all transmit operation completed  (01))
    
    U1STAbits.URXISEL0 = 0;         //Interrupt at every new word in RX buffer 
    U1STAbits.URXISEL1 = 0;
    
    IPC2bits.U1RXIP0 = 1;           // Give the receiving message a higher priority than everything else 
    IPC2bits.U1RXIP1 = 1;           // PRIORITY = 8
    IPC2bits.U1RXIP2 = 1;
    
    IPC3bits.U1TXIP0 = 0;           // Give the transmission message a lower priority than everything else 
    IPC3bits.U1TXIP1 = 1;           // PRIORITY = 2
    IPC3bits.U1TXIP2 = 0;
    //ANSBbits.ANSB7 = 0;           // Disable Analog port for U1TX
    //TRISBbits.TRISB7 = 1;         // Quando così è set come input (1) U1TX 
    
    IEC0bits.U1TXIE = 0;            // Enable UART TX interrupt
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;            // Enable UART RX interrupt
    
    U1MODEbits.UARTEN = 1;          // Enable UART
    U1STAbits.UTXEN = 1;            // Enable UART TX, Transmit is enabled; UxTX pin is controlled by UARTx
    
    
}

void send_uart (char Message){
    
    U1TXREG = Message;                // Send a byte
    while(U1STAbits.TRMT == 0){   // Wait until the transmit shift register is empty and the transmit buffer is empty (the transmission has completed), it does it automatically
    }
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Disable all the interrupts, implementation of the Global Interrupt//
//          bit                                                       //
////////////////////////////////////////////////////////////////////////////////
void GIE_OFF(void){
    // Disable all interrupts 
    IEC0 = 0x0000;                 
    IEC1 = 0x0000;                  
    IEC2 = 0x0000;                  
    IEC3 = 0x0000;            
    IEC4 = 0x0000;
}

////////////////////////////////////////////////////////////////////////////////
//Input:    Data to save in EEPROM, address in which to save                                                              //
//Output:   None                                                              //
//Function: Disable all the interrupts, and write in the right cell of the    //
//          EEPROM                                                            //
////////////////////////////////////////////////////////////////////////////////

void Save_EEPROM(int data, int address){ 
    
    __builtin_disi(0x3FFF);     // Blocks interrupts for as long as we need 
    NVMCON = 0x4004;
    __builtin_tblwtl(address,data);
    __builtin_write_NVM(); // Issue Unlock Sequence & Start Write Cycle
    while(NVMCONbits.WR==1){};
    __builtin_disi(0x0000);    // Remove the block on the interrupts
     
}


////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: Initializes the registers of ADC and  enables the ADC module     //
////////////////////////////////////////////////////////////////////////////////

void init_ADC(void){
    PMD1bits.ADC1MD = 0;        // Enables clock and all registers associated with ADC.
    AD1CON1bits.ADON = 0;       // Turn off.
    AD1CON1bits.ADSIDL = 0;     // Continue module operation in Idle mode
    AD1CON1bits.FORM = 00;      // Integer (0000 00dd dddd dddd) 0 - 1023
    AD1CON1bits.SSRC = 0b111;   // Internal counter ends sampling and starts conversion (auto-convert) - Trigger source
    AD1CON1bits.ASAM = 0;       // Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    AD1CON1bits.SAMP = 0;       // A/D sample/hold amplifiers are holding,  manual command to start the sampling 
    //AD1CON1bits.DONE = 0;       //A/D conversion is not done or has not started (READ ONLY)
            
    //PVDF1OUT & PVDF2OUT    
    ANSAbits.ANSA0 = 1;         // Enable RA0 to use as analog input
    TRISAbits.TRISA0 = 1;       // RA0 as input
    
    ANSAbits.ANSA2 = 1;         // Enable RA2 to use as analog input
    TRISAbits.TRISA2 = 1;       // RA2 as input
    
    //Select the voltage reference source
    AD1CON2bits.VCFG =0b000;        // Configure A/D voltage reference Vr+ and Vr- from AVdd and AVss (VCFG<2:0>=000),
    AD1CON2bits.CSCNA = 1;          // MUXA Scans input 
    //AD1CON2bits.BUFS = 0;        // buffer status, only valid when ADC1BUF is functioning as two buffers (BUFM = 1).
    AD1CON2bits.SMPI = 0b0001;      //Interrupt rate: Interrupts at the completion of conversion for each 2nd sample/convert sequence
    //AD1CON2bits.BUFM = 0;           //Buffer configured as one 16-word buffer (ADC1BUF0 to ADC1BUFF) 
    AD1CON2bits.ALTS = 0;           //Always uses MUX A input multiplexer settings
//    AD1CON2bits.ALTS = 1;           //Uses MUX A input multiplexer settings for the first sample, then alternates between MUX B and MUX A input multiplexer settings for all subsequent samples
    
    // Select the pins we want to sequentially scan, the ANx ports on the schematics
    AD1CSSLbits.CSSL1 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL2 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL3 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL4 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL6 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL7 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL8 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL10 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL11 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL12 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL0 = 0; //Corresponding analog channel not selected for input scan--> NOT USED WE HAVE ONLY 2 PVDF SENSORS
    AD1CSSLbits.CSSL13 = 0; //Corresponding analog channel selected for input scan--> NOT USED WE HAVE ONLY 2 PVDF SENSORS
    AD1CSSLbits.CSSL14 = 0; //Corresponding analog channel selected for input scan--> NOT USED WE HAVE ONLY 2 PVDF SENSORS
    AD1CSSLbits.CSSL15 = 0; //Corresponding analog channel selected for input scan
    AD1CSSLbits.CSSL9 = 0; //Corresponding analog channel selected for input scan

   
    // Configure input channels MUXA: CH0+ input is AN0, CH0- input is Vr- (AVss)
   // AD1CHSbits.CH0SA = 0b1001; // AN9 Channel 0 Positive Input Select for MUX A Multiplexer Setting bits
   // AD1CHSbits.CH0NA = 0;      // Channel 0 negative input is Vr-
//    AD1CHSbits.CH0SB = 0b1111; // AN15 Channel 0 Positive Input Select for MUX B Multiplexer Setting bits
//    AD1CHSbits.CH0NB = 0;      // Channel 0 negative input is Vr-
    
    //Select the appropriate sample/conversion sequence (AD1CON1<7:5> and AD1CON3<12:8>).
    AD1CON3bits.ADRC = 0;           // Clock derived from system clock
    AD1CON3bits.SAMC = 0b01100;     // 12 TAD --> 48us, don't know if this is enough since there is also the acquisition time which can be up to 20us
    
    //Select the analog conversion clock to match the desired data rate with the processor
    AD1CON3bits.ADCS = 0b00000000;   //A/D Conversion Clock Select bits: Fcy=250KHz Tcy=4us -> TAD at least 75 ns this is the converson time per bit
    // TAD = Tcy*(ADCS+1); --> TAD = Tcy = 4us
   
    // Interrupts control
    IFS0bits.AD1IF = 0;
    // set ADC priority, default priority is 4, check if this is the problem of reading input messages (also uart receiver has priority = 4)
    //IPC3bits.AD1IP0 = 1; //PRIORITY = 3
    //IPC3bits.AD1IP1 = 1; //PRIORITY = 3
    //IPC3bits.AD1IP2 = 0; //PRIORITY = 3
    IEC0bits.AD1IE = 0;
    //AD1CON1bits.ADON = 0;
//    ADC_FLAG = 0;                  // CLEAR flag Interrupt
//    ADC_INT_ENABLE = 0;            // disabled interrupt
//    ADC_ENABLE = 0;                // ADC off     
    //AD1CON1bits.ADON = 1; // turn ADC ON
}


//
//To perform an A/D conversion:
//1. Configure the A/D module:
//a) Configure port pins as analog inputs and/
//or select band gap reference inputs
//(ANSA<3:0>, ANSB<15:12,4:0> and
//ANCFG<0>).
//b) Select the voltage reference source to
//match the expected range on analog inputs
//(AD1CON2<15:13>).
//c) Select the analog conversion clock to match
//the desired data rate with the processor
//clock (AD1CON3<7:0>).
//d) Select the appropriate sample/conversion
//sequence (AD1CON1<7:5> and
//AD1CON3<12:8>).
//e) Select how conversion results are
//presented in the buffer (AD1CON1<9:8>).
//f) Select interrupt rate (AD1CON2<5:2>).
//g) Turn on A/D module (AD1CON1<15>).
//2. Configure A/D interrupt (if required):
//a) Clear the AD1IF bit.
//b) Select A/D interrupt priority.

