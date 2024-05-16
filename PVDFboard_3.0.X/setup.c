#include "setup.h"

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
    //OSCTUNbits.TUN = 0b0111;//0b011111;       // add on waleed, crystal frequency was not accurate.

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
    TRISAbits.TRISA1 = 0;    // debug
    
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
    PR1 = 16000;                      // CLK = FOSC*PLL/2 = 250 KHz   
    
    IPC0bits.T1IP = 3;               // priority 3
    
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
    
    IPC1bits.T2IP = 2;              // priority 2
    
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
    #ifdef BRGH_SET   
    U1MODEbits.BRGH = 1;            // Fast Speed Mode
    #else
    U1MODEbits.BRGH = 0;            // Normal Mode
    #endif
    U1BRG = BRGVAL;                 // Set Baud Rate to 115200
    ANSBbits.ANSB2 = 0;             // HERE LIED AN ERROR!! RX pin is shared with AN4 = RB2. We must set as digital to make it work.
    U1STAbits.UTXISEL0 = 0;         // Interrupt when the last character is shifted out 
    U1STAbits.UTXISEL1 = 1;         // of the Transmit Shift Register; all transmit operation completed  (01))
    U1STAbits.URXISEL0 = 0;         // Interrupt at every new word in RX buffer 
    U1STAbits.URXISEL1 = 0;
    
    IPC2bits.U1RXIP = 5;            // priority level.
    IPC3bits.U1TXIP = 2;            // priority level.
    
    IEC0bits.U1TXIE = 0;            // disable UART TX interrupt
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;            // Enable UART RX interrupt
    U1MODEbits.UARTEN = 1;          // Enable UART
    U1STAbits.UTXEN = 1;            // Enable UART TX, Transmit is enabled; 
                                    // UxTX pin is controlled by UARTx


    //Communication on serial port 2 (bluetooth)  
    U2MODEbits.UARTEN = 0;          // UART disabled
    U2MODEbits.STSEL = 0;           // 1-Stop bit
    U2MODEbits.PDSEL = 0;           // No Parity, 8-Data bits --> 8,N,1
    U2MODEbits.ABAUD = 0;           // Auto-Baud disabled
    U2MODEbits.UEN = 0;             // Enable only UTX and URX
    #ifdef BRGH_SET 
    U2MODEbits.BRGH = 1;            // Fast Speed Mode 
    #else
    U2MODEbits.BRGH = 0;            // Normal Mode 
    #endif
    U2BRG = BRGVAL;                      // Set Baud Rate to 115200 for port2
    ANSBbits.ANSB1 = 0;             // RX pin is shared with AN2 = RB0. We must set as digital to make it work.
    
    U2STAbits.UTXISEL0 = 0;         // Interrupt when the last character is shifted out 
    U2STAbits.UTXISEL1 = 1;         // of the Transmit Shift Register; all transmit operation completed  (01))
    
    U2STAbits.URXISEL0 = 0;         //Interrupt at every new word in RX buffer 
    U2STAbits.URXISEL1 = 0;
    
    IPC7bits.U2RXIP  = 7;           // recieve interrupt 7 (0b111) highest priority   
    IPC7bits.U2TXIP  = 6;           // send interrupt (6)  high priority
    
    IEC1bits.U2TXIE = 0;            // disable UART TX interrupt
    IFS1bits.U2RXIF = 0;            // reset the flag
    
    IEC1bits.U2RXIE = 1;            // Enable UART RX interrupt
    U2MODEbits.UARTEN = 1;          // Enable UART
    U2STAbits.UTXEN =1;            // Enable UART TX, Transmit is enabled; UxTX pin is controlled by UARTx
}
   
void send_uart (unsigned char msg){
    while(U1STAbits.TRMT == 0){}   
    U1TXREG = msg;
    
}

void send_uart2 (unsigned char msg){
    while (!U2STAbits.TRMT){} // waits for last transmission to end.
    U2TXREG = msg;
}

char read_uart (){
    return 0;
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
    AD1CSSL = 0; // all ingroned we set it in function configure_sequence_MUXA
    
        //Select the appropriate sample/conversion sequence (AD1CON1<7:5> and AD1CON3<12:8>).
    AD1CON3bits.ADRC = 0;           // Clock derived from system clock
    AD1CON3bits.SAMC = 0b01100;     // 12 TAD --> 48us, don't know if this is enough since there is also the acquisition time which can be up to 20us
    
    //Select the analog conversion clock to match the desired data rate with the processor
    AD1CON3bits.ADCS = 1;   //A/D Conversion Clock Select bits:
    // Fcy=16MHz Tcy=62.5 ns -> TAD at least 75 ns this is the converson time per bit
    // TAD = Tcy*(ADCS+1); --> TAD = 2*Tcy = 120ns > 75 ns
   
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





