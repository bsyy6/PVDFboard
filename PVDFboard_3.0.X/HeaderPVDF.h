/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:  PVDFboard_v1 
 * Author: Rebeccabaldi
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


 // PIC24F16KL401 Configuration Bit Settings

// 'C' source line config statements

/****************************************************/   
/****************************************************/
/****************************************************/
#define FCY 4000000
#define BAUDRATE 62500
#define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay
    

/****************************************************/
/*************      BIT MASK     ********************/
/****************************************************/   
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
/**************  function declaration  **************/
/****************************************************/  
void init_mcu(); 
void tmr1_init();
void tmr2_init();
void init_spi(void);
void init_uart(void);
void GIE_OFF(void);
void Save_EEPROM (int data, int address);
void init_ADC(void);
void send_uart (char Message);
void send_uart2 (char msgS);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */
