
/*
 * File:   MainPVDF.c
 * Author: Eleonora Vendrame
 *
 * Created on January 3, 2024
 * Rev. 
 */

/* v3.0   * Reading of 2 analog piezoelectric (PVDF) sensor, sampled at 1KHz
 * 
 * ------------ Testing parts ----------
 * - only UART tested.
 * 
 * - results:
 *  
 *  
 * UART transmission of sensors data
 * - BAUDRATE to set on the Serial Terminal Port is 62500 
 * - Not used. First byte sent by the main, the following bytes are sent by interrupt
 *
 * Interrupts:
 * - Timer1 --> to increment variables
 * - UART TX
 * - ADC --> it samples at 1 KHz 
 *
 * 
 */
#include "setup.h"       // <- MUST BE INCLUDED FIRST
#include "xc.h"
#include "buffers.h"    
#include "msgs.h"
#include <stdio.h>                           
#include <math.h> 
#include <stdlib.h>
#include <libpic30.h>
#include <stdbool.h> 
#include <stdint.h>
#include <limits.h>

// DEFINE PARAMETERS
#define wLenght        10   // lunghezza del buffer di acquisizione dell'adc, acquisendo 1 campione ogni 1 ms, sar� pieno dopo 10 ms visto che legge un campione per sensore 
#define windowLenght   10   // Event detection window length, 10ms
#define nSensors       2    // number of sensors to read
#define DutyCycleT     0x09   //duty cycle per attivare i motori di uno spike nel caso di evento di tocco 80%
#define DurationT      0x0F   // durata dello spike per l'evento di tocco 150ms
#define DutyCycleR     0x9    //duty cycle per attivare i motori di uno spike nel caso di evento di rilascio 80%
#define DurationR      0x0F   // durata dello spike per l'evento di tocco 150ms
#define ADCBufferLength 20    // ADC Buffer length, 20ms 

// VARIABLES

//bool state = true;
unsigned char count = 0;                     // Counter for Timer1 (increased every match with PR1), Press acquisition counter

unsigned int PVDFsensor[nSensors] = {0};                          // Raw Data ADC values for all channels
unsigned int iPVDFFiltered[ADCBufferLength][nSensors]= {{0},{0}}; // Data filtered by the LP filter

unsigned char w = 0;                                              // write index, shifts acquisition buffer from ADC
unsigned char p = 0;                                              // channel index, indicates which channel the ADC is scanning
unsigned char icheckEvent = 0;                                    // flag to check if there is an event, if==0: mette a 1 il flag per la copia e non fa check, if>0 controlla l'evento

int iPVDFFilteredBuffer[ADCBufferLength][nSensors] = {{0},{0}};     
int derivative[nSensors] = {0};   // derivative global variable

// EEPROM - Global variable located in EEPROM

unsigned int __attribute__ ((space(eedata))) eeUsedSensors = 2;  // Number of sensors connected to the board

unsigned int __attribute__ ((space(eedata))) eeThresholdT[nSensors] = {776,776};   //Contact Threshold 0x0308
unsigned int __attribute__ ((space(eedata))) eeThresholdR[nSensors] = {62,62}; //Release Threshold 0x003E

int __attribute__ ((space(eedata))) eeThresholdDerT [nSensors] = {80,80};    // Contact Threshold - Derivative 0x0050
int __attribute__ ((space(eedata))) eeThresholdDerR [nSensors] = {-35,-35};  // Release Threshold - Derivative 0xFFDD

unsigned int __attribute__ ((space(eedata))) eeMotFlag = 1;  // Flag for motors on[1]/off[0]
unsigned int __attribute__ ((space(eedata))) eeSendDer = 0;
unsigned int __attribute__ ((space(eedata))) eeStimTime = 150; // 0x32

// EEPROM data addresses

int tbloffset_thrT;
int tbloffset_thrR;
int tbloffset_thrderT;
int tbloffset_thrderR;
int tbloffset_nSens;
int tbloffset_SendDer;
int tbloffset_StimTime;
int tbloffset_MotFlag;

// Local variables saved from EEPROM

bool MotFlag;
int ActiveSens = 2;
int StimTime;
int ThresholdT[nSensors];
int ThresholdR[nSensors];
int ThresholdDerT[nSensors];
int ThresholdDerR[nSensors];
int iuart = 0;

// DIGITAL LOW PASS FILTER (Uncomment the section related to the filter you want to use)

//  Butterworth 2nd order, Fcut = 17 Hz
unsigned int x_1[nSensors] = {0};           // starting values for filter
unsigned long y_1[nSensors] = {0};
unsigned int x_2[nSensors] = {0};           // starting values for filter
unsigned long y_2[nSensors] = {0};

// Old IIR filter (low pass filter, not restrictive)
//unsigned int x_1[nSensors] = {0};           // starting values for filter
//unsigned int y_1[nSensors] = {0};

// Butterworth 4th order 
//unsigned int x_1[nSensors] = {0};           // starting values for filter
//float y_1[nSensors] = {0};
//unsigned int x_2[nSensors] = {0};           // starting values for filter
//float y_2[nSensors] = {0};
//unsigned int x_3[nSensors] = {0};           // starting values for filter
//float y_3[nSensors] = {0};
//unsigned int x_4[nSensors] = {0};           // starting values for filter
//float y_4[nSensors] = {0};

// Moving Average
//unsigned int movavg[nSensors][windowLenght] = {{0},{0}};


/// DESC FEEDBACK
unsigned char iCountEventT[nSensors] = {0};            // Counter for contact events
unsigned char iCountEventR[nSensors] = {0};            // Counter for release events
unsigned char ContactEvent[nSensors] = {0};            // Indicates that an application of pressure occurred
unsigned char ReleaseEvent[nSensors] = {0};            // Indicates that a release of pressure occurred
unsigned char MotorsActivation[nSensors] = {0};        // Temporary vector of variables to store the information regarding the activation of the 5 sensors to be sent to the motors
unsigned int tRefractory_max = 140;                    // Time to wait before activating motors after an event detection
unsigned int tLed_max = 200;                           //

/// COUNTERS FOR TIME INTERRUPTS
unsigned int CountVibr1 = 0;
unsigned int CountVibr2 = 0;

unsigned int tRefractory[nSensors] = {0};
unsigned int bluetime = 0;
unsigned int redtime = 0;
unsigned int Timer = 0;                             //Variable to send to SW as timestamp (100Hz)

uint8_t byteRecieved =0;
/***********/

// SERIAL COMMUNICATION - Modified : Waleed March-2024


uint8_t inBuffer1[20];    // uart1 buffer
uint8_t msgData1[10];     // extracted msgs go here.
uint8_t msgDataSize1 = 0; // the size of data in msgData
volatile Buffer b_inBuffer1; 

uint8_t outBuffer1[23];
volatile Buffer b_outBuffer1;


// Bluetooth
uint8_t inBuffer2[50]; // uart2 buffer
uint8_t msgData2[35];  // extracted msgs go here
uint8_t msgDataSize2 = 0; // the size of data in msgData
volatile Buffer b_inBuffer2;

uint8_t outBuffer2[59];
volatile Buffer b_outBuffer2;

bool flag_BT_reset = false;
unsigned char BT_in = 0;

uint8_t BTMAC_computer[6] = {0x5D,0xE4,0x32,0xDA,0x18,0x00};
uint8_t BTMAC_pcb[6] = {0x85, 0x98, 0x32,0xDA, 0x18 ,0x00};

uint8_t tempByte;

unsigned char byteRead[20] = {};

typedef enum {
    BT_POWER_OFF,     // 0
    BT_POWER_ON,      // 1
    BT_CONNECTED,     // 2
    BT_NOT_CONNECTED,  // 3
    BT_OK,
    BT_NOT_OK,
} bt_states;

bt_states bt_state = BT_POWER_OFF; 
uint8_t delay_attempts = 0;
/***********/

unsigned char checkCOM = 0;
unsigned char StartRX = 0;             // wait StartRX=1 for enabling serial communication
volatile unsigned char ind=0;          // shifts RX buffer
unsigned char NewDataArrived = 0;      // Flag to signal new data in RX
unsigned char SendData = 0;            // Flag to allow sending data at 100Hz
unsigned char aux = 0;                 // Used to populate properly the output buffer
unsigned char StartPoint = 0;          // Reading of data from input buffers
unsigned char OutByteToSend = 1;

unsigned char CircledBuffer = 0;     // for data acquisition
unsigned char CopiedData = 0; 
unsigned char StartDetection = 0;

// FUNCTIONS DECLARATION
void copyBuffer(unsigned int EndPoint);
void eventDetection(int s);
unsigned int LP_filter(unsigned int x0);
void configure_sequence_MUXA(char s1, char s2);
void ManageSerialTX(void);
void ManageSerialRX(void);
void ReadCmd(uint8_t* msgHolder);
void init_buffer(void);
void sr_LED_primary(char led, bool state);
void set_LED(char led);

bt_states init_BT();
bt_states connect_BT(uint8_t *BTMAC); // returns true if connected successfully
bt_states setJustWorks(void);        //  returns true if connected successfully
bt_states whoAmI(uint8_t *BTMAC);    // 
void ManageSerialTX2(void);

volatile unsigned int T1counter = 0;
const unsigned int T1counter_max = UINT_MAX; // 0xFFFF
unsigned char circleClrs = 0;
const char colors[] = {'R', 'G', 'B', 'Y', 'C', 'M', 'W','0'};
unsigned char  colorCounter = 0;
unsigned int colorTime = 900; 
volatile bool LED_switched = false;
volatile bool blockLED = false;
volatile bool LED_R_B = true;

unsigned int addmsec (unsigned int msec){
    // returns how much will bel the value of T1counter after msecs
    // used to detect and issue timeout. 
    return (T1counter + msec) % (T1counter_max) + 1;
}
//Timer1 interrupt
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){ 
    //@1KHz -> 1 msec
    DEBUG_PIN = !DEBUG_PIN;
    
    T1counter++;
    if(T1counter >= 1000){ 
        T1counter = 0;
        if(LED_R_B){
            set_LED('R');
            LED_R_B = false;
        }else{
            set_LED('B');
            LED_R_B = true;
        }
    }
    
    LED_switched = false;
    count++; 
    CountVibr1++;
    CountVibr2++;

    bluetime++;
    redtime++;

    if (count == 10) //makes it 100Hz 
    {    
        count = 0;
        SendData=1;
        Timer++;
        
    }
    
    unsigned int q ; 
    for (q=0; q<ActiveSens; q++)
    {
        // to avoid double detection of the same contact event
        tRefractory[q]++;
    }
    
    AD1CON1bits.SAMP = 1;
	
    IFS0bits.T1IF = 0; 
   
}


void __attribute__((__interrupt__)) _ADC1Interrupt(void){
    
    PVDFsensor[0] = ADC1BUF0;         // Read the AN9 channel conversion result
    iPVDFFiltered[w][0] = LP_filter(PVDFsensor[0]);    // filtro i dati Sensore p e riempio il buffer 
    // Fill the vector with data from ADC buffer 2
    PVDFsensor[1] = ADC1BUF1;         // Read the AN15 channel conversion result
    iPVDFFiltered[w][1] = LP_filter(PVDFsensor[1]);    // filtro i dati Sensore p e riempio il buffer 
    
    w++;
    if(w==ADCBufferLength){       //    w=(w+1)%ADCBufferLength;
        w=0;
        CircledBuffer = 1;
    }
    
    IFS0bits.AD1IF = 0;
}

// UART1 interrupts
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void){
    while(!U1STAbits.TRMT){}
    IFS0bits.U1TXIF = 0;  // Clear TX Interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){   
    // I put data in buffer nothing more nothing less.
    while(U1STAbits.URXDA){
        uint8_t byteRecieved;
        byteRecieved = U1RXREG;
        enq(&byteRecieved,&b_inBuffer1);
    }
    if (U1STAbits.OERR){
        U1STAbits.OERR = 0;
    }
    IFS0bits.U1RXIF = 0;                // Clear TX Interrupt flag
}

// UART2 interrupts
void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void){
    while(!U2STAbits.TRMT){}
    IFS1bits.U2TXIF = 0;  // Clear TX Interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void){
    while(U2STAbits.URXDA){
//        uint8_t byteRecieved;
        byteRecieved = U2RXREG;
        enq(&byteRecieved,&b_inBuffer2);
    }
    if (U2STAbits.OERR){
        U2STAbits.OERR = 0;
    }
    
    IFS1bits.U2RXIF = 0;                // Clear TX Interrupt flag
}

////////////////////////////////////////////////////////////////////////////////
//Input:    Index of the last value read fromm the ADC(EndPoint)              //
//Output:   None                                                              //
//Function: This function makes a copy of the samples acquired by the ADC for //
//          every channel
////////////////////////////////////////////////////////////////////////////////
void copyBuffer(unsigned int EndPoint){ // da eseguire if copyFlag[p] == 1
    
    int i;
    int j;
    int DataToRead = EndPoint-StartPoint+1; 
    if (CircledBuffer == 1){
        CircledBuffer = 0;
        DataToRead+=ADCBufferLength;
    }       
    
    for(i=0; i<ActiveSens; i++){
        for(j=0; j<DataToRead; j++){ // copio in un buffer di 10 per ogni sensore
            iPVDFFilteredBuffer[(StartPoint+j)%ADCBufferLength][i] = iPVDFFiltered[(StartPoint+j)%ADCBufferLength][i];
        }
    }
    
    CopiedData+= DataToRead;
    if (CopiedData > wLenght){
        CopiedData = CopiedData%10;
        icheckEvent = 1; // una volta terminata la copia posso andare a rilevare l'evento sulla copia
    }
    StartPoint = EndPoint+1;
 }


////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: This function initializes those values of the buffer that don't   //
//          need to be changed every time.                                    //
////////////////////////////////////////////////////////////////////////////////

void init_buffer (void){
    outBuffer1[0] = 0xA1;
    outBuffer1[1] = 0xA2;
    
    outBuffer1[2] = (ActiveSens*7)+9;     // this should be the number of data sent through the usart
    outBuffer1[3] = MotFlag;
    outBuffer1[4] = StimTime;
    // 5-9 are handled in ManageSerialTx
    outBuffer1[10] = ThresholdT[0]/4;      // the threshold voltage sensor 1  - read from the EEPROM
    outBuffer1[11] = ThresholdR[0]/4;
    outBuffer1[12] = ThresholdDerT[0]/4;
    outBuffer1[13] = (char)-1*ThresholdDerR[0]/4;
    outBuffer1[17] = ThresholdT[1]/4;     // the threshold voltage sensor 2  - read from the EEPROM
    outBuffer1[18] = ThresholdR[1]/4;
    outBuffer1[19] = ThresholdDerT[1]/4;
    outBuffer1[20] = (char)-1*ThresholdDerR[1]/4;
    
    outBuffer1[21] = 0xA2;
    outBuffer1[22] = 0xA1;      
    return;
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: This function updates the Output Buffer with the right values of  //
//          analog signals to send the interface                              //
////////////////////////////////////////////////////////////////////////////////
void ManageSerialTX(void){
    unsigned int j;
    outBuffer1[5] = (Timer & 0xFF00) >> 8;   // time stamp read at 100Hz
    outBuffer1[6] = (Timer & 0x00FF);
    for (j=0; j<ActiveSens; j++){
        aux = 7*j;
        outBuffer1[7+aux] = MotorsActivation[j];     // For each sensor/motor we want to send its state of activation
        
        outBuffer1[8+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0xFF00) >> 8;   // the voltage level read at 100Hz
        outBuffer1[9+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0x00FF);
    }
    b_outBuffer1.head = 0;
    b_outBuffer1.isEmpty = false;
    b_outBuffer1.isFull  = true;
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: This function analyzes the Serial Input Buffer to recognise header//
//          and tail of a valid message, and in this case calls ReadCmd to    //
//          perform the relative command.
////////////////////////////////////////////////////////////////////////////////
//
//void ManageSerialRX(void) {
//
//    int message_size;
//    int message_arrived=0;
//    int i;
//    for (i=0; i<InputBufferLength; i++){
//        if ((InputBuffer[i]==0xA1) && (InputBuffer[(i+1)%InputBufferLength]==0xA2)){
//            message_size = InputBuffer[(i+2)%InputBufferLength];
//            if ((InputBuffer[(i+message_size-1)%InputBufferLength]==0xA1) && (InputBuffer[(i+message_size-2)%InputBufferLength]==0xA2)){
//               ReadCmd(i,InputBuffer[(i+3)%InputBufferLength]); 
//               bluetime = 0;
//               BLUELED = 1;
//               InputBuffer[i]=0; 
//               message_arrived=1;
//            }
//        }
//    }
//    NewDataArrived=0;
//}


//comState prevState = ERROR;
//comState state = A1START;
////unsigned char size = 0;
//unsigned char countMsg = 0;


////////////////////////////////////////////////////////////////////////////////
//Input:    Byte of the Serial Input Buffer from where we start reading(start)//
//          Command type read in the incoming message(command_type)
//Output:   None                                                              //
//Function: This function performs the due changes to respond to the correct  //
//          command type, like changing the threshold for different sensors,  //
//          selecting different size and number of sensors to use, or switch  //
//          between streaming derivative or analog signal.                    //
//////////////////////////////////////////////////////////////////////////////// 

void ReadCmd(uint8_t *msgHolder){
    uint8_t shift;
    if(msgHolder[0] == 0xA1){
       shift = 3; // the first two are start flags 
    }else if(msgHolder[0] == 0x02){ 
       shift = 14; // the first 11 are start flags 
    }else
    {
       return;
    }
    
    switch (msgHolder[shift+0]){
        case 0:
            checkCOM = 1; //TX one message to confirm that is the correct port
            break;
        case 1:
            ThresholdT[0] = msgHolder[shift+1]*4;
            ThresholdR[0] = msgHolder[shift+2]*4;
            ThresholdDerT[0]= msgHolder[shift+3]*4;
            ThresholdDerR[0]= msgHolder[shift+4]*4;
            
            Save_EEPROM(ThresholdT[0],tbloffset_thrT);
            Save_EEPROM(ThresholdR[0],tbloffset_thrR);
            Save_EEPROM(ThresholdDerT[0],tbloffset_thrderT);
            Save_EEPROM((-1*ThresholdDerR[0]),tbloffset_thrderR);
           
            outBuffer1[10]= ThresholdT[0]/4;
            outBuffer1[11]= ThresholdR[0]/4;
            outBuffer1[12]= ThresholdDerT[0]/4;
            outBuffer1[13]= ThresholdDerR[0]/4;
            break;
        case 2:
            ThresholdT[1]= msgHolder[shift+1]*4;
            ThresholdR[1]= msgHolder[shift+2]*4;
            ThresholdDerT[1]= msgHolder[shift+3]*4;
            ThresholdDerR[1]= msgHolder[shift+4]*4;
            
            Save_EEPROM(ThresholdT[1],tbloffset_thrT+2);
            Save_EEPROM(ThresholdR[1],tbloffset_thrR+2);
            Save_EEPROM(ThresholdDerT[1],tbloffset_thrderT+2);
            Save_EEPROM((-1*ThresholdDerR[1]),tbloffset_thrderR+2);
                        
            outBuffer1[17]= ThresholdT[1]/4;
            outBuffer1[18]= ThresholdR[1]/4;
            outBuffer1[19]= ThresholdDerT[1]/4;
            outBuffer1[20]= ThresholdDerR[1]/4;
            break;
        case 4:
            // bluetooth setup 
            // to - do 
            
        case 6:
            if (msgHolder[1]<=nSensors){   //message is valid              
                ActiveSens = msgHolder[shift+1];
                MotFlag = msgHolder[shift+2];
                StimTime = msgHolder[shift+3]; 
               
                Save_EEPROM(ActiveSens,tbloffset_nSens);
                Save_EEPROM(MotFlag,tbloffset_MotFlag);  //change the state of motors (on/off)
                Save_EEPROM(StimTime,tbloffset_StimTime);  //change the stimulation time
                
                outBuffer1[2] = (ActiveSens*7)+8;    //Output message length
                outBuffer1[3] = MotFlag;
                outBuffer1[4] = StimTime;   
            } else {
                // to do
            }
            break;
        case 7:
            StartRX = msgHolder[shift+1];
            break;
    }
    return;
}


////////////////////////////////////////////////////////////////////////////////
//Input:    Channel number in consideration (p)                               //
//Output:   None                                                              //
//Function: This function detect the touch event and the release event        //
//          analyzing the samples inside a window of size windowLenght. It    //
//          raises a digital pin if the value is greater than a certain       //
//          iThresholdT which means a touch event or it resets the pin if     // 
//          the value is lower than iThresholdR which means there is a release//
//          event. Derivative control is also exploited, setting a pin if the //
//          average slope in the window of interest is higher/lower than a    //
//          certain iThresholdDerT/iThresholdDerR for touch/release.          //
////////////////////////////////////////////////////////////////////////////////
void eventDetection(int s){    //s indicates the sensor [0-1]
    int i;
    
    for (i=0; i<wLenght; i++){  //check how many points overcome the threshold inside the wLength window       
        if (iPVDFFilteredBuffer[(StartDetection+i)%ADCBufferLength][s] >= ThresholdT[s]){       // se il valore supera la soglia 
            iCountEventT[s]++;    // incrementa il contatore per il tocco
        }
        else if (iPVDFFilteredBuffer[(StartDetection+i)%ADCBufferLength][s] <= ThresholdR[s]){  // se il valore   inferiore la soglia 
            iCountEventR[s]++;    // incrementa il contatore per il rilascio
        }
    }
    
    if (((iCountEventT[s]>= 6) ) && (tRefractory[s] > tRefractory_max+StimTime)) { // 140 in refractory to really have 150ms
        
        ContactEvent[s] = 1;   // Attivazione DESC
        tRefractory[s] = 0;
        
        MotorsActivation[s] = 1; 
        
        
        if (MotFlag == 1){
            if (s == 0) {
                MOT1=1;
                set_LED('R');
                CountVibr1 = 0;
                redtime = 0;
            }
            if (s == 1) {
                MOT2=1;
                set_LED('B');
                CountVibr2 = 0;
                bluetime = 0;
            }
        }
    } else if (((iCountEventR[s]>= 6)) && (tRefractory[s] > tRefractory_max+StimTime)) {
        ReleaseEvent[s] = 1;               // Attivazione DESC
        tRefractory[s] = 0;

        MotorsActivation[s] = 1;                       
        if(MotFlag == 1){
            if (s == 0) {
                MOT1=1;
                set_LED('G');
                redtime = 0;
                CountVibr1 = 0;
            }
            if (s == 1) {
                MOT2=1;
                set_LED('W');
                bluetime = 0;
                CountVibr2 = 0;
            }
        }
    }
    
    iCountEventT[s] = 0;   // Reset of the counters of values over threshold
    iCountEventR[s] = 0; 
    StartDetection = (StartDetection+wLenght)%ADCBufferLength;  // Move the window inside the buffer
}

////////////////////////////////////////////////////////////////////////////////
//Input:  value is the sample coming out the ADC                              //
//Output: filtered sample                                                     //
//Function: IIR Butterworth 4th order LP filter:                              //
// y[n] = b*(x[n]+2x[n-1]+x)-(-a1y[n-1]+a2y[n-2])//
//			a = 0.239 --> <<10 --> 0d245 = 0xF5;                              //
//			b =  0.522 --> <<10 --> 0d534 = 0x216;                            //
////////////////////////////////////////////////////////////////////////////////
unsigned int LP_filter(unsigned int i_x0){

    unsigned int i_y0 = 0;

    //  Old LP filter  // not very selective of the noise, too wide
    
//    unsigned int a = 0xF5;
//    unsigned int b = 0x216;
//    unsigned long aux1 = 0;
//    unsigned long aux2 = 0;
//    unsigned long aux3 = 0;
//    
//    y0 = ((unsigned long)a * (x0 + x_1))>>10 + ((unsigned long)b * y_1)>>10;
//    aux1 = ((unsigned long)a * (i_x0 + x_1[p]))>>10 ;
//    aux2 = ((unsigned long)b * y_1[p])>>10;
//    aux3 = aux1+aux2;
//    i_y0 = (unsigned int)aux3;
//    x_1[p] = i_x0;
//    y_1[p] = i_y0;
    
    
    // Butterworth 4th  // It is too computationally heavy to be implemented in our application
   
//    float b = 0.0000079446;    
//    float a1 = 3.71274;   
//    float a2 = 5.17878;
//    float a3 = 3.21604;   
//    float a4 = 0.75014;
//    
//    float aux1 = 0;
//    float aux2 = 0;
//    float aux3 = 0;
//    
//    aux1 = b*(float)(i_x0+4*x_1[p]+6*x_2[p]+4*x_3[p]+x_4[p]);
//    aux2 = a2*y_2[p]+a4*y_4[p]-a1*y_1[p]-a3*y_3[p];
//	  aux3 = aux1-aux2;
//    i_y0 = (unsigned int)aux3;
//    x_4[p] = x_3[p];
//	  y_4[p] = y_3[p];
//    x_3[p] = x_2[p];
//	  y_3[p] = y_2[p];
//    x_2[p] = x_1[p];
//	  y_2[p] = y_1[p];
//	  x_1[p] = i_x0;    
//	  y_1[p] = aux3;
    
    
    // Moving average  // works fine, 18 should be the best but too big of a window maybe
    
//    unsigned int i = 0;   
//    unsigned int average= 0;
//    //average[p] = average[p] - movavg[p][windowLenght-1];
//    for (i=0; i<windowLenght-2; i++){
//        movavg[p][i+1] = movavg[p][i];
//        average = average + movavg[p][i];
//    }
//    average = average + i_x0;
//    movavg[p][0] = i_x0;
//    i_y0 = (unsigned int) average/windowLenght;
    
    
    // Butterworth 2nd 17 Hz // Even faster than moving average, more selective
    
    unsigned long b = 3;    
    unsigned long a1 = 1889;   
    unsigned long a2 = 875;

    unsigned long aux1=0;
    unsigned long aux2=0;
    unsigned long aux3=0;
    long aux4=0;
    
    aux1 = (b*((unsigned long)(i_x0) + (unsigned long)(x_2[p]) + (2*(unsigned long)(x_1[p]))))/1024;
    aux2 = (a1 * y_1[p])/1024;
    aux3 = (a2 * y_2[p])/1024;
	aux4 = aux1+aux2-aux3;
    if (aux4<0) aux4 = 0;
    i_y0 = (unsigned int)aux4;
    x_2[p] = x_1[p];
	y_2[p] = y_1[p];
	x_1[p] = i_x0;
	y_1[p] = aux4;
    
    
    // Notch filter 50 Hz
//    unsigned long b =  878; 
//    unsigned long b1 = -1671; 
//    unsigned long b2 =  878; 
//    unsigned long a1 = -1671;   
//    unsigned long a2 =  733;
//    
//    unsigned long aux1=0;
//    unsigned long aux2=0;
//    unsigned long aux3=0;
//    
//    aux1 = (b*(unsigned long)(i_x0) + b1*(unsigned long)(x_1[p])+ b2*(unsigned long)(x_2[p]))/1024;
//    aux2 = (a1 * y_1[p]+a2 * y_2[p])/1024;
//    aux3 = aux1-aux2;
//    i_y0 = (unsigned int)aux3;

	return i_y0;
}

void sr_LED_primary(char led, bool state)
{
  if (!state)
    switch (led)
    {
    case ('R'):
      REDLED=1;
      return;
    case ('G'):
      GREENLED=1;
      return;
    case ('B'):
      BLUELED=1;
      return;
    default:
      return;
    }
  else
    switch (led)
    {
    case ('R'):
      REDLED=0;
      return;
    case ('G'):
      GREENLED=0;
      return;
    case ('B'):
      BLUELED=0;
      return;
    default:
      return;
    }
}

void set_LED(char led)
{
  switch (led)
  {
  case ('R'):
    sr_LED_primary('R', 1);
    sr_LED_primary('G', 0);
    sr_LED_primary('B', 0);
    return;
  case ('G'):
    sr_LED_primary('R', 0);
    sr_LED_primary('G', 1);
    sr_LED_primary('B', 0);
    return;
  case ('B'):
    sr_LED_primary('R', 0);
    sr_LED_primary('G', 0);
    sr_LED_primary('B', 1);
    return;
  case ('Y'):
    sr_LED_primary('R', 1);
    sr_LED_primary('G', 1);
    sr_LED_primary('B', 0);
    return;
  case ('C'):
    sr_LED_primary('R', 0);
    sr_LED_primary('G', 1);
    sr_LED_primary('B', 1);
    return;
  case ('M'):
    sr_LED_primary('R', 1);
    sr_LED_primary('G', 0);
    sr_LED_primary('B', 1);
    return;
  case ('W'):
    sr_LED_primary('R', 1);
    sr_LED_primary('G', 1);
    sr_LED_primary('B', 1);
    return;
  default:
    sr_LED_primary('R', 0);
    sr_LED_primary('G', 0);
    sr_LED_primary('B', 0);
    return;
  }
}


int main(int argc, char** argv){
    // Initialization of micro and peripheral      
    init_mcu();
    // uart 1 buffers
    b_inBuffer1 = initBuffer(inBuffer1,sizeof(inBuffer1)/sizeof(inBuffer1[0]));
    b_outBuffer1 = initBuffer(outBuffer1,sizeof(outBuffer1)/sizeof(outBuffer1[0]));
        
    // Uart 2 buffers
    b_inBuffer2  = initBuffer(inBuffer2,sizeof(inBuffer2)/sizeof(inBuffer2[0]));
    b_outBuffer2 = initBuffer(outBuffer2,sizeof(outBuffer2)/sizeof(outBuffer2[0]));
    
    // Read last data saved in the EE Data Memory (EEPROM) before initializing the output buffer 
    TBLPAG = 0x7F;  //Select the page to point (check EE Data Memory)
    
    tbloffset_thrT = __builtin_tbloffset(&eeThresholdT[0]); //initialize lower word of address
    tbloffset_thrR = __builtin_tbloffset(&eeThresholdR[0]);  //address offset in the EEPROM
    tbloffset_thrderT = __builtin_tbloffset(&eeThresholdDerT[0]);
    tbloffset_thrderR = __builtin_tbloffset(&eeThresholdDerR[0]);
    tbloffset_nSens = __builtin_tbloffset(&eeUsedSensors);
    tbloffset_SendDer = __builtin_tbloffset(&eeSendDer);
    tbloffset_StimTime = __builtin_tbloffset(&eeStimTime);
    tbloffset_MotFlag = __builtin_tbloffset(&eeMotFlag);
    
    //Copy variables from EEPROM on local variables at the selected address (thresholds are read in the init_buffer function)
    
    ActiveSens = __builtin_tblrdl(tbloffset_nSens);    
    StimTime = __builtin_tblrdl(tbloffset_StimTime);
    MotFlag = __builtin_tblrdl(tbloffset_MotFlag);
    
    int i;
    for (i=0;i<ActiveSens;i++){
        int eeprom_pos = i*2;
        ThresholdT[i] = __builtin_tblrdl(tbloffset_thrT+eeprom_pos);
        ThresholdR[i] = __builtin_tblrdl(tbloffset_thrR+eeprom_pos);
        ThresholdDerT[i] = __builtin_tblrdl(tbloffset_thrderT+eeprom_pos);
        ThresholdDerR[i] = __builtin_tblrdl(tbloffset_thrderR+eeprom_pos);
    }
   
    tmr1_init();
    
    
    
    DEBUG_PIN = 0;
    
    init_ADC();
    
    init_uart();
    init_buffer(); // sets the default values in output message
    init_buffer2();
    
	// Timer init
	TMR1_INT_ENABLE = 1;            // Enable interrupt of Timer1
    TMR1_ENABLE = 1;                // Enable Timer1
     
    IEC0bits.U1TXIE = 1;
    
    circleClrs = 1;
    colorTime = 100;
    send_uart(bt_state);
    bt_state = init_BT();
    send_uart(bt_state);
    if(bt_state == BT_POWER_ON){
        bt_state = whoAmI(BTMAC_pcb);
        send_uart(bt_state);
        bt_state = connect_BT(BTMAC_computer);
        send_uart(bt_state);
    }
    
    while (1){
        copyBuffer(w);
        

        if (icheckEvent==1){
            icheckEvent = 0;
            int p2;
            for(p2=0; p2<ActiveSens; p2++) {
                eventDetection(p2);   // rileva l'evento (nel nostro caso fino a1 massimo in una window di acquisizione del ADC buffer)
            }
        }
        
        // CountVibr1 and 2 are in msec (frequency of timer1)
        if (CountVibr1 > StimTime)  {            
            MOT1 = 0;
            MotorsActivation[0] = 0;
        }
        if (CountVibr2 > StimTime)  {
            MOT2 = 0;
            MotorsActivation[1] = 0;
        }
        
        
        //--- Debugging tools
        // [1] mimic bluetooth interface on uart1
        // used for debugging
        /*
        if(b_inBuffer1.head > 7){
            while(!b_inBuffer1.isEmpty){
              deq(&msgData2,&b_inBuffer1); // copy from 2
              enq(&msgData2,&b_inBuffer2); // put in 1
            }
            processMsgBluetooth(&b_inBuffer2, 0x41);
        }
        */
        // [2] echo2To1
        // echo from 2 to 1
        /*
        while(!b_inBuffer2.isEmpty){
            //set_LED('R');
            deq(&msgData2,&b_inBuffer2); // copy from 2
            enq(&msgData2,&b_outBuffer1); // put in 1
        }
        */
        //--- end of Debugging tools
        
        // check if valid messages are recieved 
        processMsg(&b_inBuffer1);
        processMsgBluetooth(&b_inBuffer2, 0x84);
        
        if(b_inBuffer1.msgCount >= 1){
            getMsg(&b_inBuffer1, msgData1,&msgDataSize1);
            ReadCmd(msgData1);
        }
        
        if(b_inBuffer2.msgCount >= 1){
            getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
            ReadCmd(msgData2);
        }
        
        
        if (SendData==1 && b_outBuffer1.isEmpty ){
            // every 100Hz
            SendData=0;
            if(StartRX) ManageSerialTX();
            if(bt_state == BT_CONNECTED)   ManageSerialTX2();
        }
        
        // check if there are bytes to send.
        if(!b_outBuffer1.isEmpty){
            deq(&tempByte, &b_outBuffer1);
            send_uart(tempByte);
        }
        
        if(!b_outBuffer2.isEmpty){
            deq(&tempByte, &b_outBuffer2);
            send_uart2(tempByte);
        }
        

        // this is non-blocking LED blink.
        /*
        if ((circleClrs > 0) && !(T1counter % colorTime) && !LED_switched && !blockLED){
            // little dance
            set_LED(colors[colorCounter++]);
            // DEBUG_PIN = !DEBUG_PIN;
            if(!(colorCounter % 8)) {
                circleClrs--;
                colorCounter = 0;
            }
            LED_switched = true;
        }
        */
    }
	return (EXIT_SUCCESS);
}


bt_states init_BT(){
    unsigned char msgsAfter =  b_inBuffer2.msgCount + 1;

    BT_RESET = 0;
    __delay_ms(10); //BT user manual (pag. 33)
    BT_RESET = 1;
    
    const char expectedResponse[7] = {0x02, 0x41, 0x02, 0x00, 0x01, 0x01, 0x41};
    
    unsigned int timeOut = addmsec(100);
    
    while(b_inBuffer2.msgCount != msgsAfter || T1counter < timeOut){
       processMsgBluetooth(&b_inBuffer2, 0x41);
    }
    
    while(b_inBuffer2.msgCount > 0){
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(!memcmp(msgData2,expectedResponse,msgDataSize2)){ // success
            return(BT_POWER_ON);
        }
    }
    return(BT_POWER_OFF);
}

bt_states connect_BT(uint8_t *BTMAC){
    
    unsigned char msgsAfter =  b_inBuffer2.msgCount + 1;
    
    outBuffer2[0]=0x02;
    outBuffer2[1]=0x06;
    outBuffer2[2]=0x06;
    outBuffer2[3]=0x00;
    outBuffer2[4]= BTMAC[0];//0x5D; //dongle BT MAC
    outBuffer2[5]= BTMAC[1];//0xE4; //dongle BT MAC
    outBuffer2[6]= BTMAC[2];//0x32; //dongle BT MAC
    outBuffer2[7]= BTMAC[3];//0xDA; //dongle BT MAC
    outBuffer2[8]= BTMAC[4];//0x18; //dongle BT MAC
    outBuffer2[9]= BTMAC[5];//0x00; //dongle BT MAC
    outBuffer2[10]=calcCS_array(outBuffer2, 10); //checksum
        
    for (int i = 0; i<11; i++) {
      send_uart2(outBuffer2[i]);
    }
    
    
    unsigned int timeOut = addmsec(1000);
    while(b_inBuffer2.msgCount != msgsAfter || T1counter < timeOut){
       processMsgBluetooth(&b_inBuffer2, 0x86);
    }
    
    while(b_inBuffer2.msgCount>0){ 
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[2] == 7 && !memcmp(&msgData2[5],BTMAC,6)){ // success
            return(BT_CONNECTED);
        }
    }
    return(BT_NOT_CONNECTED);
}

bt_states whoAmI(uint8_t *BTMAC){
    unsigned char msgsAfter =  b_inBuffer2.msgCount + 1;
    
    const uint8_t CMD_GET_REQ[6] = {0x02,0x10,0x01,0x00,0x04,0x17};
    for (int i = 0; i<6; i++) {
      send_uart2(CMD_GET_REQ[i]);
    }
    unsigned int timeOut = addmsec(100);
    
    while(b_inBuffer2.msgCount != msgsAfter || T1counter < timeOut){
       processMsgBluetooth(&b_inBuffer2, 0x50);
    }

    while(b_inBuffer2.msgCount > 0 ){
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[1] == 0x50 && msgData2[2] == 0x07){// got a BTMAC response
            for(int i = 0; i < 5;i++){
                BTMAC[i] = msgData2[i+5];
            }
            return(BT_OK);
        }
    }
    return(BT_NOT_OK);
}

bt_states setJustWorks(void){
    unsigned char msgsAfter =  b_inBuffer2.msgCount + 1;
    const uint8_t CMD_SET_REQ_JW[7] = {0x02, 0x11, 0x02, 0x00, 0x0C, 0x02, 0x1F};
    for (int i = 0; i<7; i++) {
      send_uart2(CMD_SET_REQ_JW[i]);
    }
    
    unsigned int timeOut = addmsec(100);
    while(b_inBuffer2.msgCount != msgsAfter || T1counter < timeOut){
       processMsgBluetooth(&b_inBuffer2, 0x51);
    }
    
    while(b_inBuffer2.msgCount>0){ 
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[1] == 0x51 && msgData2[2] == 0x01){// got a BTMAC response
            return(BT_OK);
        }
    }
    return(BT_NOT_OK);
}

void init_buffer2(void) { 
    // follows CMD_DATA_REQ in proteusIII manual
    // 0x02 0x04 LENGTH[2 bytes] PAYLOAD[LENGTH bytes] CS[1Byte] // 
    
    // start + command bytes
    outBuffer2[0] = 0x02;     
    outBuffer2[1] = 0x04;
    
    // Length
    outBuffer2[2] = 0x17;     // LENGTH BYTE 1 LSB
    outBuffer2[3] = 0x00;     // LENGTH BYTE 2 MSB
    
    // Payload
    outBuffer2[4] = 0xA1;
    outBuffer2[5] = 0xA2;
    outBuffer2[6] = (ActiveSens*7)+9;
    outBuffer2[7] = MotFlag;
    outBuffer2[8] = StimTime;
    outBuffer2[14] = ThresholdT[0]/4;      // the threshold voltage sensor 1  - read from the EEPROM
    outBuffer2[15] = ThresholdR[0]/4;
    outBuffer2[16] = ThresholdDerT[0]/4;
    outBuffer2[17] = (char)-1*ThresholdDerR[0]/4;
    outBuffer2[21] = ThresholdT[1]/4;     // the threshold voltage sensor 2  - read from the EEPROM
    outBuffer2[22] = ThresholdR[1]/4;
    outBuffer2[23] = ThresholdDerT[1]/4;
    outBuffer2[24] = (char)-1*ThresholdDerR[1]/4;
    outBuffer2[25] = 0xA2;
    outBuffer2[26] = 0xA1;  
    
    // check-sum
    outBuffer2[27] = calcCS_array(outBuffer2,27);
}

void ManageSerialTX2(void){
    
    outBuffer2[9] = (Timer & 0xFF00) >> 8;   // the voltage level read at 100Hz
    outBuffer2[10] = (Timer & 0x00FF);
    for (uint8_t j=0; j<ActiveSens; j++){
        aux = 7*j;
        outBuffer2[11+aux] = MotorsActivation[j];     // For each sensor/motor we want to send its state of activation
        outBuffer2[12+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0xFF00) >> 8;   // the voltage level read at 100Hz
        outBuffer2[13+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0x00FF);
    }
    outBuffer2[27] = calcCS_array(outBuffer2,27);
    b_outBuffer2.head = 0;
    b_outBuffer2.isEmpty = false;
    b_outBuffer2.isFull  = true;
}


