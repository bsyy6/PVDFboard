
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
#include <stdio.h>                           
#include <math.h> 
#include <stdlib.h>
#include "HeaderPVDF.h"
#include "xc.h"
#include <libpic30.h>
#include <stdbool.h> 



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


// DEFINE PARAMETERS
#define wLenght        10   // lunghezza del buffer di acquisizione dell'adc, acquisendo 1 campione ogni 1 ms, sarà pieno dopo 10 ms visto che legge un campione per sensore 
#define windowLenght   10   // Event detection window length, 10ms
#define nSensors       2    // number of sensors to read
#define DutyCycleT     0x09   //duty cycle per attivare i motori di uno spike nel caso di evento di tocco 80%
#define DurationT      0x0F   // durata dello spike per l'evento di tocco 150ms
#define DutyCycleR     0x9    //duty cycle per attivare i motori di uno spike nel caso di evento di rilascio 80%
#define DurationR      0x0F   // durata dello spike per l'evento di tocco 150ms
#define ADCBufferLength 20    // ADC Buffer length, 20ms 

// VARIABLES

bool state = true;
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
unsigned int __attribute__ ((space(eedata))) eeStimTime = 50; // 0x32

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

// SERIAL COMMUNICATION
#define OutputBufferLength 23           //Serial command to be transmitted to interface (43 max length with 5 sensors)
#define InputBufferLength  10 //15      //Serial command to be received from interface
unsigned char OutputBuffer[OutputBufferLength] = {0};  
volatile unsigned char InputBuffer[10] = {0};  

unsigned char header[2] = {0xA1,0xA2};               
unsigned char tail[2] = {0xA2,0xA1};

unsigned char checkCOM = 0;
unsigned char StartRX = 0;             // wait StartRX=1 for enabling serial communication
volatile unsigned char ind=0;                   // shifts RX buffer
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
void ReadCmd(int start, char command_type);
void init_buffer(void);
void sr_LED_primary(char led, bool state);
void set_LED(char led);

//Handler interrupt
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){ //@1KHz
    
    count++;                            // Every 1 ms T1 interrupt increase count

    CountVibr1++;
    CountVibr2++;

    bluetime++;
    redtime++;
    
    if (count == 10) //makes it 100Hz 
    {    
        count = 0;
        SendData=1;   //100Hz
        Timer++;
    }
    unsigned int q ; 
    for (q=0; q<ActiveSens; q++)
    {
        // to avoid double detection of the same contact event
        tRefractory[q]++;
    }
    
    // ADC enable to sample the analog input
    p = 0;
    IFS0bits.AD1IF = 0;
    IEC0bits.AD1IE = 1;  // Enable the interrupts while reading all five sensors


    configure_sequence_MUXA(p,p+1);   // Create the sequence of channels for MUXA to read
    AD1CON1bits.ASAM = 1;      // Start the sampling of the channels in sequence 
    AD1CON1bits.ADON = 1;      // We can only activate the ADC here because to change the settings it must be off 
    while (!IFS0bits.AD1IF){}  // We wait for the reading to finish
    AD1CON1bits.ADON = 0;
    IFS0bits.AD1IF = 0;
    PVDFsensor[p] = ADC1BUF0;         // Read the AN9 channel conversion result
    iPVDFFiltered[w][p] = LP_filter(PVDFsensor[p]);    // filtro i dati Sensore p e riempio il buffer 
    p++;
    // Fill the vector with data from ADC buffer 2
    PVDFsensor[p] = ADC1BUF1;         // Read the AN15 channel conversion result
    iPVDFFiltered[w][p] = LP_filter(PVDFsensor[p]);    // filtro i dati Sensore p e riempio il buffer 
    p++;



    w++;
    if(w==ADCBufferLength){       //    w=(w+1)%ADCBufferLength;
        w=0;
        CircledBuffer = 1;
    }
    
	IFS0bits.T1IF = 0; 					// Clear Timer1 interrupt flag
   
}
/*
void __attribute__((__interrupt__, no_auto_psv)) _ADC1Intterrupt(void){
 // to do 
}
*/


void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void){
    
//    if (OutByteToSend == OutputBuffer[2]-2){     // recognise when it's time to send the tail
//        U1TXREG = 0xA2; //OutputBuffer[OutputBufferLength-2];
//        U1TXREG = 0xA1; //OutputBuffer[OutputBufferLength-1];
//        IEC0bits.U1TXIE = 0;  // DISABLE THE INTERRUPT since I sent the entire message
//        OutByteToSend = 1;
//        IFS0bits.U1TXIF = 0;
//        return;
//    }
//    else if (OutByteToSend == OutputBuffer[2]-1){
//        U1TXREG = 0xA1;//OutputBuffer[OutputBufferLength-1];
//        IEC0bits.U1TXIE = 0;  // DISABLE THE INTERRUPT since I sent the entire message
//        OutByteToSend = 1;
//        IFS0bits.U1TXIF = 0;
//        return;  
//    }
    
      // U1TXREG = OutputBuffer[0];          //  SEND ONE BYTE AT A TIME but while we transmit we don't wait, maybe we can go in couples
//    OutByteToSend++;
//    
//    if (OutByteToSend == OutputBuffer[2]-2){
//        U1TXREG = 0xA2;
//    }
//    else{
//        U1TXREG = OutputBuffer[OutByteToSend];
//    }
//    OutByteToSend++;
    while(!U1STAbits.TRMT){}
    IFS0bits.U1TXIF = 0;  // Clear TX Interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){   
    
    
    InputBuffer[ind] = (unsigned char) (U1RXREG & 0xFF) ;   // as long as data comes in we save it inside a buffer.
    ind++;
    
    NewDataArrived = 1;
    if (ind==10){
        ind = 0;    
    }
    
    if (U1STAbits.OERR){
        U1STAbits.OERR = 0;
    }

    IFS0bits.U1RXIF = 0;                // Clear TX Interrupt flag
}


void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void){
    while(!U2STAbits.TRMT){}
    IFS1bits.U2TXIF = 0;  // Clear TX Interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void){
    IFS1bits.U2RXIF = 0;  // Clear TX Interrupt flag
}
////////////////////////////////////////////////////////////////////////////////
//Input:    The number of the ADC channels we want to read(s1,s2)             //
//Output:   None                                                              //
//Function: This function modifies the sequence of channels to read           //
////////////////////////////////////////////////////////////////////////////////
void configure_sequence_MUXA(char s1, char s2){
    
    AD1CSSLbits.CSSL0 = 0; //Corresponding analog channel not selected for input scan
    AD1CSSLbits.CSSL13 = 0; //Corresponding analog channel selected for input scan
//    AD1CSSLbits.CSSL14 = 0; //Corresponding analog channel selected for input scan
//    AD1CSSLbits.CSSL15 = 0; //Corresponding analog channel selected for input scan
//    AD1CSSLbits.CSSL9 = 0; //Corresponding analog channel selected for input scan
    
    if (s1 == 0 || s2 == 0) AD1CSSLbits.CSSL0 = 1;    // We activate the channels based on the sensors we want
    if (s1 == 1 || s2 == 1) AD1CSSLbits.CSSL13 = 1;   // to read it should work because at couples the channels 
//  if (s1 == 2 || s2 == 2) AD1CSSLbits.CSSL0 = 1;    //are in increasing order.
//  if (s1 == 3 || s2 == 3) AD1CSSLbits.CSSL13 = 1;   // CHANGE TO L0 AND L13 FOR THE NEW BOARD
//  if (s1 == 4 || s2 == 4) AD1CSSLbits.CSSL15 = 1;
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
    
    OutputBuffer[0] = 0xA1;
    OutputBuffer[1] = 0xA2;
    OutputBuffer[2] = (ActiveSens*7)+9;     // this should be the number of data sent through the usart
    OutputBuffer[3] = MotFlag;
    OutputBuffer[4] = StimTime;
    
    OutputBuffer[10] = ThresholdT[0]/4;      // the threshold voltage sensor 1  - read from the EEPROM
    OutputBuffer[11] = ThresholdR[0]/4;
    OutputBuffer[12] = ThresholdDerT[0]/4;
    OutputBuffer[13] = (char)-1*ThresholdDerR[0]/4;
    OutputBuffer[17] = ThresholdT[1]/4;     // the threshold voltage sensor 2  - read from the EEPROM
    OutputBuffer[18] = ThresholdR[1]/4;
    OutputBuffer[19] = ThresholdDerT[1]/4;
    OutputBuffer[20] = (char)-1*ThresholdDerR[1]/4;
    
    OutputBuffer[21] = 0xA2;
    OutputBuffer[22] = 0xA1;   
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: This function updates the Output Buffer with the right values of  //
//          analog signals to send the interface                              //
////////////////////////////////////////////////////////////////////////////////
void ManageSerialTX(void){
    
    unsigned int j;
    OutputBuffer[5] = (Timer & 0xFF00) >> 8;   // time stamp read at 100Hz
    OutputBuffer[6] = (Timer & 0x00FF);
    for (j=0; j<ActiveSens; j++){
        aux = 7*j;
        OutputBuffer[7+aux] = MotorsActivation[j];     // For each sensor/motor we want to send its state of activation
        OutputBuffer[8+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0xFF00) >> 8;   // the voltage level read at 100Hz
        OutputBuffer[9+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0x00FF);
    }
}

////////////////////////////////////////////////////////////////////////////////
//Input:    None                                                              //
//Output:   None                                                              //
//Function: This function analyzes the Serial Input Buffer to recognise header//
//          and tail of a valid message, and in this case calls ReadCmd to    //
//          perform the relative command.
////////////////////////////////////////////////////////////////////////////////
void ManageSerialRX(void) {

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
}

////////////////////////////////////////////////////////////////////////////////
//Input:    Byte of the Serial Input Buffer from where we start reading(start)//
//          Command type read in the incoming message(command_type)
//Output:   None                                                              //
//Function: This function performs the due changes to respond to the correct  //
//          command type, like changing the threshold for different sensors,  //
//          selecting different size and number of sensors to use, or switch  //
//          between streaming derivative or analog signal.                    //
//////////////////////////////////////////////////////////////////////////////// 
void ReadCmd(int start, char command_type){
    
//    switch (command_type){
//        case 0:
//            checkCOM = 1; //TX one message to confirm that is the correct port
//            break;
//        case 1:
//            Save_EEPROM(InputBuffer[(start+4)%InputBufferLength]*4,tbloffset_thrT);
//            Save_EEPROM(InputBuffer[(start+5)%InputBufferLength]*4,tbloffset_thrR);
//            Save_EEPROM(InputBuffer[(start+6)%InputBufferLength]*4,tbloffset_thrderT);
//            Save_EEPROM((-1*InputBuffer[(start+7)%InputBufferLength]*4),tbloffset_thrderR);
//           
//            ThresholdT[0]=InputBuffer[(start+4)%InputBufferLength]*4;
//            ThresholdR[0]=InputBuffer[(start+5)%InputBufferLength]*4;
//            ThresholdDerT[0]=InputBuffer[(start+6)%InputBufferLength]*4;
//            ThresholdDerR[0]=InputBuffer[(start+7)%InputBufferLength]*4;
//            
//            OutputBuffer[10]= ThresholdT[0]/4;
//            OutputBuffer[11]= ThresholdR[0]/4;
//            OutputBuffer[12]= ThresholdDerT[0]/4;
//            OutputBuffer[13]= ThresholdDerR[0]/4;
//            break;
//        case 2:
//            Save_EEPROM(InputBuffer[(start+4)%InputBufferLength]*4,tbloffset_thrT+2);
//            Save_EEPROM(InputBuffer[(start+5)%InputBufferLength]*4,tbloffset_thrR+2);
//            Save_EEPROM(InputBuffer[(start+6)%InputBufferLength]*4,tbloffset_thrderT+2);
//            Save_EEPROM((-1*InputBuffer[(start+7)%InputBufferLength]*4),tbloffset_thrderR+2);
//            
//            ThresholdT[1]=InputBuffer[(start+4)%InputBufferLength]*4;
//            ThresholdR[1]=InputBuffer[(start+5)%InputBufferLength]*4;
//            ThresholdDerT[1]=InputBuffer[(start+6)%InputBufferLength]*4;
//            ThresholdDerR[1]=InputBuffer[(start+7)%InputBufferLength]*4;
//            
//            OutputBuffer[17]= ThresholdT[1]/4;
//            OutputBuffer[18]= ThresholdR[1]/4;
//            OutputBuffer[19]= ThresholdDerT[1]/4;
//            OutputBuffer[20]= ThresholdDerR[1]/4;
//            break;
//        case 6:
//            if (InputBuffer[(start+4)%InputBufferLength]<=nSensors){   //message is valid              
//                Save_EEPROM(InputBuffer[(start+4)%InputBufferLength],tbloffset_nSens);
//                ActiveSens = InputBuffer[(start+4)%InputBufferLength];
//                OutputBuffer[2] = (ActiveSens*7)+8;    //Output message length
//                
//                Save_EEPROM(InputBuffer[(start+5)%InputBufferLength],tbloffset_MotFlag);  //change the state of motors (on/off)
//                MotFlag = InputBuffer[(start+5)%InputBufferLength];
//                OutputBuffer[3] = MotFlag;
//                
//                Save_EEPROM(InputBuffer[(start+6)%InputBufferLength],tbloffset_StimTime);  //change the stimulation time
//                StimTime = InputBuffer[(start+6)%InputBufferLength]; 
//                OutputBuffer[4] = StimTime;
//            }
//            break;
//        case 7:
//            StartRX = InputBuffer[(start+4)%InputBufferLength];
//            break;
//    }
}

////////////////////////////////////////////////////////////////////////////////
//Input:    Channel number in consideration (p)                               //
//Output:   None                                                              //
//Function: This function detect the touch event and the release event        //
//          analyzing the samples inside a window of size windowLenght. It    //
//          raises a digital pin if the value is greater than a certain       //
//          iThresholdT which means a touch event or it resets the pin if  // 
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
    
    //derivative[s] = (iPVDFFilteredBuffer[(StartDetection+9)%ADCBufferLength][s] - iPVDFFilteredBuffer[StartDetection%ADCBufferLength][s])/2;
    //|| (derivative[s] >= (int)ThresholdDerT[s])
    
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
    }
    
    //|| (derivative[s] <= (int)ThresholdDerR[s])
    else if (((iCountEventR[s]>= 6)) && (tRefractory[s] > tRefractory_max+StimTime)) {
        ReleaseEvent[s] = 1;                              // Attivazione DESC
        tRefractory[s] = 0;

        MotorsActivation[s] = 1;                       // Don't know if put it here to add the condition on the previous status of the contact before sending an activation or not
        
        if(MotFlag == 1){
            if (s == 0) {
                MOT1=1;
                set_LED('R');
                redtime = 0;
                CountVibr1 = 0;
            }
            if (s == 1) {
                MOT2=1;
                set_LED('B');
                bluetime = 0;
                CountVibr2 = 0;
            }
        }
    }
    
    iCountEventT[s] = 0;   // Reset of the counters of values over threshold
    iCountEventR[s] = 0; 
    StartDetection= (StartDetection+wLenght)%ADCBufferLength;  // Move the window inside the buffer
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

void circleColors(unsigned int delays, bool reverse ) {
    char colors[] = {'R', 'G', 'B', 'Y', 'C', 'M', 'W','0'};
    int num_colors = 8;
    int i;
    if(!reverse){
        for (i = 0; i < num_colors; i++) {
            // Set the LED to the current color
            set_LED(colors[i]);
            // Delay before changing to the next color
            __delay_ms(delays);
        }
    }else{
        for (i = num_colors; i > 0; i--) {
            // Set the LED to the current color
            set_LED(colors[i]);
            // Delay before changing to the next color
            __delay_ms(delays);
        }
    }
}

int main(int argc, char** argv){
   
    // Initialization of micro and peripheral      
    init_mcu();
    tmr1_init();
    init_uart();
    init_ADC();   
    
    circleColors(250,1);
    circleColors(250,0);
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
   
    init_buffer();
    
	// Timer init
	TMR1_INT_ENABLE = 1;            // Enable interrupt of Timer1
    TMR1_ENABLE = 1;                // Enable Timer1
    
    AD1CON1bits.SAMP = 1; 

    set_LED('G');
    
    //IEC0bits.U1TXIE = 1;   // ENABLE TX INTERRUPT so we can start the data send routine
    //U1TXREG = 0xA1;// THE FIRST WRITING IN THE TXREG WILL CALL THE FIRST INTERRUPT
    
    //	main's while
	while (1){
        circleColors(10,1);
        if (NewDataArrived==1){ 
            // echo back to two uarts
            send_uart2(InputBuffer[(ind+9)%10]);
            send_uart(ind);
            send_uart(0xFF);
            send_uart(InputBuffer[(ind+9)%10]);
            NewDataArrived = 0;
        }else{
            //send_uart(0xF0);
            //send_uart2(0xAB);
        }        
    }
    
    // non va sotto mai.
    while (1){
        if (NewDataArrived==1){ 
            ManageSerialRX();
        }
        
        if (SendData==1  && (StartRX==1 || checkCOM==1)){
            checkCOM=0;
            SendData=0;
            ManageSerialTX();      
            IEC0bits.U1TXIE = 1;   // ENABLE TX INTERRUPT so we can start the data send routine -> we should enable this only when there is the serial connection with the GUI
            U1TXREG = 0xA1;        // THE FIRST WRITING IN THE TXREG WILL CALL THE FIRST INTERRUPT 
        }

        copyBuffer(w);

        if (icheckEvent==1){
            icheckEvent = 0;
            int p2;
            for(p2=0; p2<ActiveSens; p2++) {
                eventDetection(p2);   // rileva l'evento (nel nostro caso fino a1 massimo in una window di acquisizione del ADC buffer)
            }
        }
        
        if (CountVibr1 > StimTime)  {             // Reset timer for "vibration activation", STIMULUS DURATION = 50ms
            MOT1 = 0;
            MotorsActivation[0] = 0;
        }
        if (CountVibr2 > StimTime)  {
            MOT2 = 0;
            MotorsActivation[1] = 0;
        }
        if (bluetime > tLed_max && redtime > tLed_max)  {
            set_LED('G');
        }
    }
	return (EXIT_SUCCESS);
}


