
/*
 * File:   MainPVDF.c
 * Author: Eleonora Vendrame
 *
 * Created on January 3, 2024
 * Rev. 
 */

/* v4.0  
 * Author : Waleed Alghilan, May 2024
 *
 * 
 * UART 1 baudrate 115200 8N1   [PC]
 * UART 2 baudrate 115200 8N1   [Bluetooth]
 * ADC reading 2 channels at 1KHz AN0 and AN13
 * ADC signal filtered with a 4th order Butterworth LP filter
 * 
 * [Fcy] = Fosc/2 = 32Mhz/2 = 16MHz [PPL x4 activated and no prescalers]
 * [Tcy] = 62.5ns
 * 
 * interrupts: 
 * [1] Timer1 1KHz.
 * [2] UART1 RX, TX recieve and send. 
 * [3] ADC interrupt when two channels are read. set to start sampling every Timer1 interrupt (1KHz).
 * 
 * 
 * setup.c is used to initialize the microcontroller and the peripherals.
 * 
 * buffers.c is used to manage data incoming from UART1 and UART2.
 * msgs.c is reading the incoming data in buffers.c and gets the data from them following a spcecifc order.
 *  - processMsgBluetooth is used to handle the incoming data from the bluetooth module. [02 COMMAND SIZE PAYLOAD CS]
 *  - processMsg is used to handle the incoming data from the PC. [A1 A2 SIZE PAYLOAD A2 A1]
 *  - in both cases data recieved in uart are discared (noise) if it fails to follow the communication protocol.
 * timeout.c is used to manage the timeout for the communication.
 * 
 */


#include "setup.h"       // <- MUST BE INCLUDED FIRST
#include "xc.h"
#include "buffers.h"    
#include "msgs.h"
#include "timeout.h"
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
#define ADCBufferLength 20    // ADC Buffer length, 20ms 

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

// variables loaded from EEPROM
bool MotFlag;
int ActiveSens = 2;
int StimTime;
int ThresholdT[nSensors];
int ThresholdR[nSensors];
int ThresholdDerT[nSensors];
int ThresholdDerR[nSensors];

// FUNCTIONS AND VARIALBES 
// timer1
unsigned int count = 0;                  // Counter for Timer1 (increased every match with PR1), Press acquisition counter
const unsigned int count_max = 65000;    // 65 seconds

unsigned int CountVibr1 = 0;
unsigned int CountVibr2 = 0;

unsigned int tRefractory[nSensors] = {0};
unsigned int bluetime = 0;
unsigned int redtime = 0;
unsigned int Timer = 0;                             //Variable to send to SW as timestamp (100Hz)

//ADC
unsigned char w = 0;            // write index. 
unsigned char p = 0;            // channel index.
unsigned char icheckEvent = 0;  // if 1: checks the event.

unsigned int PVDFsensor[nSensors] = {0};                          // Raw Data ADC values for all channels
unsigned int iPVDFFiltered[ADCBufferLength][nSensors]= {{0},{0}}; // Data filtered by the LP filter

int iPVDFFilteredBuffer[ADCBufferLength][nSensors] = {{0},{0}};     
int derivative[nSensors] = {0};   // derivative global variable

unsigned char CircledBuffer = 0;     // for data acquisition
unsigned char CopiedData = 0; 
unsigned char StartPoint = 0;          // Reading of data from input buffers

//  ADC DIGITAL FILTER 
unsigned int x_1[nSensors] = {0};           // starting values for filter
unsigned long y_1[nSensors] = {0};
unsigned int x_2[nSensors] = {0};           // starting values for filter
unsigned long y_2[nSensors] = {0};


void copyBuffer(unsigned int EndPoint);
unsigned int LP_filter(unsigned int x0);

// Touch Release events
unsigned char StartDetection = 0;
void eventDetection(int s);
void activateMotor(uint8_t s);

// DESC FEEDBACK
unsigned char iCountEventT[nSensors] = {0};            // Counter for contact events
unsigned char iCountEventR[nSensors] = {0};            // Counter for release events
unsigned char ContactEvent[nSensors] = {0};            // Indicates that an application of pressure occurred
unsigned char ReleaseEvent[nSensors] = {0};            // Indicates that a release of pressure occurred
unsigned char MotorsActivation[nSensors] = {0};        // Temporary vector of variables to store the information regarding the activation of the 5 sensors to be sent to the motors
unsigned int tRefractory_max = 140;                    // Time to wait before activating motors after an event detection
unsigned int tLed_max = 200;                           //


//communication - general 
unsigned char checkCOM = 0;
unsigned char SendData = 0;  

void ReadCmd(uint8_t* msgHolder);

// UART 
unsigned char StartRX = 0;

uint8_t inBuffer1[20];    // uart1 buffer

uint8_t msgData1[20];     // extracted msgs go here.
uint8_t msgDataSize1 = 0; // the size of data in msgData
volatile Buffer b_inBuffer1; 

uint8_t outBuffer1[23];
volatile Buffer b_outBuffer1;

void msgInit_UART(void);
void msgUpdate_UART(void);
void msgUpdate_BT(void);


// Bluetooth

unsigned char StartBT = 1;

uint8_t inBuffer2[50]; // uart2 buffer
uint8_t msgData2[35];  // extracted msgs go here
uint8_t msgDataSize2 = 0; // the size of data in msgData
volatile Buffer b_inBuffer2;

uint8_t outBuffer2[28];
volatile Buffer b_outBuffer2;

bool flag_BT_reset = false;
unsigned char BT_in = 0;

uint8_t BTMAC_computer[6] = {0x5D,0xE4,0x32,0xDA,0x18,0x00}; // Default: last device connected and saved in EEPROM
                                                             // can be set by user using UART command 0xA1 0xA2 0x0B 0x10 BTMAC[6] 0xA2 0xA1
uint8_t BTMAC_pcb[6]; // gets filled upon when calling whoAmI() function
uint8_t tempByte;

typedef enum {
    BT_POWER_OFF,       // 0
    BT_POWER_ON,        // 1
    BT_CONNECTED,       // 2
    BT_NOT_CONNECTED,   // 3
    BT_OK,              // 4
    BT_NOT_OK,          // 5
    BT_TIMEOUT,         // 6
    BT_STATUS_NOT_OK    // 7  A message response is received but says it failed to execute the command
} bt_states;

bt_states bt_state = BT_POWER_OFF; 
timeOutObj timeOut;

void msgInit_BT(void);
void msgUpdate_BT(void);
// low-level interfaces 
bt_states CMD_REQ(const uint8_t *msg, uint8_t msgSize, uint16_t waitTime); 
bt_states CMD_RSP_IND(const uint8_t flag,  uint16_t waitTime);
// high-level funcitons
bt_states init_BT();
bt_states connect_BT(uint8_t *BTMAC); 
bt_states setJustWorks(void);        
bt_states whoAmI(uint8_t *BTMAC);
bt_states disconnect_BT(void);
bt_states rename_BT(uint8_t* name, uint8_t nameSize);

// LED
unsigned char circleClrs = 0;
const char colors[] = {'R', 'G', 'B', 'Y', 'C', 'M', 'W','0'};
unsigned char  colorCounter = 0;
unsigned int colorTime = 900; 
volatile bool LED_switched = false;
volatile bool blockLED = false;
volatile bool LED_R_B = true;
volatile bool wrapped = false;

void sr_LED_primary(char led, bool state);
void set_LED(char led);


// interrupt routines 
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){ 
    //@1KHz -> 1 msec
    DEBUG_PIN = !DEBUG_PIN;
    
    
    
    if(count%1000 == 0){ // every second
        if(LED_R_B){
            set_LED('R');
            LED_R_B = false;
        }else{
            set_LED('B');
            LED_R_B = true;
        }
    }
    
    LED_switched = false;
    
    CountVibr1++;
    CountVibr2++;

    bluetime++;
    redtime++;
    
    if (count%100 == 0) //makes it 100Hz 
    {   
        if(count == count_max){ // overflow
            count = 0;
            wrapped = !wrapped;
        }
        SendData=1;
        Timer = count;
    }
    
    count++; 
    
    unsigned char q ; 
    for (q=0; q<ActiveSens; q++)
    {
        // to avoid double detection of the same contact event
        tRefractory[q]++;
    }
    
    AD1CON1bits.SAMP = 1; // samples at 1kHz
	
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
        uint8_t byteRecieved;
        byteRecieved = U2RXREG;
        enq(&byteRecieved,&b_inBuffer2);
    }
    if (U2STAbits.OERR){
        U2STAbits.OERR = 0;
    }
    
    IFS1bits.U2RXIF = 0;                // Clear TX Interrupt flag
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
    init_ADC();
    init_uart();
	// Timer init
	TMR1_INT_ENABLE = 1;            // Enable interrupt of Timer1
    TMR1_ENABLE = 1;                // Enable Timer1
     
    IEC0bits.U1TXIE = 1;
    
    circleClrs = 1;
    colorTime = 100;
    
    bt_state = init_BT();
    if(bt_state == BT_POWER_ON){
        bt_state = whoAmI(BTMAC_pcb);
        bt_state = connect_BT(BTMAC_computer);
    }

    msgInit_UART(); // sets the default values in output message
    msgInit_BT();
    send_uart(0xFF);
    send_uart(0xF2);  
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
        if (CountVibr1 > StimTime && MotFlag )  {            
            MOT1 = 0;
            MotorsActivation[0] = 0;
        }
        if (CountVibr2 > StimTime && MotFlag)  {
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
            if(StartRX) msgUpdate_UART(); // 
            if(bt_state == BT_CONNECTED && StartBT) msgUpdate_BT(); //
        }
        
        // check if there are bytes to send.

        timeOut = timeOutBegin(&count,5,&wrapped);
        while(!b_outBuffer1.isEmpty){
            deq(&tempByte, &b_outBuffer1);
            send_uart(tempByte);
           if(timeOutCheck(&timeOut)) break;
        }
        
        timeOut = timeOutBegin(&count,5,&wrapped);
        while(!b_outBuffer2.isEmpty){
            deq(&tempByte, &b_outBuffer2);
            send_uart2(tempByte);
            if(timeOutCheck(&timeOut)) break;
        }
        
        // this is non-blocking LED blink.
        /*
        if ((circleClrs > 0) && !(count % colorTime) && !LED_switched && !blockLED){
            // little dance
            set_LED(colors[colorCounter++]);
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

void msgInit_UART (void){
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
void msgUpdate_UART(void){
    unsigned int j;
    outBuffer1[5] = (Timer & 0xFF00) >> 8;   // time stamp read at 100Hz
    outBuffer1[6] = (Timer & 0x00FF);
    for (j=0; j<ActiveSens; j++){
        uint8_t aux = 7*j;
        outBuffer1[7+aux] = MotorsActivation[j];     // For each sensor/motor we want to send its state of activation
        
        outBuffer1[8+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0xFF00) >> 8;   // the voltage level read at 100Hz
        outBuffer1[9+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0x00FF);
    }
    b_outBuffer1.head = 0;
    b_outBuffer1.tail = 0;
    b_outBuffer1.isEmpty = false;
    b_outBuffer1.isFull  = true;
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
    
    switch (msgHolder[shift]){
        case 0: // check Communication
            checkCOM = 1; //TX one message to confirm that is the correct port
            break;
        case 1: // set thresholds for thumb 
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
        case 2:// set thresholds for index
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
        
        case 6: // setting for feedback
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
        case 7: // send data uart
            StartRX = msgHolder[shift+1];
            break;
        case 8: // send data bluetooth
            StartBT = msgHolder[shift+1];
            break;
        case 9: // connect/disconnect BT
            if(msgHolder[shift+1] == 0x00){
                disconnect_BT();
            }else{
                connect_BT(BTMAC_computer);
            }
            break;
        case 0x0A: // connect to new BTMAC 
            if(msgHolder[shift-1] == 0xC){ // check BTMAC valid
                if(BT_CONNECTED) bt_state = disconnect_BT();
                StartBT = 0;
                bt_state= BT_CONNECTED; //connect_BT(&msgHolder[shift+1]);
                if(bt_state == BT_CONNECTED){
                    StartBT = 0;
                    memcpy(&BTMAC_computer,&msgHolder[shift+1],6);
                    
                    // to-do update eeprom too
                    
                }
            }
            break;
        case 0x0B: // set name
            if(msgHolder[shift-1] == 0xB){ // check name length is 5
                rename_BT(&msgHolder[shift+1],5);
            }
            break;
        default:
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
    
    if (tRefractory[s] > tRefractory_max+StimTime){ 
        // time has passed
        if (iCountEventT[s]>= 6){ 
            ContactEvent[s] = 1;  
            tRefractory[s] = 0;
            MotorsActivation[s] = 1; 
            if (MotFlag == 1) activateMotor(s);
            
        }else if(iCountEventR[s]>= 6) {
            ReleaseEvent[s] = 1;
            tRefractory[s] = 0;
            MotorsActivation[s] = 1;                       
            if(MotFlag == 1)activateMotor(s);  
        }
    }
    
    iCountEventT[s] = 0;   // Reset of the counters of values over threshold
    iCountEventR[s] = 0; 
    StartDetection = (StartDetection+wLenght)%ADCBufferLength;  // Move the window inside the buffer
}

// helper funtion activates motors and resets timers.
void activateMotor(uint8_t s){
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
    // Butterworth 2nd 17 Hz 
    // Even faster than moving average, more selective
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


bt_states init_BT(){
    BT_RESET = 0;
    __delay_ms(10); //BT user manual (pag. 33)
    BT_RESET = 1;

    CMD_RSP_IND(0x41,500); // CMD_RESET_IND
    if(msgData2[4] == 1 && msgData2[5] == 1){ // [4] role : 1 -> peripheral, [5] action : 1 -> idle
        return(BT_POWER_ON);
    }
    return(BT_POWER_OFF);
}

bt_states connect_BT(uint8_t BTMAC[6]){
    
    bt_states temp;

    uint8_t msg[11] = {0x02,0x06,0x06,0x00,BTMAC[0],BTMAC[1],BTMAC[2],BTMAC[3],BTMAC[4],BTMAC[5],0x00};
    msg[10] = calcCS_array(msg,10);
    
    temp = CMD_REQ(msg,11,100);
    if(temp != BT_OK) return(temp);

    temp = CMD_RSP_IND(0x86,100); // CMD_CONNECT_IND
    if(temp != BT_OK) return(temp);

    temp = CMD_RSP_IND(0x88,100); // CMD_CONNECT_IND
    if(temp != BT_OK) return(temp);

    temp = CMD_RSP_IND(0xC6,100); // CMD_CONNECT_IND
    if(temp != BT_OK) return(temp);

    if(!memcmp(&msgData2[5],BTMAC,6)) return(BT_CONNECTED);

    return(BT_NOT_CONNECTED);
}

bt_states whoAmI(uint8_t *BTMAC){
    
    bt_states temp;

    const uint8_t CMD_GET_REQ[6] = {0x02,0x10,0x01,0x00,0x04,0x17};
    
    temp = CMD_REQ(CMD_GET_REQ,6,100);
    if(temp != BT_OK) return(temp);

    if(msgData2[2] == 0x07){// got a BTMAC response
        for(int i = 0; i < 5;i++){
            BTMAC[i] = msgData2[i+5];
        }
        return(BT_OK);
    }
    
    return(BT_NOT_OK);
}

bt_states setJustWorks(void){
    bt_states temp;

    const uint8_t CMD_SET_REQ_JW[7] = {0x02, 0x11, 0x02, 0x00, 0x0C, 0x02, 0x1F};

    temp = CMD_REQ(CMD_SET_REQ_JW,7,100);
    if (temp != BT_OK) return(temp);
    
    if(msgData2[4] == 0x00){ //success
        temp = CMD_RSP_IND(0x41,100);
        if(msgData2[2] == 0x01) return(BT_OK);  
    }
    
    return(BT_NOT_OK);
}

bt_states disconnect_BT(void){
    bt_states temp;

    const uint8_t CMD_DISCONNECT_REQ[5] = {0x02,0x07,0x00,0x00,0x05};

    temp = CMD_REQ(CMD_DISCONNECT_REQ,5,100);
    if (temp != BT_OK) return(temp);
    
    if(msgData2[4] == 0x00){// disconnected 
        return(BT_NOT_CONNECTED);
    }

    return(BT_NOT_OK);
}



bt_states rename_BT(uint8_t* name, uint8_t nameSize){
    
    bt_states temp; 

    // build the message
    uint8_t CMD_SET_REQ_RENAME[11] = {0x02, 0x11, 0x06, 0x00, 0x02, name[0], name[1], name[2], name[3], name[4],0};
    CMD_SET_REQ_RENAME[10] = calcCS_array(CMD_SET_REQ_RENAME,10); // sets the name of the module

    //send the message
    temp = CMD_REQ(CMD_SET_REQ_RENAME,11,100);
    if (temp != BT_OK) return(temp);

    // check rename is success
    const uint8_t CMD_GET_REQ_NAME[6] = {0x02,0x10,0x01,0x00,0x02,0x11}; // Requests name from module
    temp = CMD_REQ(CMD_GET_REQ_NAME,6,100);
    if (temp != BT_OK) return(temp);
   
    if(!memcmp(&msgData2[5],name,5)){
        return(BT_OK);
    }
    return(BT_NOT_OK);
}


void msgUpdate_BT(void){
    // check outBuffer is correct.
    const uint8_t msgStart[4] = {0x02, 0x04, 0x17, 0x00};
    if(memcmp(outBuffer2,msgStart,4)){
        msgInit_BT(); // the outBuffer2 has old data from other commands re-initialize it.
    }

    outBuffer2[9] = (Timer & 0xFF00) >> 8;   // the voltage level read at 100Hz
    outBuffer2[10] = (Timer & 0x00FF);
    for (uint8_t j=0; j<ActiveSens; j++){
        uint8_t aux = 7*j;
        outBuffer2[11+aux] = MotorsActivation[j];     // For each sensor/motor we want to send its state of activation
        outBuffer2[12+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0xFF00) >> 8;   // the voltage level read at 100Hz
        outBuffer2[13+aux] = (iPVDFFilteredBuffer[StartDetection][j] & 0x00FF);
    }
    outBuffer2[27] = calcCS_array(outBuffer2,27);
    b_outBuffer2.head = 0;
    b_outBuffer2.tail = 0;
    b_outBuffer2.isEmpty = false;
    b_outBuffer2.isFull  = true;
}


void msgInit_BT(void) { 
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


bt_states CMD_REQ(const uint8_t *msg, uint8_t msgSize, uint16_t waitTime){

    // An interface that send CMD_XXX_REQ msg and waits for recieve confirmation CMD_XXX_CNF from the bluetooth module

    // - if no correct confimraiton response is receieved within waitTime it returns BT_TIMEOUT
    // - if correct confirmaiton response is recieved it returns BT_OK and stores the message in msgData2

    // a confirmaiton response is a response that has the second byte equal to msg[1]|0x40 and status byte is 0x00 -> check manual
    // else it returns BT_NOT_OK
    
    // send message
    for (int i = 0; i<msgSize; i++) {
       send_uart2(msg[i]);
    }


    // wait for confirmation of reciept 
    timeOut = timeOutBegin(&count,waitTime,&wrapped);
    while(!processMsgBluetooth(&b_inBuffer2, msg[1]|0x40)){
       if(timeOutCheck(&timeOut)) return(BT_TIMEOUT);
    }

    // return OK when message is recieved and moved to msgData2
    while(b_inBuffer2.msgCount>0){ 
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[1]==(msg[1]|0x40)){
            if(msgData2[4] == 0x00){
                return(BT_OK); // status byte is 0x00 recieved and processed
            }
            else{
                return(BT_STATUS_NOT_OK);  // break out
            }
        }
    }

    return(BT_NOT_OK);
}

bt_states CMD_RSP_IND(const uint8_t flag,  uint16_t waitTime){
    // an interface that waits for a message with the second byte equal to flag
    // used for commands that end with CMD_XXX_IND or CND_XXX_RSP -> check manual

    timeOut = timeOutBegin(&count,waitTime,&wrapped);
    while(!processMsgBluetooth(&b_inBuffer2,flag)){
       if(timeOutCheck(&timeOut)) return(BT_TIMEOUT);
    }

    while(b_inBuffer2.msgCount>0){
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[1]==flag) return(BT_OK);
    }

    return(BT_NOT_OK);

}