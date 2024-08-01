#include "bluetooth.h"
#include <libpic30.h>

// things in the main 
extern uint8_t msgData2[35];  // extracted msgs go here
extern uint8_t msgDataSize2; // the size of data in msgData
extern uint8_t outBuffer2[28];
extern timeOutObj timeOut;
extern unsigned int count;
extern volatile bool wrapped;
extern volatile Buffer b_inBuffer2;
extern unsigned char StartRX;

bt_states init_BT(){
    BT_RESET = 0;
    __delay_ms(10); //BT user manual (pag. 33)
    BT_RESET = 1;
    
    bt_states temp;
    temp = CMD_RSP_IND(0x41,500); // CMD_RESET_IND
    if(temp != BT_OK) return(temp);
    
    if(msgData2[4] == 1 && msgData2[5] == 1){ // [4] role : 1 -> peripheral, [5] action : 1 -> idle
        return(BT_POWER_ON);
    }
    
    communicateError_BT(BT_NOT_OK,NULL,0);
    return(BT_NOT_OK);
}

bt_states connect_BT(uint8_t BTMAC[6]){
    
    bt_states temp;

    uint8_t msg[11] = {0x02,0x06,0x06,0x00,BTMAC[0],BTMAC[1],BTMAC[2],BTMAC[3],BTMAC[4],BTMAC[5],0x00};
    msg[10] = calcCS_array(msg,10);
    temp = CMD_REQ(msg,11,2000);
    if(temp != BT_OK) return(temp);   
    temp = CMD_RSP_IND(0x86,3000); // CMD_CONNECT_IND
    if(temp != BT_OK) return(temp);
    temp = CMD_RSP_IND(0x88,5000); // CMD_CONNECT_IND
    if(temp != BT_OK) return(temp);

    temp = CMD_RSP_IND(0xC6,3000); // CMD_CONNECT_IND
    if(temp != BT_OK) return(temp);

    if(!memcmp(&msgData2[5],BTMAC,6)) return(BT_CONNECTED);

    communicateError_BT(BT_NOT_OK,msg,11);
    return(BT_NOT_OK);
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
    
    communicateError_BT(BT_NOT_OK,CMD_GET_REQ,6);
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
    
    communicateError_BT(BT_NOT_OK,CMD_SET_REQ_JW,7);
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
    
    communicateError_BT(BT_NOT_OK,CMD_DISCONNECT_REQ,5);
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
    
    communicateError_BT(BT_NOT_OK,CMD_GET_REQ_NAME,6);
    return(BT_NOT_OK);
}

bt_states setConnectionTiming_BT(const uint8_t val){
    // minimum is 7.5 ms when val = 1
    // check datasheet 8.16 RF_ConnectionTiming
    bt_states temp;
    
    uint8_t CMD_SET_REQ_CT[7]={0x02,0x11,0x02,0x00,0x08,val,0x00};
    CMD_SET_REQ_CT[6] = calcCS_array(CMD_SET_REQ_CT,6);
    
    
    temp = CMD_REQ(CMD_SET_REQ_CT,7,1000);
    if(temp != BT_OK) return(temp);
    
    
    communicateError_BT(BT_NOT_OK,CMD_SET_REQ_CT,7);
    return(BT_NOT_OK);
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
       if(timeOutCheck(&timeOut)) {
           communicateError_BT(BT_TIMEOUT,msg, msgSize);
           return(BT_TIMEOUT);
       }
    }

    // return OK when message is recieved and moved to msgData2
    while(b_inBuffer2.msgCount>0){ 
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[1]==(msg[1]|0x40)){
            if (msgData2[4] == 0x00){
                return(BT_OK); // status byte is 0x00 recieved and processed
            }else{
                communicateError_BT(BT_STATUS_NOT_OK,msg, msgSize);
                return(BT_STATUS_NOT_OK);  // break out
            }
        }
    }
    communicateError_BT(BT_NOT_OK,msg, msgSize);
    return(BT_NOT_OK);
}

bt_states CMD_RSP_IND(const uint8_t flag,  uint16_t waitTime){
    // an interface that waits for a message with the second byte equal to flag
    // used for commands that end with CMD_XXX_IND or CND_XXX_RSP -> check manual

    timeOut = timeOutBegin(&count,waitTime,&wrapped);
    while(!processMsgBluetooth(&b_inBuffer2,flag)){
       if(timeOutCheck(&timeOut)) {
            communicateError_BT(BT_TIMEOUT,NULL,0);
            return(BT_TIMEOUT);
       }
    }

    while(b_inBuffer2.msgCount>0){
        getMsg(&b_inBuffer2, msgData2,&msgDataSize2);
        if(msgData2[1]==flag){
            return(BT_OK);
        }
    }

    communicateError_BT(BT_NOT_OK,NULL, 0);
    return(BT_NOT_OK);
}

void communicateError_BT(bt_states bs,const uint8_t *msg, uint8_t msgSize){
    // when error is detected in communication with the bluetooth module:
    // sends on uart1: 
    // 0xA1 0xA2 0xF1 bt_State what was sent         0xA1 0xA2
    // 0xA1 0xA2 0xF2 bt_state last message recieved 0xA2 0xA1
    StartRX = 0; // block the data stream.

    send_uart(0xA1);
    send_uart(0xA2);
    send_uart(0xF1);
    send_uart(bs);
    if(msg!= NULL){
        for (int i = 0; i<msgSize;i++){
            send_uart(msg[i]);
        }
    }
    send_uart(0xA2);
    send_uart(0xA1);

    send_uart(0xA1);
    send_uart(0xA2);
    send_uart(0xF2);
    send_uart(bs);
    for (int i = 0; i<msgDataSize2; i++){
        send_uart(msgData2[i]); // last message recieved
    }
    send_uart(0xA2);
    send_uart(0xA1);

    return;
}