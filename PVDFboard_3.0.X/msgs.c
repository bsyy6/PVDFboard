/* created by Waleed - 2024 
    A simple message library that takes a buffer (check buffers.h) and processes the messages in it.
    it discards any bytes that don't belong to a message.
    keeps messages on the buffer and marks the start and end of each message.
*/

#include "msgs.h"
#include "buffers.h"

void processMsg(volatile Buffer *raw_buffer){
    
    // Reads through the buffer: if a valid message is found it:
    // [1] freezes its locaiton in the buffer so it is not overwritten.
    // [2] stores the location of the message in the buffer->msgRanges
    
    // to retrieve the message, use the 
    // void getMsg(volatile Buffer *buffer, uint8_t* msgOut); 
    // it will copy the message to msgOut array and unfreeze the buffer to work as usual.
    // you can check if messages are available by checking buffer->msgCount.

    typedef enum {
        START1, // waiting for first flag
        START2, // waiting for second flag       
        SIZE, // checking size
        DATA, // getting data
        END1,
        END2,
        ERROR, // error
    } State;
    
    // message structure => [firstStartFlag,seocondStartFlag,Size,{DATA[]},firstEndFlag,firstStartFlag]
    const uint8_t firstStartFlag = 0xA1;
    const uint8_t secondStartFlag = 0xA2;
    const uint8_t firstEndFlag = 0xA2;
    const uint8_t secondEndFlag = 0xA1;
    const uint8_t MSG_ADDED_BYTES = 5; // 2 start flags + 2 end flags + 1 size byte
    const uint8_t MAX_MSG_SIZE = 15;
    const uint8_t MIN_MSG_SIZE = 6;

    static uint8_t byte;
    static State state = START1;
    static uint8_t msgBytesLeft = 0;

    if(raw_buffer->isEmpty){
        return;
    }
    while(!raw_buffer->isEmpty){
        deq(&byte,raw_buffer);
        switch (state){
            case START1:
                if(byte == firstStartFlag){
                    state = START2;
                    msgBytesLeft = 0;
                    setMsgStart(raw_buffer);
                }else{
                    state = ERROR;
                }
                break;
            case START2:
                if(byte == secondStartFlag){
                    state = SIZE;
                }else{
                    state = ERROR;
                }
                break;
            case SIZE:
                if(byte > MAX_MSG_SIZE || byte < MIN_MSG_SIZE){
                    state = ERROR;
                }else{
                    msgBytesLeft = byte - (MSG_ADDED_BYTES);
                    state = DATA;
                }
                break;
            case DATA:
                msgBytesLeft--; 
                if (msgBytesLeft == 0){
                    state = END1;
                }
                break;
            case END1:
                if(byte == firstEndFlag){
                    state = END2;
                }else{
                    state = ERROR;
                }
                break;
            case END2:
                if(byte == secondEndFlag){
                    enqMsg(raw_buffer);
                    state = START1;
                }else{
                    state = ERROR;
                }
                break;
            case ERROR:
                break;
        }
        
        if(state == ERROR){
            if(findNextMsgStart(raw_buffer)){
                jumpToMsgStart(raw_buffer);
                delRange(raw_buffer, 0, raw_buffer->tail-1, true);
            }else{
                removeMsgStart(raw_buffer); // free up the buffer for overwrites.
                delRange(raw_buffer, 0, raw_buffer->tail-1, true);
            }
            state = START1;
        }   
    }
}

void processMsgBluetooth(volatile Buffer *raw_buffer, uint8_t secondStartFlag){
    // Reads through the buffer: if a valid message is found it:
    // [1] freezes its locaiton in the buffer so it is not overwritten.
    // [2] stores the location of the message in the buffer->msgRanges
    
    // to retrieve the message, use the 
    // void getMsg(volatile Buffer *buffer, uint8_t* msgOut); 
    // it will copy the message to msgOut array and unfreeze the buffer to work as usual.
    // you can check if messages are available by checking buffer->msgCount.
    
    
    // message structure => [firstStartFlag,seocondStartFlag,Size,{DATA[]},firstEndFlag,firstStartFlag]
    
    typedef enum {
        START1, // waiting for first flag
        START2, // waiting for second flag       
        SIZE1, // checking BT_size
        SIZE2, // checking BT_size
        DATA, // getting data
        CS,   // waiting for checksum
        ERROR, // error
    } State_BT; 

    const uint8_t firstStartFlag = 0x02;
    const uint8_t MAX_MSG_SIZE = 70;
    const uint8_t MIN_MSG_SIZE = 1;


    static uint8_t byte;
    static State_BT state = START1; // bluetooth message states
    static uint16_t msgBytesLeft = 0;
    static uint16_t msgSize = 0;
    static uint8_t checkSumFlag = 0x00;


    if(raw_buffer->isEmpty){
        return;
    }
    bool foundMsg = false; // Quit checking the buffer upon finding a message. 
    while(!raw_buffer->isEmpty && !foundMsg){
        deq(&byte,raw_buffer);
        switch (state){
            case START1:
                if(byte == firstStartFlag){
                    state = START2;
                    msgBytesLeft = 0;
                    setMsgStart(raw_buffer);
                }else{
                    state = ERROR;
                }
                break;
            case START2:
                if(byte == secondStartFlag){
                    state = SIZE1;
                }else{
                    state = ERROR;
                }
                break;
            case SIZE1:
                // the LSB of the size
                msgBytesLeft = byte;
                msgSize = msgBytesLeft; // used later to calcualte the checksum
                state = SIZE2;
                break;
            case SIZE2:
                // the MSB of the size
                msgBytesLeft |= (byte << 8);

                if(msgBytesLeft > MAX_MSG_SIZE || msgBytesLeft < MIN_MSG_SIZE){
                    state = ERROR;
                }else{
                    state = DATA;
                }
                break;
            case DATA: // aka payload
                msgBytesLeft--; 
                if (msgBytesLeft == 0){
                    state = CS;
                    // jump back to the start
                    jumpToMsgStart(raw_buffer);
                    checkSumFlag = calcCS_buffer(raw_buffer, msgSize+4); // +4 for the flags and size bytes
                }
                break;
            case CS:
                if(byte == checkSumFlag){
                    enqMsg(raw_buffer);
                    state = START1;
                    foundMsg = true;
                }else{
                    state = ERROR;
                }
                break;
            case ERROR:
                break;
        }
        
        if(state == ERROR){
            if(findNextMsgStart(raw_buffer)){
                jumpToMsgStart(raw_buffer);
                delRange(raw_buffer, 0, raw_buffer->tail-1, true);
            }else{
                removeMsgStart(raw_buffer); // free up the buffer for overwrites.
                delRange(raw_buffer, 0, raw_buffer->tail-1, true);
            }
            state = START1;
        }
    }
}

uint8_t calcCS_buffer(volatile Buffer* buffer, uint8_t size){
    // calculates the checksum of size bytes in the buffer
    // starting from tail positon to size.
    if(size > howMuchData(buffer)){
        return 0;
    }
    
    uint8_t cs = 0;
    uint8_t b = 0;
    for(uint8_t i = 0; i < size; i++){
        deq(&b, buffer);
        cs^=b;
    }
    return cs;
}

uint8_t calcCS_array(uint8_t* arr, uint8_t n){
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < n; i++) {
        checksum ^= arr[i];
    }
    return(checksum);
}