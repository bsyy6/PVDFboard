/* 

Waleed - 2024
modified msgs library for bluetooth communication:

Differences:
start flags change depending on the message sent.
end flags is now a checksum.

warning: make sure to update the start flags

... let's hope for the best

*/

#include "bt_msgs.h"
#include "buffers.h"

BT_msg initMsg( volatile Buffer *msgs_buffer, volatile Buffer *raw_buffer, volatile Buffer *msgs_idxs, void *msgData){
    BT_msg M;
    M.rawBuffer = raw_buffer;
    M.msgQueue = msgs_buffer;
    M.msgIdx = msgs_idxs;
    // where it outputs the message
    M.msgData = msgData;
    M.msgDataSize = 0;
    //
    M.state = START1;
    M.msgSize = 0;
    M.msgsAvailable = 0;
    M.byte = 0;
    M.prevByte = 0;
    M.state = START1;
    M.prevState = START1;
    M.size = 0;
    M.partCount = 0;

    // set the start flags
    M.firstStartFlag = 0x02;
    M.secondStartFlag = 0x00; // updates based on message sent and expeceted response -> command | 0x40
    M.endFlag = 0x00;         // is the checksum of the whole package sent
    return M;
}


void processMsg(BT_msg *m){
    if(m->rawBuffer->isEmpty){
        return;
    }
    // update history
    m->prevByte  = m->byte;
    m->prevState = m->state;
    while(!m->rawBuffer->isEmpty){
        deq(&m->byte,m->rawBuffer);
        switch (m->state){
            case START1:
                if(m->byte == m->firstStartFlag){
                    m->state = START2;
                    m->msgSize = 0;
                    setBookmark(m->rawBuffer);
                }
                break;
            case START2:
                if(m->byte == m->secondStartFlag){
                    m->state = SIZE1;
                }else{
                    m->state = ERROR;
                }
                break;
            case SIZE1:
                // LSB
                m->msgSize = m->byte;
                m->state = SIZE2; // wait for msb
                break;
            case SIZE2:
                // MSB
                m->msgSize |= m->byte<<8;
                m->partCount = m->msgSize;
                if(m->msgSize > BT_MAX_MSG_SIZE || m->msgSize < BT_MIN_MSG_SIZE){
                    m->state = ERROR;
                }else{
                    m->state = DATA;
                }
                break;
            case DATA:
                // payload
                enq(&m->byte,m->msgQueue); 
                m->msgSize = m->msgSize-1; 
                if(m->msgSize == 0){
                    m->state = CS;
                    jumpToBookmark(m->rawBuffer);
                    // move the tail one byte behind
                    m->rawBuffer->tail = (m->rawBuffer->tail - 1) % m->rawBuffer->arraySize;
                    m->endFlag = calcCS_buffer(m->rawBuffer,m->partCount+4); // payload + 2 start flag bytes + 2 size bytes
                }
                break;
            case CS:
                if(m->byte == m->endFlag){
                    m->msgsAvailable++;
                    removeBookmark(m->rawBuffer);
                    m->state = START1;
                }else{
                    m->state = ERROR;
                }
                break;      
            case ERROR:
                if(findNextBookmark(m->rawBuffer)){
                    m->state = START2;
                    m->prevState = START1;
                    m->byte = m->firstStartFlag;
                    jumpToBookmark(m->rawBuffer);
                }else{
                    removeBookmark(m->rawBuffer);
                }
                if(m->msgSize != 0){ // clear the last two data points in msgIdx
                    rollback(m->msgIdx ,1);
                    rollback(m->msgQueue,m->partCount);
                }
                break;
        }   
    }
}


uint8_t calcCS_buffer(Buffer* buffer, uint8_t size){
    // calculates the checksum of size bytes in the buffer
    // starting from tail positon to size.
    if(size > howMuchData(buffer)){
        return 0;
    }
    
    uint8_t cs = 0;
    uint8_t b = 0; // a temporary byte
    for(uint8_t i = 0; i < size; i++){
        deq(&b, buffer);
        cs^=b;
    }
    return cs;
}

uint8_t getChecksum(uint8_t* arr, uint8_t n){
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < n; i++) {
        checksum ^= arr[i];
    }
    return(checksum);
}


void getMsg(BT_msg *m){
    if(m->msgsAvailable > 0){
        deq(&(m->msgDataSize), m->msgIdx);
        nDeq(m->msgData,m->msgQueue,m->msgDataSize);
        m->msgsAvailable--;
    }
}

