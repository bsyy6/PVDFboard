#include "msgs.h"
#include "buffers.h"

msg initMsg( volatile Buffer *msgs_buffer, volatile Buffer *raw_buffer, volatile Buffer *msgs_idxs, void *msgData){
    
    msg M;
    M.rawBuffer = raw_buffer;
    M.msgQueue = msgs_buffer;
    M.msgIdx = msgs_idxs;

    // where it outputs the message
    M.msgData = msgData;
    M.msgDataSize = 0;
    M.msgsAvailable = 0;

    // where it outputs the message
    M.msgSize = 0;
    M.msgBytesLeft = 0;
    M.byte = 0;
    M.prevByte = 0;
    M.state = START1;
    M.prevState = START1;
    return M;
}

void processMsg(msg *m){
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
                if(m->byte == firstStartFlag){
                    m->state = START2;
                    m->msgBytesLeft = 0;
                    setBookmark(m->rawBuffer);
                }
                break;
            case START2:
                if(m->byte == secondStartFlag){
                    m->state = SIZE;
                }else{
                    m->state = ERROR;
                }
                break;
            case SIZE:
                if(m->byte > MAX_MSG_SIZE || m->byte < MIN_MSG_SIZE){
                    m->state = ERROR;
                }else{
                    m->msgSize = m->byte - (NUM_OF_FLAGS+NUM_OF_EXTRA_BYTES);
                    m->msgBytesLeft = m->msgSize;
                    enq(&m->msgSize, m->msgIdx);
                    m->state = DATA;
                }
                break;
            case DATA:
                // messageStart 
                enq(&m->byte,m->msgQueue);  // add the byte to the message buffer
                m->msgBytesLeft--; 
                if (m->msgBytesLeft == 0){
                    m->state = END1;
                }
                break;

            case END1:
                if(m->byte == firstEndFlag){
                    m->state = END2;
                }else{
                    m->state = ERROR;
                }
                break;
            case END2:
                if(m->byte == secondEndFlag){
                    m->msgsAvailable++;
                    removeBookmark(m->rawBuffer);
                    m->state = START1;
                }else{
                    m->state = ERROR;
                }
                break;
                
            case ERROR:
                if(m->prevState > 2){
                    rollback(m->msgIdx, 1);
                    rollback(m->msgQueue, (m->msgSize-m->msgBytesLeft) );
                }
                if(findNextBookmark(m->rawBuffer)){
                    m->state = START2;
                    m->prevState = START1;
                    m->byte = firstStartFlag;
                    jumpToBookmark(m->rawBuffer);
                }else{
                    removeBookmark(m->rawBuffer);
                    m->prevState = START1;
                    m->state = START1;
                }
                break;
        }   
    }
}

void getMsg(msg *m){
    if(m->msgsAvailable > 0){
        deq(&(m->msgDataSize), m->msgIdx);
        nDeq(m->msgData,m->msgQueue,m->msgDataSize);
        m->msgsAvailable--;
    }
}