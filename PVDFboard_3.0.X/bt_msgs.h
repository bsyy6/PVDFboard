#ifndef BT_MSGS_H
#define BT_MSGS_H

#include "buffers.h"

#define BT_MAX_MSG_SIZE 64
#define BT_MIN_MSG_SIZE 1
typedef enum {
    START1, // waiting for first flag
    START2, // waiting for second flag       
    SIZE1, // checking size
    SIZE2, // checking size
    DATA, // getting data
    CS,   // waiting for checksum
    ERROR, // error
} State;

typedef struct {
    // any time a valid message is detected inside the buffer these values are updated.
    volatile Buffer *msgQueue; // handles the biggest message possible two times
    volatile Buffer *rawBuffer; // raw data buffer
    volatile Buffer *msgIdx;    // where the message sizes are stored.
    
    // where to read the messages from
    // updated using getMsg
    void *msgData; // message data
    uint8_t msgDataSize;
    uint8_t msgsAvailable;
    
    // private variables
    uint16_t msgSize;
    uint8_t startSearch;
    State   state;
    State   prevState;
    uint8_t byte;
    uint8_t prevByte;
    uint8_t size;
    uint8_t msgCount;
    uint16_t partCount;

    // start flags
    uint8_t firstStartFlag;
    uint8_t secondStartFlag;
    uint8_t endFlag;
}BT_msg;



BT_msg initMsg( volatile Buffer *msgs_buffer, volatile Buffer *raw_buffer, volatile Buffer *msgs_idxs, void *msgData);
void processMsg(BT_msg *m);
void getMsg(BT_msg *m);
uint8_t calcCS_buffer(Buffer* buffer, uint8_t size); // calculates CS of buffer until size
uint8_t getChecksum(uint8_t* arr, uint8_t n);       // calcuates CS of an array
#endif // MACRO
