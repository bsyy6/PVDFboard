
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* buffer header */
// use this to change the buffer size.
#define BUFFER_SIZE 25
#define BLOCK_WHEN_FULL true     // set to true if you want it to stop writing
                                 // on overflow.
                                 // set to false if you want it to overwrite the
                                 // oldest data in buffer on overflow



#define BUFFER_INIT {{0},0,0,1,0} // INITIAL VALUES ALWAYS USE THIS TO START

typedef struct {
    uint8_t data[BUFFER_SIZE];
    uint8_t head;  // write index
    uint8_t tail;  // read  index
    bool isEmpty;  // there is nothing inside
    bool isFull;   // it went to overflow at some point
} Buffer;

// volatile types ( to be used in interrupts )
void writeBuffer(uint8_t byte, volatile Buffer *buffer);
uint8_t readBuffer(volatile Buffer *buffer);



/* message header */

typedef enum {
    A1START, // waiting
    A2START, //        
    SIZE, // checking size
    DATA, // getting data
    A2END,
    A1END,
    ERROR, // error
    ERROR_REPORTED,
} State;

State prevState = ERROR_REPORTED;
State state = A1START;
uint8_t size = 0;
uint8_t count = 0;
uint8_t b = 0;
uint8_t myData[20] = {0x0,0x0,0x0};
uint8_t prevByte = 0;
uint8_t byte  = 0;
uint8_t errorIndex = 0;
bool ciao;

uint8_t U1RXREG[12] = { 161, 161, 162, 11, 242, 162,  // wrong A2
                        161, 162, 6, 241, 162, 161}; // wrong A1
                       
void processMessage(volatile Buffer *buffer, bool *msgRecieved, uint8_t msgArray[], size_t *messageSize);


Buffer buffer1 = BUFFER_INIT;
                       
//   0xA1,0xA2,0x06,0xF1,0xA2,0xA1, // good
//   0xA1,0xFF,0x06,0xF2,0xA2,0xA2, // wrong A2
//   0xA1,0xA2,0x05,0xF3,0xA2,0xA1, // wrong count                    
int main(){
    int i = 0;
    bool msgRcvd = false;
    uint8_t msgArray[11];
    size_t msgSize = 0;
    
    for (i = 0; i<13;i++)
    {
        if(i%3==0 && i>=3){
            writeBuffer(U1RXREG[i-3],&buffer1);
            writeBuffer(U1RXREG[i-2],&buffer1);
            writeBuffer(U1RXREG[i-1],&buffer1);
        }
        processMessage(&buffer1,&msgRcvd,msgArray,&msgSize);
    }
    
    return 0;
}          
          


void writeBuffer(uint8_t byte, volatile Buffer *buffer) {
    // check if you are going to over-write data not read yet.
    // once this is true, it stays always true
    
    buffer->isFull |= ((buffer->tail == buffer->head) && !buffer->isEmpty);
    if(BLOCK_WHEN_FULL &&  buffer->isFull){
        // doesn't add anymore
        return;
    }else{

        // and now the buffer is no longer empty
        buffer->isEmpty = false;

        // write data in head
        buffer->data[buffer->head] = byte; 
        // update head
        buffer->head = (buffer->head+1) % BUFFER_SIZE;
    }
}

uint8_t readBuffer(volatile Buffer *buffer) {
    if (buffer->isEmpty) {
        // doesn't read anymore
       return 0;
    } else {
        uint8_t byte = buffer->data[buffer->tail];
        buffer->tail = (buffer->tail + 1) % BUFFER_SIZE;
        // read everything there to read!
        buffer->isEmpty = (buffer->head == buffer->tail);
        return byte;
    }
}

void processMessage(volatile Buffer *buffer, bool *msgRecieved, uint8_t msgArray[], size_t *messageSize) {
    /* I check the buffer for valid messages */
    /* when I find a valid message I modify three global variables:
     1) I set message recieved flag to [msgRecieved = true].
     2) I save the message in msgArray.
     3) I save the message size in messageSize
    */
    uint8_t prevByte  = byte;
    State   prevState = state;
    while(!buffer->isEmpty){
        byte = readBuffer(buffer);
        switch (state) {
            case A1START:
                if (byte == 0xA1) {
                    state = A2START;
                }
                break;
            
            case A2START:
                if(byte == 0xA2){
                    state = SIZE;
                    errorIndex = buffer->tail; // here I found a good start flag in my buffer
                } else {
                    state = ERROR;
                }
                break;

            case SIZE:
                size = byte;
                count = 0;
                if(size > 15 || (size-4-1) <= 0){
                    state = ERROR; 
                }else{
                    state = DATA;    
                }
                
                break;

            case DATA:
                myData[count] = byte;
                count++;
                if (count == size-4-1 ) {
                    state = A2END;
                }
                break;

            case A2END:
                if (byte == 0xA2) {
                    state = A1END;
                } else {
                    state = ERROR;
                }
                break;
                
            case A1END:
                if (byte == 0xA1){
                    state = A1START;
                    errorIndex = buffer->tail; // move error pointer to end of correct message.
                    *msgRecieved = true;
                    memcpy(msgArray, myData, count);
                    *messageSize = count;
                }
                else{
                    state = ERROR;
                }
                break;
                
            case ERROR:
                uint8_t i = errorIndex; // start from the byte after the error
                uint8_t startSearch = i;
                bool startFlagFound = false;

                do{
                    if (buffer->data[i] == 0xA1) {
                        state = A2START;
                        prevState = A1START;
                        byte = buffer->data[i];
                        if ( i > 0){
                            prevByte = buffer->data[i - 1];
                        }else{
                            prevByte = byte;
                        }
                        errorIndex = (i + 1) % BUFFER_SIZE; // set the buffer index to the byte after the start flag
                        startFlagFound = true;
                        break;
                    } else {
                        i = (i + 1) % BUFFER_SIZE;
                    }
                }while (i != buffer->head)
                
                if (!startFlagFound) { // if no start flag was found in the buffer
                    state = A1START; // wait for the next start flag
                }else{ 
                    buffer->tail = errorIndex;
                }
                break;
        }
    }
}