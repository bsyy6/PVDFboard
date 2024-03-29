#include "buffers.h"

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