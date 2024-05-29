#ifndef MSGS_H
#define MSGS_H

#include "buffers.h"
#include <stdbool.h>


// message structure => [firstStartFlag,secondStartFlag,Size,{DATA[]},firstEndFlag,secondEndFlag]
void processMsg(volatile Buffer *raw_buffer);


// message structure => [firstStartFlag,secondStartFlag,Size[2 bytes],{DATA[]},checksum]
bool processMsgBluetooth(volatile Buffer *raw_buffer, uint8_t secondStartFlag);
uint8_t calcCS_buffer(volatile Buffer* buffer, uint8_t size);
uint8_t calcCS_array(uint8_t* arr, uint8_t n);

#endif // MSGS_H
