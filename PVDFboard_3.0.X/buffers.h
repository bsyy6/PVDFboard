/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   buffer.h
 * Author: Waleed Alghilan -  March 2024
 * Comments:
 * Revision history: 
 *  v 1.0 : initial commit.
 */

#ifndef BUFFERS_H
#define	BUFFERS_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h> // standard int library
#include <stdbool.h> // to get bool types


// use this to change the buffer size.
#define BUFFER_SIZE 25
#define BLOCK_WHEN_FULL true     // set to true if you want it to stop writing
                                 // on overflow.
                                 // set to false if you want it to overwrite the
                                 // oldest data in buffer on overflow



#define BUFFER_INIT {{0},0,0,1,0} // INITIAL VALUES ALWAYS USE THIS TO START

/**
 * <p><b>Summary:
 *  A buffer that is circular.
 *  basically starts from 0 and fills it with data until BUFFER_SIZE
 
 *  use add2Buffer ( uint8_t byte , CircularBuffer *buf)
 *          1) adds data to the buffer on location buffer->head
 *          2) increments the head by 1.
 *  
 *  use readBuffer (CircularBuffer *buf) to read data from buffer.
 *          1) returns data saved in location buffer->tail.
 *          2) increments the tail by 1. 
 *          -  reading an empty buffer returns 0.
 * 
 *  use clearBuffer(CircularBuffer *buf) to clear the buffer.
 *          clearing the buffer resets all values to 0.
 * 

 *  warning : in case of overflow it overwrites the oldest data in buffer. [default]
 *  warning : you can make it block the writing and wait until you read all data.
 *  instead by defining block_when_full. 
 * 
 * 
 *  example use:
 *  include "buffers.h"
 * 
 *  Buffer myBuffer = BUFFER_INIT; // make sure to start the buffer like this.
 *  
 * int main(){
 *  unsigned char x = 2;
 *  unsigned char y = 0; 
 * 
 *  // add value of x to buffer
 *  addToBuffer(x,&myBuffer);  // myBuffer = [ 2 , 0, 0 .. to BUFFER_SIZE] 
 *  x++;
 *  addToBuffer(x,&myBuffer);  // myBuffer = [ 2 , 3, 0 .. to BUFFER_SIZE]
 *  x++;
 *  addToBuffer(x,&myBuffer);  // myBuffer = [ 2 , 3, 4 .. to BUFFER_SIZE]
 *  x++;
 *  
 * // read the buffer into y
 *  y = readFromBuffer(&myBuffer); // y = 2
 *  y = readFromBuffer(&myBuffer); // y = 3
 *  y = readFromBuffer(&myBuffer); // y = 4 // buffer is empty
 *  y = readFromBuffer(&myBuffer); // y = 0 // buffer is empty
 * 
 * // if at any point the buffer became full and can't save anymore
 * myBuffer.isFull is set to true
 *  
 *  
 * 
 * 
 * }
 * 
 * </b></p>
  */ 



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

#endif	/* XC_HEADER_TEMPLATE_H */

