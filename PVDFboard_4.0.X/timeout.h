#ifndef TIMEOUT_H
#define TIMEOUT_H
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>


typedef struct{
    uint16_t *currentTimePtr;
    bool *currentWrapStatusPtr;
    uint16_t ttimeOut;
    bool wrapCycle;
} timeOutObj;

timeOutObj timeOutBegin(uint16_t* currentTimePtr,uint16_t delay, bool* wrapped);
bool timeOutCheck(timeOutObj *toPtr);


#endif

