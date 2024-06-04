#include <stdbool.h>
#include <limits.h>
#include "timeout.h"


timeOutObj timeOutBegin(uint16_t* currentTimePtr,uint16_t delay, bool* wrapped){
    // returns a timoutObj
    timeOutObj to;
    
    const uint16_t currentTime_max = 60000; // change if type  of currentTime is changed
    to.currentTimePtr = currentTimePtr;
    to.ttimeOut = (*currentTimePtr + delay) % (currentTime_max) + 1;
    
    if(to.ttimeOut < *currentTimePtr){
     to.wrapCycle = !*wrapped;
    }else{
     to.wrapCycle = *wrapped;
    }
    to.currentWrapStatusPtr = wrapped;
    to.timedOut = false;
    return (to);
}

bool timeOutCheck(timeOutObj *toPtr){
    // returns true when timeout passed 
    if(!toPtr->timedOut){
        // avoids checking an already expired timer.
        toPtr->timedOut = !(*toPtr->currentTimePtr < toPtr->ttimeOut  || *toPtr->currentWrapStatusPtr != toPtr->wrapCycle);   
    }
    return (toPtr->timedOut);
}
