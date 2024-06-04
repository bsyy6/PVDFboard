#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "timeout.h"
#include "setup.h"
#include "msgs.h"


typedef enum {
    BT_POWER_OFF,       // 0
    BT_POWER_ON,        // 1
    BT_CONNECTED,       // 2
    BT_NOT_CONNECTED,   // 3
    BT_OK,              // 4
    BT_NOT_OK,          // 5
    BT_TIMEOUT,         // 6
    BT_STATUS_NOT_OK    // 7  A message response is received but says it failed to execute the command
} bt_states;


bt_states init_BT();
bt_states connect_BT(uint8_t BTMAC[6]);
bt_states whoAmI(uint8_t *BTMAC);
bt_states setJustWorks(void);
bt_states disconnect_BT(void);
bt_states rename_BT(uint8_t* name, uint8_t nameSize);
bt_states setConnectionTiming_BT(const uint8_t val);
void msgUpdate_BT(void);
void msgInit_BT(void);
bt_states CMD_REQ(const uint8_t *msg, uint8_t msgSize, uint16_t waitTime);
bt_states CMD_RSP_IND(const uint8_t flag,  uint16_t waitTime);
void communicateError_BT(bt_states bs,const uint8_t *msg, uint8_t msgSize);

#endif	/* BLUETOOTH_H */

