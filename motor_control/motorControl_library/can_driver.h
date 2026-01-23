#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "ak_motor.h"

int  can_init(const char *ifname);
void can_receive_loop(ak_motor_t *motors, int motor_count);

#endif
