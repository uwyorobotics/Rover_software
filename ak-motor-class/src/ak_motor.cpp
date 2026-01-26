#include "ak_motor.h"
#include "can_control.h"

//All of the above is assuming that the motor is in Servo Mode

/*
typedef enum {
    CAN_PACKET_SET_DUTY = 0, // Duty Cycle Mode
    CAN_PACKET_SET_CURRENT, // Current Loop Mode
    CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
    CAN_PACKET_SET_RPM, // Speed Mode
    CAN_PACKET_SET_POS, // Position Mode
    CAN_PACKET_SET_ORIGIN_HERE, // Set Origin Mode
    CAN_PACKET_SET_POS_SPD, // Position-Speed Loop Mode
} CAN_PACKET_ID;

inline void buffer_append_int32(uint8_t* buffer, int32_t value, int32_t* index) {
    buffer[(*index)++] = (value >> 24) & 0xFF;
    buffer[(*index)++] = (value >> 16) & 0xFF;
    buffer[(*index)++] = (value >> 8)  & 0xFF;
    buffer[(*index)++] = value & 0xFF;
}
*/

//The speed value is of type int32, and the range is -100000-100000, representing -100000-100000 electrical RPM

//Example for Speed Loop Mode Transmission
/*void comm_can_set_rpm(uint8_t controller_id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

//Example for Position Loop Mode Transmission:
void comm_can_set_pos(uint8_t controller_id, float pos) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

*/

int ak_motor_check(CanDriver& driver, int can_id) {
    //check that the can id is found on the canbus?
    return 1;

}

void ak_motor_stop(CanDriver& driver, int can_id) { //set the speed of the motor to zero
    // Code to stop the motor
    int speed = 0;
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)speed, &send_index);
    driver.sendExt(can_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void ak_motor_set_speed(CanDriver& driver, int can_id, int speed) { 
    //set the speed of the motor to be speed:
    //What units is the speed given in? EPRM
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)speed, &send_index);
    driver.sendExt(can_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);

}

int ak_motor_get_speed(CanDriver& driver, int can_id) {

    return 0; 
}