#pragma once
#include <cstdint>
#include "can_control.h"

// ===================== CAN Packet IDs =====================
enum CAN_PACKET_ID {
    CAN_PACKET_SET_DUTY = 0,          // Duty Cycle Mode
    CAN_PACKET_SET_CURRENT,            // Current Loop Mode
    CAN_PACKET_SET_CURRENT_BRAKE,     // Current Brake Mode
    CAN_PACKET_SET_RPM,                // Speed Mode
    CAN_PACKET_SET_POS,                // Position Mode
    CAN_PACKET_SET_ORIGIN_HERE,        // Set Origin Mode
    CAN_PACKET_SET_POS_SPD,            // Position-Speed Loop Mode
};

// ===================== Helper Functions =====================
/**
 * Append a 32-bit integer to a buffer in big-endian order
 */
inline void buffer_append_int32(uint8_t* buffer, int32_t value, int32_t* index) {
    buffer[(*index)++] = (value >> 24);
    buffer[(*index)++] = (value >> 16);
    buffer[(*index)++] = (value >> 8);
    buffer[(*index)++] = value;
}

inline void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}


// ===================== AK Motor API =====================

/**
 * Check if motor is present on the CAN bus
 * @param driver  Reference to CanDriver
 * @param can_id  Motor CAN ID
 * @return 1 if present, 0 if not (currently stub)
 */
int ak_motor_check(CanDriver& driver, int can_id);

/**
 * Stop the motor (sets speed to zero)
 * @param driver  Reference to CanDriver
 * @param can_id  Motor CAN ID
 */
void ak_motor_stop(CanDriver& driver, int can_id);

/**
 * Set motor speed (in electrical RPM)
 * @param driver  Reference to CanDriver
 * @param can_id  Motor CAN ID
 * @param speed   Desired speed in ERPM (32 bits)
 */
void ak_motor_set_speed(CanDriver& driver, int can_id, int speed);

/**
 * Get motor speed (in electrical RPM)
 * @param driver  Reference to CanDriver
 * @param can_id  Motor CAN ID
 * @return Current speed in ERPM (stub: always returns 0)
 */
int ak_motor_get_speed(CanDriver& driver, int can_id);

