#ifndef AK_MOTOR_H
#define AK_MOTOR_H

#include <stdint.h>

/* ===================== CAN CONTROL MODES ===================== */
typedef enum {
    AK_CAN_DUTY        = 0,
    AK_CAN_CURRENT     = 1,
    AK_CAN_CURRENT_BRAKE = 2,
    AK_CAN_SPEED       = 3,
    AK_CAN_POSITION    = 4,
    AK_CAN_SET_ORIGIN  = 5,
    AK_CAN_POS_SPEED   = 6
} ak_control_mode_t;

/* ===================== FEEDBACK STRUCT ===================== */

typedef struct {
    float position_deg;
    float speed_erpm;
    float current_a;
} ak_feedback_t;

/* ===================== MOTOR HANDLE ===================== */
typedef struct {
    uint8_t can_id;        // Motor CAN ID (1–255)
    ak_feedback_t feedback;
} ak_motor_t;

/* ===================== USER API ===================== */
void ak_init(ak_motor_t *motor, uint8_t can_id);

void ak_process_feedback(ak_motor_t *motor, uint32_t can_id, uint8_t *data);

void ak_set_duty(ak_motor_t *motor, float duty);          // -1.0 to 1.0
void ak_set_current(ak_motor_t *motor, float current);    // Amps
void ak_set_speed(ak_motor_t *motor, float erpm);         // Electrical RPM
void ak_set_position(ak_motor_t *motor, float degrees);   // Degrees
void ak_set_pos_speed(ak_motor_t *motor,
                      float degrees,
                      int16_t erpm,
                      int16_t accel_erpm_s);

void ak_set_origin(ak_motor_t *motor, uint8_t permanent);

/* ===================== LOW LEVEL ===================== */
void ak_can_send(uint32_t ext_id, uint8_t *data, uint8_t len);

#endif
