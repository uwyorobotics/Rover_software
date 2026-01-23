#include "ak_motor.h"

/* ===================== INTERNAL HELPERS ===================== */

static inline void append_i32(uint8_t *buf, int32_t val, uint8_t *idx) {
    buf[(*idx)++] = (val >> 24) & 0xFF;
    buf[(*idx)++] = (val >> 16) & 0xFF;
    buf[(*idx)++] = (val >> 8)  & 0xFF;
    buf[(*idx)++] =  val        & 0xFF;
}

static inline void append_i16(uint8_t *buf, int16_t val, uint8_t *idx) {
    buf[(*idx)++] = (val >> 8) & 0xFF;
    buf[(*idx)++] =  val       & 0xFF;
}

static inline uint32_t make_ext_id(uint8_t can_id, ak_control_mode_t mode) {
    return ((uint32_t)mode << 8) | can_id;
}

static inline int32_t read_i32(uint8_t *buf) {
    return ((int32_t)buf[0] << 24) |
           ((int32_t)buf[1] << 16) |
           ((int32_t)buf[2] << 8)  |
           ((int32_t)buf[3]);
}

static inline int16_t read_i16(uint8_t *buf) {
    return ((int16_t)buf[0] << 8) | buf[1];
}

/* ===================== FEEDBACK DECODER ===================== */

void ak_process_feedback(ak_motor_t *motor,
                         uint32_t can_id,
                         uint8_t *data)
{
    uint8_t rx_motor_id = can_id & 0xFF;

    if (rx_motor_id != motor->can_id) {
        return;
    }

    int32_t pos_raw = read_i32(&data[0]);
    int16_t spd_raw = read_i16(&data[4]);
    int16_t cur_raw = read_i16(&data[6]);

    motor->feedback.position_deg = pos_raw / 10000.0f;
    motor->feedback.speed_erpm   = spd_raw * 10.0f;
    motor->feedback.current_a    = cur_raw / 100.0f;
}

/* ===================== PUBLIC API ===================== */

void ak_init(ak_motor_t *motor, uint8_t can_id) {
    motor->can_id = can_id;
}

void ak_set_duty(ak_motor_t *motor, float duty) {
    uint8_t buf[4];
    uint8_t idx = 0;
    append_i32(buf, (int32_t)(duty * 100000.0f), &idx);
    ak_can_send(make_ext_id(motor->can_id, AK_CAN_DUTY), buf, idx);
}

void ak_set_current(ak_motor_t *motor, float current) {
    uint8_t buf[4];
    uint8_t idx = 0;
    append_i32(buf, (int32_t)(current * 1000.0f), &idx);
    ak_can_send(make_ext_id(motor->can_id, AK_CAN_CURRENT), buf, idx);
}

void ak_set_speed(ak_motor_t *motor, float erpm) {
    uint8_t buf[4];
    uint8_t idx = 0;
    append_i32(buf, (int32_t)erpm, &idx);
    ak_can_send(make_ext_id(motor->can_id, AK_CAN_SPEED), buf, idx);
}

void ak_set_position(ak_motor_t *motor, float degrees) {
    uint8_t buf[4];
    uint8_t idx = 0;
    append_i32(buf, (int32_t)(degrees * 10000.0f), &idx);
    ak_can_send(make_ext_id(motor->can_id, AK_CAN_POSITION), buf, idx);
}

void ak_set_pos_speed(ak_motor_t *motor,
                      float degrees,
                      int16_t erpm,
                      int16_t accel_erpm_s) {

    uint8_t buf[8];
    uint8_t idx = 0;

    append_i32(buf, (int32_t)(degrees * 10000.0f), &idx);
    append_i16(buf, erpm / 10, &idx);
    append_i16(buf, accel_erpm_s / 10, &idx);

    ak_can_send(make_ext_id(motor->can_id, AK_CAN_POS_SPEED), buf, idx);
}

void ak_set_origin(ak_motor_t *motor, uint8_t permanent) {
    uint8_t buf = permanent ? 1 : 0;
    ak_can_send(make_ext_id(motor->can_id, AK_CAN_SET_ORIGIN), &buf, 1);
}
