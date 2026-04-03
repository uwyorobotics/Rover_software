#include <iostream>
#include <cassert>
#include "CanDriver.hpp"
#include "Motor.hpp"

// A dummy CAN interface for testing (replace with real if needed)
class DummyCanDriver : public CanDriver {
public:
    DummyCanDriver() : CanDriver("can0") {}
    int sendExt(uint32_t /*ext_id*/, const uint8_t* /*data*/, uint8_t /*len*/) override {
        // Do nothing for unit tests
        return 0;
    }
    int sendStd(uint16_t /*std_id*/, const uint8_t* /*data*/, uint8_t /*len*/) override {
        return 0;
    }
};

// ===================== Test functions =====================
void test_motor_start() {
    DummyCanDriver can;
    Motor motor(can, 1);

    motor.start();
    assert(motor.running() == true);
}

void test_motor_stop() {
    DummyCanDriver can;
    Motor motor(can, 1);

    motor.start();
    motor.stop();
    assert(motor.running() == false);
}

void test_motor_set_speed() {
    DummyCanDriver can;
    Motor motor(can, 1);

    motor.setSpeed(50);
    assert(motor.getSpeed() == 50);
}

void test_motor_invalid_speed() {
    DummyCanDriver can;
    Motor motor(can, 1);

    motor.setSpeed(-10);
    // Assuming your Motor class clamps negative speed to 0
    assert(motor.getSpeed() >= 0);
}

// ===================== Main =====================
int main() {
    test_motor_start();
    test_motor_stop();
    test_motor_set_speed();
    test_motor_invalid_speed();

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
