#include "ak_motor.h"
#include "can_driver.h"
#include <pthread.h>

ak_motor_t motors[1];

void *rx_thread(void *arg) {
    can_receive_loop(motors, 1);
    return NULL;
}

int main(void)
{
    can_init("can0");

    ak_init(&motors[0], 1);

    pthread_t rx;
    pthread_create(&rx, NULL, rx_thread, NULL);

    while (1) {
        ak_set_current(&motors[0], 2.0f);

        printf("Pos: %.2f deg | Speed: %.1f erpm | I: %.2f A\n",
               motors[0].feedback.position_deg,
               motors[0].feedback.speed_erpm,
               motors[0].feedback.current_a);

        usleep(10000);
    }
}
