#include "ak_motor.h"

#include "ak_motor.h"
#include "can_driver.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <errno.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <net/if.h>

extern int can_socket;

/* ===================== CAN SOCKET ===================== */

static int can_socket = -1;

/* ===================== INITIALIZATION ===================== */

int can_init(const char *ifname)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* Create CAN RAW socket */
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        perror("socket");
        return -1;
    }

    /* Get interface index */
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(can_socket);
        return -1;
    }

    /* Bind socket */
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(can_socket);
        return -1;
    }

    /* Disable CAN filters (receive everything if needed later) */
    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return 0;
}

/* ===================== SEND FUNCTION (AK LIBRARY HOOK) ===================== */

void ak_can_send(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    if (can_socket < 0) {
        fprintf(stderr, "CAN socket not initialized\n");
        return;
    }

    if (len > 8) {
        len = 8;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id  = ext_id | CAN_EFF_FLAG;  // Extended frame
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    ssize_t nbytes = write(can_socket, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("CAN write");
    }
}

/* ===================== RECEIVE LOOP ===================== */

void can_receive_loop(ak_motor_t *motors, int motor_count)
{
    struct can_frame frame;

    while (1) {
        ssize_t nbytes = read(can_socket, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("CAN read");
            continue;
        }

        if (!(frame.can_id & CAN_EFF_FLAG)) {
            continue;  // ignore standard frames
        }

        uint32_t ext_id = frame.can_id & CAN_EFF_MASK;

        for (int i = 0; i < motor_count; i++) {
            ak_process_feedback(&motors[i], ext_id, frame.data);
        }
    }
}
