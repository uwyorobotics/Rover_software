#include "can_control.h"
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>

// ================= Constructor / Destructor =================

CanDriver::CanDriver(const std::string& interface)
    : iface_name(interface), sock_fd(-1)
{
    // Open CAN RAW socket
    sock_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd < 0)
        throw std::runtime_error("Failed to create CAN socket");

    // Find interface index
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(sock_fd);
        throw std::runtime_error("ioctl() failed");
    }

    // Bind socket to CAN interface
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(sock_fd);
        throw std::runtime_error("bind() failed");
    }
}

CanDriver::~CanDriver()
{
    if (sock_fd >= 0)
        ::close(sock_fd);
}

// ================= CAN Transmission =================

int CanDriver::sendExt(uint32_t ext_id, const uint8_t* data, uint8_t len)
{
    if (len > 8) len = 8;

    struct can_frame frame{};
    frame.can_id = ext_id | CAN_EFF_FLAG; // Extended ID flag
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);

    ssize_t nbytes = ::write(sock_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
        return -1;

    return 0;
}
/* this function (sendExt) should have the same behavior as this function given in the documentation:
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
    uint8_t i=0;
    if (len > 8) {
        len = 8;
    }
    CanTxMsg TxMessage;
    TxMessage.StdId = 0;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.ExtId = id;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = len;
    //memcpy(txmsg.data8, data, len);
    for(i=0;i<len;i++)
    TxMessage.Data[i]=data[i];
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

*/



int CanDriver::sendStd(uint16_t std_id, const uint8_t* data, uint8_t len)
{
    if (len > 8) len = 8;

    struct can_frame frame{};
    frame.can_id = std_id & CAN_SFF_MASK; // Standard ID
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);

    ssize_t nbytes = ::write(sock_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
        return -1;

    return 0;
}

// ================= CAN Receive =================

bool CanDriver::receive(uint32_t& can_id,
                        std::vector<uint8_t>& data,
                        bool& is_extended,
                        int timeout_ms)
{
    struct pollfd pfd{};
    pfd.fd = sock_fd;
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret < 0)
        throw std::runtime_error("poll() failed");
    if (ret == 0)
        return false; // timeout

    struct can_frame frame{};
    ssize_t nbytes = ::read(sock_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
        throw std::runtime_error("read() failed");

    is_extended = frame.can_id & CAN_EFF_FLAG;
    can_id = frame.can_id & (is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    data.assign(frame.data, frame.data + frame.can_dlc);

    return true;
}
