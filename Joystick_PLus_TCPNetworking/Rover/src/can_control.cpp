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
//Constructor:
//Takes in the name of the can interface, eg "can0"
// following is the Initializer list, sets the parameters "interface name" and "sock_fd" of the CanDriver class to the given values
// sock_fd is init to -1. Once the socket is opened, it go to 0 for Valid socket. 
CanDriver::CanDriver(const std::string& interface)
    : iface_name(interface), sock_fd(-1)
{
    // Open CAN RAW socket
    // From sys/socket.h
    // the :: forces global namespace, forcing the pull from socket.h
    sock_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    //Check the socket is open
    if (sock_fd < 0)
        throw std::runtime_error("Failed to create CAN socket");

    // ifreq -> interface request. {} initializes to zero
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ - 1);//copies the interface name into the struct
    
    if (::ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {//checks that the interface got opened
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
//Send an extended Can Frame
//Extended Can frame is the same message length, but has a larger Idenrtifier. this allows for more messages to be sent on the Bus.
//Takes in three arguments: The Reciever ID, a poiter to the data, and the message length (in Bytes)
int CanDriver::sendExt(uint32_t ext_id, const uint8_t* data, uint8_t len)
{
    //check that the length is no more than 8 bytes
    if (len > 8){
        //len = 8; 
        return -1; //error if length is too long
    }

    //instance a new can frame
    struct can_frame frame{};
    
    //set the Can id and the can length of the new frame
    frame.can_id = ext_id | CAN_EFF_FLAG; // Extended ID flag
    frame.can_dlc = len;

    //copy the message data into the can frame data field
    std::memcpy(frame.data, data, len);

    //send it to the SocketCAN Stack
    ssize_t nbytes = ::write(sock_fd, &frame, sizeof(frame));
    //check how many bytes were sent
    if (nbytes != sizeof(frame))
        return -1;

    //if everything went well, the message sent
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


//Send Standard 11-Bit can frame
//takes in:
//std_id: the id for the message. 
//data is a pointer to the data
//length is an 8 bit in for the message length
int CanDriver::sendStd(uint16_t std_id, const uint8_t* data, uint8_t len)
{
    //if the message is longer than 8 bytes, cut it down. 
    //should probably also split the message into multuple messages. 
    if (len > 8){ 
        //len = 8;
        return -1; //error if length is too long
    }

    //instance a can frame
    struct can_frame frame{};
    //set the Can ID and the can frame message length
    frame.can_id = std_id & CAN_SFF_MASK; // Standard ID
    frame.can_dlc = len;
    //copy the data into the Can frame data field
    std::memcpy(frame.data, data, len);

    //send it to the SocketCAN Stack
    ssize_t nbytes = ::write(sock_fd, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)){
        //check how many bytes were sent    
        return -1;
    }

    //if everything went well, the message sent
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
