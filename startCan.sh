#! /bin/bash

echo "setting up can"

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 down
echo "loopback on"
ip link set can0 type can bitrate 1000000 loopback on
ip link set can0 up
