#!/bin/bash

# ================== RADAR ==================
# kvaser_usb 드라이버 활성화
modprobe -r mttcan
modprobe -r kvaser_usb
modprobe kvaser_usb

# can 포트 열기
ip link set can0 down
ip link set can0 type can bitrate 500000
ip link set can0 up


# ================== RTK ==================
# 포트 설정
stty -F /dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTGI4BH8-if00-port0 \
  115200 cs8 -cstopb -parenb -ixon -ixoff

