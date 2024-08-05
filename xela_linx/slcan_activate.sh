#!/bin/bash

if [ "$(id -u)" -ne 0 ]
  then echo "Please run as root"
  exit
fi
# 使用用户提供的参数设置USB设备
USB_DEVICE="$1"

#sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB"$1"
sudo ifconfig slcan0 up
