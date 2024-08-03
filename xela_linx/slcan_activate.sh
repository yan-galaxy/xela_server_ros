#!/bin/bash

if [ "$(id -u)" -ne 0 ]
  then echo "Please run as root"
  exit
fi

sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ifconfig slcan0 up
