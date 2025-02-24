#!/bin/bash

if [ "$(id -u)" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "Installing VScom USB-CAN Plus auto-start service..."
#cp ./99-VScom_USB-CAN_Plus.rules /etc/udev/rules.d/
cp ./90-slcan.rules /etc/udev/rules.d/
cp ./slcan_add.sh /usr/local/bin/
cp ./slcan@.service /etc/systemd/system/
chmod +x /usr/local/bin/slcan_add.sh
udevadm control --reload-rules
systemctl daemon-reload
echo "Done."
echo "============================================"
echo "If having issues, check the following files:"
echo "/etc/udev/rules.d/90-slcan.rules"
echo "/usr/local/bin/slcan_add.sh"
echo "/etc/systemd/system/slcan@.service"
