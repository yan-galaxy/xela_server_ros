#!/bin/bash

if [ "$(id -u)" -ne 0 ]
  then echo "Please run as root"
  exit
fi

echo "Removing VScom USB-CAN Plus auto-start service..."
#rm /etc/udev/rules.d/99-VScom_USB-CAN_Plus.rules
rm /etc/udev/rules.d/90-slcan.rules
rm /usr/local/bin/slcan_add.sh
rm /etc/systemd/system/slcan@.service
udevadm control --reload-rules
systemctl daemon-reload
echo "Done."
echo "============================================"
echo "If having issues, check the following files:"
echo "/etc/udev/rules.d/90-slcan.rules"
echo "/usr/local/bin/slcan_add.sh"
echo "/etc/systemd/system/slcan@.service"
