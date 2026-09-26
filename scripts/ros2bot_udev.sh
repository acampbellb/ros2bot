#!/usr/bin/env bash
# Set up ros2bot udev rules.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
	echo "Run this script with sudo: sudo $0" >&2
	exit 1
fi

printf '%s\n' 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout", SYMLINK+="rplidar"' >/etc/udev/rules.d/99-rplidar.rules
printf '%s\n' 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{devpath}=="2.3.3", MODE:="0777", SYMLINK+="r2bserial"' >/etc/udev/rules.d/99-serial.rules
printf '%s\n' 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{devpath}=="2.3.2", MODE:="0777", SYMLINK+="r2bspeach"' >/etc/udev/rules.d/99-speach.rules
printf '%s\n' 'SUBSYSTEM=="tty" MODE="0777"' >/etc/udev/rules.d/99-usb-serial.rules

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm settle

if [[ -L /dev/r2bserial ]]; then
	echo "/dev/r2bserial -> $(readlink -f /dev/r2bserial)"
else
	echo "Rule installed, but /dev/r2bserial was not created. Connect the 1a86:7523 device on USB port 2.3.3, or update the rule to match its actual USB path." >&2
fi


