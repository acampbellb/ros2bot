# setup ros2bot udev rules 

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout", SYMLINK+="rplidar"' >/etc/udev/rules.d/99-rplidar.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{devpath}=="2.3.3", MODE:="0777", SYMLINK+="r2bserial"' >/etc/udev/rules.d/99-serial.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{devpath}=="2.3.2", MODE:="0777", SYMLINK+="r2bspeach"' >/etc/udev/rules.d/99-speach.rules
echo  'SUBSYSTEM=="tty" MODE="0777"' >/etc/udev/rules.d/99-usb-serial.rules

service udev reload
sleep 2
service udev restart


