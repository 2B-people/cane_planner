# echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="rplidar_laser"' >/etc/udev/rules.d/rplidar_laser.rules
# echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/fdilink_ahrs.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",MODE:="0777", GROUP:="dialout", SYMLINK+="feedback_controller"' >/etc/udev/rules.d/feedback_controller.rules

service udev reload
sleep 2
service udev restart


