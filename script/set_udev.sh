echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="4083", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyImu"' >/etc/udev/rules.d/imu.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyRobot"' >/etc/udev/rules.d/robot_exa.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyG6"' >/etc/udev/rules.d/ydlidar_g6.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyUrg"' >/etc/udev/rules.d/urg.rules

service udev reload
sleep 2
service udev restart
