#CP2102 串口号0002 设置别名为forklift_controller
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="forklift_controller"' >/etc/udev/rules.d/forklift_controller.rules

#CP2102 设置别名为forklift_lidar
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="d40788f1e413ec11a099f0ef7a109228", MODE:="0777", GROUP:="dialout", SYMLINK+="forklift_lidar"' >/etc/udev/rules.d/forklift_lidar.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="forklift_N10_PLUS"' >/etc/udev/rules.d/forklift_lidar.rules

#未知芯片 设置别名为forklift_imu
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="forklift_imu"' >/etc/udev/rules.d/forklift_imu.rules

# 相机别名  买的单目相机 和 深度相机
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="USB 2.0 Camera: HD USB Camera",ATTR{index}=="0",MODE:="0777",SYMLINK+="forklift_camera"' >>/etc/udev/rules.d/forklift_camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="forklift_realsense"' >>/etc/udev/rules.d/forklift_camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="USB 2.0 Camera",ATTR{index}=="0",MODE:="0777",SYMLINK+="forklift_camera_jetson"' >>/etc/udev/rules.d/forklift_camera.rules

service udev reload
sleep 2
service udev restart


