# jsk_nextage_robot

## Installation
- Following pages provide informative resources.
- Official page: https://nextage.kawadarobot.co.jp/open
- Nextage Open Tutorials: https://rtmros-nextage.readthedocs.io/en/latest/


### Installation for User PC
1. If you want to use TouchUSBs (haptic devices), install OpenHaptics and Touch Device Driver from here: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US
   * If your Ubuntu version is not supported, check JSK backup: https://drive.google.com/drive/folders/1FiQ4m3XtoDlwdRIq3H7LJv8T882SBVBl

2. Install ROS packages:
   ```bash
   mkdir -p ~/nextage_ws/src
   cd ~/nextage_ws/src
   wstool init
   wstool merge https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_nextage_robot/jsk_panda_user.rosinstall
   wstool update
   cd ../
   source /opt/ros/<Your ROS Distribution>/setup.bash
   rosdep install -y -r --from-paths src --ignore-src --skip-keys=librealsense2,realsense2_camera
   catkin build jsk_nextage_bringup
   source devel/setup.bash
   ```
3. If you want to use TouchUSBs (haptic devices), see [this README](https://github.com/pazeshun/Geomagic_Touch_ROS_Drivers/tree/dual-phantom-readme#use-multiple-devices) to setup devices.
   * Note that this README was written for old Ubuntu.
     - On Ubuntu 18.04, run `Touch_Setup` instead of `/opt/geomagic_touch_device_driver/Geomagic_Touch_Setup`. If you want to check device status, run `Touch_Diagnostic` instead of `/opt/geomagic_touch_device_driver/Geomagic_Touch_Diagnostic`.

#### Trouble Shooting
1. `Failed to initialize haptic device`  -> Please give access to haptic devices, i.e `sudo chmod 777 /dev/ttyACM[0-1]`

