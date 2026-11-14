# jsk\_baxter\_startup

The `jsk_baxter_startup` package.

## Installation

### Copy `env.sh` to home dir for `baxter-c1`

Copy `env.sh` to home directory for machine tag in baxter-c1

```bash
ssh baxter-c1
roscd jsk_baxter_startup/jsk_baxter_machine
cp env.sh ~/
```

### Copy udev to `/etc/udev/rules.d`

```bash
roscd jsk_baxter_startup/jsk_baxter_udev
sudo cp * /etc/udev/rules.d
```

## ROS launch and euslisp

### Default gripper

Launch and euslisp program for Baxter + default gripper

```bash
roslaunch jsk_baxter_startup baxter_default.launch
```

```bash
roscd baxtereus
roseus baxter-interface.l
;; euslisp interpreter
(baxter-init)
```

### Softhand gripper

Launch and euslisp program for Baxter + [Softhand](https://ieeexplore.ieee.org/document/8968011)

```bash
roslaunch jsk_baxter_startup baxter_softhand.launch
```

```bash
roscd baxtereus
roseus baxter-softhand-interface.l
;; euslisp interpreter
(baxter-init)
```

### Rosbag record

```bash
roslaunch jsk_baxter_startup baxter_rosbag_record.launch
```

### Rosbag play

```bash
roslaunch jsk_baxter_startup baxter_rosbag_play.launch
```

## ROS nodes

### xdisplay\_image\_topic.py

**Usage**

```bash
rosrun jsk_baxter_startup xdisplay_image_topic.py /cameras/head_camera/image
```

<img src="images/xdisplay_image_topic.jpg" width="30%" />

## ROS 1 SDK and ROS 2 bridge

This package provides shared Baxter startup, standard ROS state conversion and
the ROS 1/ROS 2 bridge. Existing ROS 1 startup remains available. Application
nodes, policy clients and their configuration belong in downstream packages.

### Build for ROS 2 Jazzy

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths \
  src/jsk-ros-pkg/jsk_robot/jsk_baxter_robot/jsk_baxter_startup
source install/setup.bash
```

### ROS 1 SDK host

Use the Baxter SDK workspace and set `ROS_MASTER_URI`/`ROS_IP` for the real robot.
Expose this source package to rospack, or install it in the SDK workspace:

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/colcon_ws/src/jsk-ros-pkg/jsk_robot/jsk_baxter_robot:$ROS_PACKAGE_PATH
roslaunch jsk_baxter_startup baxter_ros1.launch enable_robot:=false
```

This starts the SDK arm/gripper action servers and the standard-state adapter.
If the servers already run, pass `start_action_servers:=false`. The SDK gripper
server may calibrate uncalibrated electric grippers on startup. Enable the robot
through the SDK when ready to execute; no motion sanity-check is launched.
Open the required cameras separately; the SDK permits only two active cameras.

`ros1_state_adapter.py` translates Baxter `AssemblyState`, `EndpointState` and
`EndEffectorState` into standard messages:

| Topic | Type | Values |
| --- | --- | --- |
| `/baxter/robot_enabled` | `std_msgs/Bool` | SDK enabled state |
| `/baxter/{left,right}/endpoint_pose` | `geometry_msgs/PoseStamped` | Endpoint pose |
| `/baxter/{left,right}/gripper_state` | `sensor_msgs/JointState` | Electric gripper opening, 0 closed / 1 open |

Invalid or uncalibrated electric gripper state is not forwarded. The standard
messages avoid requiring a ROS 2 Baxter message package or custom bridge mappings.
The ROS 1 install includes the launch and executable; the ROS 2 install also
ships the adapter under `share/jsk_baxter_startup/ros1` for use on the SDK host.

### Bridge host

```bash
source ~/colcon_ws/install/setup.bash
export ROS_MASTER_URI=http://BAXTER_HOST:11311
export ROS_IP=THIS_PC_IP
ros2 run jsk_baxter_startup bridge.sh
```

The wrapper sources `/opt/ros/one/setup.bash` and `~/colcon_ws/install/setup.bash`
only for the bridge process. Override `BAXTER_ROS1_SETUP` and `BAXTER_ROS2_SETUP`
for another installation. Use an **action-enabled** `ros1_bridge`;
`--print-pairs` must list `FollowJointTrajectory` and `GripperCommand` actions.
Run exactly one dynamic/parameter bridge on the same graph. Start downstream
ROS 2 nodes in a separate, pure ROS 2 shell.
