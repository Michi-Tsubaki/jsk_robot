# g1eus
![g1eus](./figs/g1eus.png)
## Setup

Please install ROS 1 and some development tools like rosdep, vcstools at first.

Then build the packages.

```shell
source /opt/ros/<ROS DISTRO>/setup.bash
mkdir -p <catkin workspace>/src
cd <catkin workspace>/src
wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/refs/heads/master/jsk_g1_robot/g1eus/ros-o.repos.yaml -O- | vcs import
sudo apt update && rosdep update && rosdep install -iqry --from-paths .
cd ..
catkin build g1eus
```
