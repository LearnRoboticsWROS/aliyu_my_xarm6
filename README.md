# 🦾 FAIRINO + MoveIt 2 Integration (ROS 2 Humble)

This guide explains how to simulate xarm6 robot using the official repository of xarm_ros2 repository provided by U_FACTORY. This package takes the key file of the official repository to custom needs, meaning setting up the position of the robot
with the respect of the world, setup a camera by creating a new urdf.xacro file and create a custom world. The camera is simulated, so from the official Orbbec package we use only the .stl. the data of the topic /camera/image_raw and /camera/points
are simulated using the Gazebo plugin

---

## ⚙️ Prerequisites

- Ubuntu 22.04  
- ROS 2 Humble (`ros-humble-desktop`)  
- xarm_ros2 `build inside a workspace ~/aliyu_ws 
- OrbbecSDK_ROS2 package build inside a workspace ~/aliyu_ws
- these 2 package need to share the same src folder of the aliyu_ws
- clone the link attacher package, we will use in pick and place application

```bash
cd ~/aliyu_ws/src
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git 
```
clone in ~/aliyu_ws/src this repository


## Build and run

- each terminal that you about to open, make sure to source the environment
```bash
cd ~/aliyu_ws
colcon build
source install/setup.bash
```

- Terminal 1:
```bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```


