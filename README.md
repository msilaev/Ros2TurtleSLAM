# TurtleSLAM Setup and Navigation Instructions

## Setting up TurtleBot3 Simulation

Follow the detailed setup instructions from this [Medium article](https://medium.com/@nilutpolkashyap/setting-up-turtlebot3-simulation-in-ros-2-humble-hawksbill-70a6fcdaf5de).

## Navigation Instructions

Refer to this [ROS2 Navigation Tutorial](https://roboticsbackend.com/ros2-nav2-tutorial/) for detailed navigation instructions.

## Setup Packages

```sh
cd Documents/Robotics/TurtleSLAM/ros2_ws
colcon build --symlink-install
```

## SLAM in turtle3_world setting

### Start Gazebo TurtleBot Simulation

```sh
pkill -9 ros2
pkill -9 gzclient
pkill -9 gzserver
pkill -9 robot_state_pub
pkill -9 lidarNode
cd Documents/Robotics/Ros2TurtleSLAM/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Cartographer

```sh
cd Documents/Robotics/Ros2TurtleSLAM/ros2_ws
cd Documents/Robotics/SLAM/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Motion Control Loop Controlled by LIDAR Sensor

```sh
cd Documents/Robotics/Ros2TurtleSLAM/ros2_ws
source install/setup.bash
pkill -9 lidarNode
ros2 run lidar_avoidance lidarNode
```

### Save Map

```sh
cd Documents/Robotics/Ros2TurtleSLAM/ros2_ws
ros2 run nav2_map_server map_saver_cli -f ./map
```

### If Robot is Stuck, Use Teleop

```sh
cd Documents/Robotics/SLAM/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

### Navigation in the Saved Map

```sh
```sh
cd Documents/Robotics/SLAM/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```
```
