# xArm Galactic Docker + Unity
This repository contains the Docker setup for ROS 2 Galactic and basic Unity integration with the xArm robot. All commands and notes are together for quick reference.

The Dockerfile will work using Galactic driver for the uf850 xarm model. Currently the dockerfile has been tested and works.

# Docker
build and run the Dockerfile
```bash
# Navigate to Docker folder
cd docker

# Build and start Docker
docker compose build
docker compose up -d

# Open a shell in the running container
docker exec -it <container_name> bash
```

# Gazebo Simulation

Enable display forwarding for Gazebo (Gazebo is recomended for simulation)
```bash

echo $DISPLAY
export DISPLAY=host.docker.internal:0

# Navigate to ROS2 workspace
cd ~/dev_ws

# Source ROS and workspace
source /opt/ros/galactic/setup.bash
source install/setup.bash

# Build workspace
colcon build
source install/setup.bash
```

# xArm ROS2 Launch Commands
```bash

ros2 launch xarm_api uf850_driver.launch.py robot_ip:=<ROBOT_IP>
#Starts the xArm API nodes that communicate with the physical robot.

ros2 launch xarm_moveit_config dual_uf850_moveit_fake.launch.py add_gripper:=true
#Loads a simulated version of the xArm in MoveIt!, use to plan motions, visualize in RViz, and test algorithms safely

ros2 launch xarm_controller uf850_control_rviz_display.launch.py robot_ip:=<ROBOT_IP> add_gripper:=true
#Starts the ROS 2 controller nodes for the xArm. Opens RViz to visualize the robot state in real-time.

ros2 launch xarm_moveit_servo uf850_moveit_servo_realmove.launch.py robot_ip:=<ROBOT_IP> start_servo:=true
#Allows sending joint velocity or pose commands in real-time.


ros2 run xarm_moveit_servo xarm_keyboard_input
#Opens a terminal-based interface to move the robot using keys.
```

# Unity ROS TCP Connection


Start ROS TCP server inside Docker

```bash

ros2 run ros_tcp_endpoint default_server_endpoint
#listens to Unity TCP endpoint
```

