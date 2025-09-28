# xArm Galactic Docker + Unity
This repository contains the Docker setup for ROS 2 Galactic and basic Unity integration with the xArm robot. All commands and notes are together for quick reference.

The Dockerfile works with the Galactic driver for the **UF850 xArm model**.

## Quickstart
1. Clone this repo (Docker setup):  
   `git clone https://github.com/Ale-Torres/xarm-galactic-dev.git`
2. Clone xArm ROS 2 drivers into your workspace:  
   `git clone --recurse-submodules -b galactic https://github.com/xArm-Developer/xarm_ros2.git`
3. Build & run Docker:  
   `docker compose up -d`


# Prerequisites
Before using this setup, make sure you have:

- Docker & Docker Compose
- ROS 2 Galactic installed (on host, optional for some steps)
- Unity (optional, for visualization and ROS TCP endpoint)
- The xArm ROS2 Galactic repository cloned (https://github.com/xArm-Developer/xarm_ros2)  


```bash
# Navigate to your workspace
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src

# Clone the xArm ROS2 Galactic branch
git clone --recurse-submodules -b galactic https://github.com/xArm-Developer/xarm_ros2.git
```


# Docker
Build and run the Docker container
```bash

#Clone this repo if you haven’t already
git clone https://github.com/Ale-Torres/xarm-galactic-dev.git

# Navigate to Docker folder
cd docker

# Build and start Docker
docker compose build
docker compose up -d

# Open a shell in the running container
docker ps  # check NAME or CONTAINER ID of your container
docker exec -it <container_name_or_id> bash
```

# Gazebo Simulation

Enable display forwarding for Gazebo (Gazebo is recommended for simulation)
```bash
# Enable display forwarding (for Gazebo and RViz)
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

To bridge Unity and ROS 2 you need:

- [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (runs inside ROS/Docker)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) (installed as a Unity package)

```bash

ros2 run ros_tcp_endpoint default_server_endpoint
#listens to Unity TCP endpoint
#In Unity, install the ROS-TCP-Connector package and configure the ROS IP/Port to match your running Docker container (default 0.0.0.0:10000)
```


