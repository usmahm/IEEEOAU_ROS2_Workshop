# 🤖 ROS2 Workshop: Gazebo + TurtleBot3 Simulation

Welcome to today’s **ROS2 + Gazebo** session! 🎉  
This class is intended to be **hands‑on and interactive**. You’ll see a robot in simulation, send commands, inspect its sensors, and learn how everything ties together.  

---

## 📌 Prerequisites

- Ubuntu 22.04 (Desktop)  
- ROS 2 Humble installed  
- Basic Linux terminal skills  
- Internet connection for downloading packages  
- Curiosity and patience 😎  

---

## 📝 What is Gazebo?

**Gazebo** is a powerful open-source 3D robotics simulator. It lets you:

- Test **robot models** in realistic physics  
- Simulate **sensors** (LiDAR, cameras, IMU, etc.)  
- Develop and debug **navigation, SLAM, perception** before deploying to hardware  
- Reduce risks and costs by verifying on virtual robots first  

In short: *Gazebo allows you to run your robot virtually so you catch problems early.*

---

## ⚙️ Install & Set Up  

These steps are adapted from the ROBOTIS TurtleBot3 Quick Start guide. ([emanual.robotis.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/))  

### 1.Install Gazebo and Dependencies

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

#### b) Create a workspace and clone TurtleBot3 packages

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
sudo apt install python3-colcon-common-extensions
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

#### c) Configure environment variables

```bash
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


Install Simulation Package
---
```bash
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install
```


---

### 2. Launch TurtleBot3 in Gazebo

```bash
cd ~/turtlebot3_ws
source ~/.bashrc
```

With your environment ready:
Empty World
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```



TurtleBot3 World

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

You’ll see the TurtleBot3 in a Gazebo world.


TurtleBot3 House
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

---

### 3. Control the Robot (Teleop)

Open a new terminal (don’t forget `source ~/.bashrc`), then:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use your keyboard to drive the robot in the simulation.

---

### 4. Visualize with RViz

In another terminal:

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

Watch sensor data, laser scans, and transforms as you drive.

---

### 5. Inspect Topics / Nodes / Services

Use standard ROS 2 commands:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan
ros2 service call /reset_simulation std_srvs/srv/Empty
```

---

### 6. Spawn a Custom Robot (via URDF)

You can also spawn your own URDF model:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -file ~/ros2_ws/src/my_robot/urdf/my_robot.urdf \
  -entity my_robot
```

Then it appears inside the Gazebo world.

---

## 🎯 Wrap-Up

In this session you will:

- Install and set up **TurtleBot3 + Gazebo**  
- Launch the robot in simulation  
- Drive it around using teleoperation  
- Visualize sensor data in **RViz**  
- Inspect topics, nodes, and services  
- Spawn custom robots using URDF  
