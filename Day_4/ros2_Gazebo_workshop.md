# 🤖 ROS 2 Workshop: Gazebo + TurtleBot3 Simulation

Welcome to today’s **ROS 2 + Gazebo** session! 🎉  
This class is designed to be **hands-on and interactive**.  
You’ll simulate a robot, send commands, inspect its sensors, and learn how everything connects.  

---

## 📌 Prerequisites

- Ubuntu 22.04 (Desktop)  
- ROS 2 Humble installed  
- Basic Linux terminal skills  
- Internet connection for downloading packages  
- Curiosity and patience 😎  

---

## 📝 What is Gazebo?

**Gazebo** is a powerful open-source 3D robotics simulator. It allows you to:

- Test **robot models** in realistic physics environments  
- Simulate **sensors** (LiDAR, cameras, IMU, etc.)  
- Develop and debug **navigation, SLAM, and perception** before deploying to hardware  
- Reduce risks and costs by validating designs virtually  

👉 *In short: Gazebo lets you run your robot virtually so you can catch problems early.*

---

## ⚙️ Installation & Setup  

These steps are adapted from the [ROBOTIS TurtleBot3 Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).  

### 1. Install Gazebo and Dependencies

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### 2. Create a Workspace & Clone TurtleBot3 Packages

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

### 3. Configure Environment Variables

```bash
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 4. Install Simulation Package

```bash
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws
colcon build --symlink-install
```

---

## 🚀 Running the Simulation

### 1. Launch TurtleBot3 in Gazebo

```bash
cd ~/turtlebot3_ws
source ~/.bashrc
```

There are **three simulation environments** available (Pick One):  

- **Empty World**
  ```bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo empty_world.launch.py
  ```

- **TurtleBot3 World**
  ```bash
  export TURTLEBOT3_MODEL=waffle
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```

- **TurtleBot3 House**
  ```bash
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
  ```

---

### 2. Control the Robot (Teleop)

Open a **new terminal**

```bash
cd ~/turtlebot3_ws
source ~/.bashrc
```

then:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use your keyboard to drive the robot in simulation.

---

### 3. Visualize with RViz

Open another terminal and run:

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

Here you can watch **sensor data, laser scans, and transforms** as you drive.

---

### 4. Inspect Topics, Nodes, and Services

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan
ros2 service call /reset_simulation std_srvs/srv/Empty
```

---

### 5. Spawn a Custom Robot (via URDF)

You can also load your own URDF model into Gazebo:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -file ~/ros2_ws/src/my_robot/urdf/my_robot.urdf \
  -entity my_robot
```

Your robot will appear inside the Gazebo world.

---

## 🎯 Wrap-Up

In this workshop you will:  

- Install and configure **TurtleBot3 + Gazebo**  
- Launch the robot in different simulation environments  
- Drive it around using **teleoperation**  
- Visualize data in **RViz**  
- Inspect topics, nodes, and services  
- Spawn your own robots with **URDF**  

---

✨ You’re now ready to explore robotics virtually with **ROS 2 + Gazebo + TurtleBot3**!
