# Butler Robot

A ROS 2 based autonomous robot for restaurant service automation. The Butler Robot is designed to autonomously navigate restaurant environments, pick up orders from the kitchen, and deliver them to specific tables.

## Features

- Autonomous navigation in restaurant environments
- Dynamic pose tracking and localization
- Pre-mapped location memory (kitchen, tables, home position)
- Order management and delivery system
- Gazebo simulation support
- Integration with Nav2 for path planning

## Prerequisites

- ROS 2 Humble
- Gazebo
- Python 3.8+
- Nav2 Stack

### Required ROS 2 Packages

```bash
sudo apt install ros-humble-gazebo-ros2-control \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-xacro \
                 ros-humble-nav2-bringup \
                 ros-humble-tf-transformations
```

### Python Dependencies

```bash
pip3 install transforms3d
```

## Installation

1. Create a workspace and clone the repository:
```bash
mkdir goat_robotics_ws/
cd goat_robotics_ws/
git clone git@github.com:virensompura/butler_robot.git
```

2. Build the workspace:
```bash
cd ..
colcon build
source install/setup.bash
```

## Usage

### 1. Launch Simulation Environment

Start the Gazebo simulation with the Butler Robot:
```bash
ros2 launch butler_robot launch_sim.launch.py
```

### 2. Initialize Localization

Launch AMCL (Adaptive Monte Carlo Localization) with your map:
```bash
ros2 launch nav2_bringup localization.launch.py map:=/path/to/your/map.yaml
```

### 3. Start Navigation Stack

Initialize the navigation system:
```bash
ros2 launch butler_robot navigation.launch.py
```

### 4. Record Robot Positions

Run the pose recording node to save important locations (home, kitchen, tables):
```bash
ros2 run butler_robot get_pose.py
```
This will create a `location.txt` file containing all the saved positions.

### 5. Start Server

Launch the navigation server (uses Nav2's Basic Navigator):
```bash
ros2 run butler_robot server.py
```

### 6. Start Order Management

Run the order management system:
```bash
ros2 run butler_robot user_order.py
```

## Operation Flow

1. The robot starts at its home position
2. When an order is ready, the system notifies the robot
3. Robot navigates to the kitchen to collect the order
4. Using saved positions, the robot delivers the order to the specified table
5. After delivery, the robot returns to its home position

## File Structure

```
butler_robot/
├── config/                  # Configuration files
├── description/            # Robot URDF and Xacro files
├── launch/                # Launch files
├── scripts/               # Python scripts
│   ├── get_pose.py       # Position recording
│   ├── server.py         # Navigation server
│   └── user_order.py     # Order management
├── worlds/                # Gazebo world files
└── CMakeLists.txt        # Build configuration
```

