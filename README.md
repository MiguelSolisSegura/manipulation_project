# Manipulation Project

This repository contains the ROS 2 packages developed for the manipulation project using the UR3e robotic arm. The project integrates MoveIt2 for controlling the robotic arm and implements perception for a robust Pick & Place task.

## Prerequisites

Before you begin, ensure you have ROS 2 installed along with the following dependencies:

- MoveIt2
- Gazebo for ROS 2
- RViz for visualization

## Installation

Clone this repository into the `src` directory of your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/MiguelSolisSegura/manipulation_project.git
```

Compile the packages with:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

This project is divided into two main parts:

1. **Basic Pick & Place Task** (Checkpoint 13)
2. **Pick & Place with Perception** (Checkpoint 14)

### Testing Basic Pick & Place Task

To launch the basic Pick & Place task in simulation:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_moveit_config move_group.launch.py
ros2 launch my_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place.launch.py
```

### Testing Pick & Place with Perception

To launch the Pick & Place task with perception capabilities:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_moveit_config move_group.launch.py
ros2 launch my_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place_perception.launch.py
```

## Real Robot Testing

To test the project with the real UR3e robot, follow these steps:

1. Book a session with the UR3e real robot lab.
2. Connect to the real robot on the scheduled day and time.
3. Run the following commands to start the Pick & Place task with perception:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch real_moveit_config move_group.launch.py
ros2 launch real_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py
```
