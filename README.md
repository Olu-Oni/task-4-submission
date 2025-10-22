# Kuka Stick Description

A ROS 2 package for visualizing a KUKA med-14 robot URDF model.

## Prerequisites 
For :
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble (or newer)

- Required ROS 2 packages:
  - `ros-humble-urdf-tutorial`
  - `ros-humble-joint-state-publisher-gui`
  - `ros-humble-robot-state-publisher`
  - `ros-humble-rviz2`

## Installation

### 1. Install ROS 2

If you haven't already installed ROS 2, follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install ros-humble-urdf-tutorial ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher ros-humble-rviz2
```

### 3. Set Up Workspace

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/Olu-Oni/task-4-submission/ kuka_stick_description

cd ~/ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select kuka_stick_description

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the URDF Visualization

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch with RViz
ros2 launch urdf_tutorial display.launch.py model:=src/kuka_stick_description/urdf/stick.urdf
```

This will open:
- **RViz2**: For 3D visualization of the robot model
- **Joint State Publisher GUI**: For interactively controlling joint positions

### View Available Meshes

The package includes core meshes in th `meshes/` directory, 
and simplified meshes for the collisions in the `meshes/simplified/` directory and DAE mesh files for the links.

## Package Structure

```
kuka_stick_description/
├── meshes/
│   ├── simplified/     # Simplified mesh models
│   ├── link_0.dae      # Link mesh files
│   ├── link_1.dae
│   └── ...
├── urdf/
│   └── stick.urdf      # Robot description file
├── CMakeLists.txt
└── package.xml
```

(last minute readme from claude 😭)
