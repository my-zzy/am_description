# Aerial Manipulator (am_description)

A ROS 2 package for simulating an aerial manipulator system in Gazebo.


## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11 (gazebo_ros)


## Installation

1. Clone this package into your ROS 2 workspace:
```bash
cd ~/am_ws/src
git clone <repository_url> am_description
```

2. Install dependencies:
```bash
cd ~/am_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/am_ws/
colcon build --packages-select am_description
source install/setup.bash
```

## Usage

### Launch the Simulation

```bash
ros2 launch am_description am_launch.launch.py
```

This will:
- Start Gazebo with the aerial manipulator model
- Load the robot URDF
- Start the controller manager
- Spawn all configured controllers

### Control the Arm Joints

Send position commands to individual joints:

```bash
# Control arm joint 1 (pitch motion)
ros2 topic pub /arm_joint_1_position_controller/command std_msgs/msg/Float64 "data: 0.5"

# Control arm joint 2 (yaw motion)
ros2 topic pub /arm_joint_2_position_controller/command std_msgs/msg/Float64 "data: -0.5"
```

### View Joint States

```bash
ros2 topic echo /joint_states
```

## Package Structure

```
am_description/
├── config/
│   └── controllers.yaml          # Controller configurations
├── launch/
│   └── am_launch.launch.py       # Main launch file
├── urdf/
│   └── am_min.urdf               # Robot description
├── meshes/                        # (Optional) 3D mesh files
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Controllers

The package uses three controllers:

1. **joint_state_broadcaster**: Publishes joint states to `/joint_states` topic
2. **arm_joint_1_position_controller**: Controls the first arm joint (Y-axis rotation)
3. **arm_joint_2_position_controller**: Controls the second arm joint (Z-axis rotation)

### Controller Parameters

Both position controllers use PID control with:
- P gain: 5.0
- I gain: 0.0
- D gain: 0.1

Joint limits: ±1.57 radians (±90°)

## URDF Details

### Links
- **base_link**: 0.4×0.4×0.1m box (gray)
- **arm_base_link**: 0.05m cylinder (blue)
- **arm_link_1**: 0.1m cylinder (green)
- **arm_link_2**: 0.1m cylinder (red)

### Joints
- **arm_mount**: Fixed joint connecting arm to quadrotor
- **arm_joint_1**: Revolute joint (Y-axis, ±90°)
- **arm_joint_2**: Revolute joint (Z-axis, ±90°)

