# Aerial Manipulator (am_description)

A ROS 2 package for simulating an aerial manipulator system in Gazebo.

## Logs




## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 (sudo apt install ros-humble-gazebo-ros-pkgs)
- gazebo_ros, gazebo_ros2_control
- ros2_control, ros2_controllers


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
- Load the robot URDF with ros2_control
- Spawn the robot at position (0, 0, 1)
- Load and activate joint_state_broadcaster and arm_controller


### Using the Drone Controller Script

An interactive Python script is provided:

```bash
ros2 run am_description drone_controller.py
```

### Using the Trajectory Controller

An autonomous trajectory following controller:

```bash
ros2 run am_description trajectory_controller.py
```

Commands:
- `start <mode>` - Start trajectory following (modes: hover, circle, square, figure8)
- `stop` - Stop the controller
- `status` - Show current status
- `quit` - Exit

The trajectory controller uses PID control to make the drone:
- Take off and hover at 2m height
- Follow circular, square, or figure-8 trajectories
- Maintain stable flight with position and attitude control


### Control the Drone (Thrust)

The drone uses `gazebo_ros_force` plugin to apply thrust. Publish to `/aerial_manipulator/thrust`:

```bash
# Takeoff (apply ~25N upward force, hover is ~20.6N for 2.1kg mass)
ros2 topic pub /aerial_manipulator/thrust geometry_msgs/msg/Wrench "{force: {z: 25.0}}" -r 50

# Hover
ros2 topic pub /aerial_manipulator/thrust geometry_msgs/msg/Wrench "{force: {z: 20.6}}" -r 50

# Land (reduce thrust gradually)
ros2 topic pub /aerial_manipulator/thrust geometry_msgs/msg/Wrench "{force: {z: 15.0}}" -r 50

# Stop
ros2 topic pub /aerial_manipulator/thrust geometry_msgs/msg/Wrench "{force: {z: 0.0}}" --once
```

### Control the Arm Joints

The arm uses `ForwardCommandController` for position control. Send commands to both joints:

```bash
# Move arm to position [joint1, joint2] in radians (range: -1.57 to 1.57)
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, 0.3]}" --once

# Return to center position
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}" --once

# Extend arm
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5]}" --once
```

Commands:
- `takeoff` - Apply 30% extra thrust to lift off
- `hover` - Apply hover thrust (~20.6N)
- `land` - Reduce thrust for landing
- `stop` - Turn off motors
- `up` / `down` - Adjust thrust ±2N
- `thrust <N>` - Set specific thrust value
- `arm <j1> <j2>` - Set arm joint positions in radians
- `quit` - Exit

### View Joint States

```bash
ros2 topic echo /joint_states
```

### Check Controllers

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Package Structure

```
am_description/
├── config/
│   └── controllers.yaml          # Controller configurations
├── launch/
│   └── am_launch.launch.py       # Main launch file
├── scripts/
│   └── drone_controller.py       # Interactive drone controller
├── urdf/
│   └── am_min.urdf               # Robot description with ros2_control
├── meshes/                        # (Optional) 3D mesh files
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Controllers

The package uses two controllers:

1. **joint_state_broadcaster**: Publishes joint states to `/joint_states` topic
2. **arm_controller**: ForwardCommandController for both arm joints (position interface)

Joint limits: ±1.57 radians (±90°)

## URDF Details

### Links
- **base_link**: Quadrotor body, 0.4×0.4×0.1m box (gray), mass 1.5kg
- **rotor_1/2/3/4**: Four rotors at corners (visual only)
- **arm_base_link**: Arm mount, 0.05m cylinder (blue)
- **arm_link_1**: First arm segment, 0.1m cylinder (green)
- **arm_link_2**: Second arm segment, 0.1m cylinder (red)

### Joints
- **rotor_X_joint**: Fixed joints for rotor visuals
- **arm_mount**: Fixed joint connecting arm to quadrotor
- **arm_joint_1**: Revolute joint (Y-axis rotation, ±90°)
- **arm_joint_2**: Revolute joint (Z-axis rotation, ±90°)

### Gazebo Plugins
- **gazebo_ros_force**: Applies thrust force to base_link via `/aerial_manipulator/thrust` topic
- **gazebo_ros2_control**: Provides ros2_control hardware interface for arm joints

## Physical Parameters

| Component | Mass (kg) |
|-----------|-----------|
| base_link | 1.5 |
| arm_base_link | 0.2 |
| arm_link_1 | 0.1 |
| arm_link_2 | 0.1 |
| rotor (×4) | 0.05 each |
| **Total** | **2.1** |

Hover thrust required: 2.1 × 9.81 ≈ **20.6 N**

