# MPC End-Effector Controller for Aerial Manipulator

Real-time Model Predictive Control (MPC) for aerial manipulator **end-effector trajectory tracking** using the [acados](https://docs.acados.org/) solver.

## Overview

This package implements a whole-body MPC controller that actively controls both the quadrotor base and the 2-DOF arm to track desired end-effector trajectories in world frame. Unlike `mpc_acados` which keeps the arm fixed, this controller:

- **Actively controls arm joints** to achieve end-effector tracking
- Uses **inverse kinematics (IK)** to convert EE trajectories to joint references
- Tracks both base position and EE position simultaneously
- Maintains the same 17-state, 6-control formulation for compatibility

## End-Effector Kinematics

### Arm Configuration (from URDF)

```
                 base_link
                     │
                     │ arm_mount (fixed)
                     │ offset: (0, 0, -0.05)
                     ▼
               arm_base_link
                     │
         ┌──────────┐│ arm_joint_1 (revolute, Y-axis)
         │ Link 1   ││ L1 = 0.2m
         │          │▼
         └──────────┘
               arm_link_1
                     │
         ┌──────────┐│ arm_joint_2 (revolute, Y-axis)
         │ Link 2   ││ offset: -π rotation
         │          ││ L2 = 0.2m
         └──────────┘▼
               arm_link_2
                     │
                     ▼
              end_effector
```

### Forward Kinematics

End-effector position in **body frame**:

$$x_{ee} = L_1 \sin(q_1) - L_2 \sin(q_1 + q_2)$$
$$y_{ee} = 0$$
$$z_{ee} = z_{\text{mount}} - L_1 \cos(q_1) + L_2 \cos(q_1 + q_2)$$

where:
- $L_1 = L_2 = 0.2$ m (arm link lengths)
- $z_{\text{mount}} = -0.05$ m (arm mount offset below base)
- $q_1, q_2$ = joint angles (rad)

**Important configurations:**
| Configuration | $q_1$ | $q_2$ | Description |
|---------------|-------|-------|-------------|
| Folded | 0 | 0 | Arm folded up (due to -π offset) |
| Extended down | 0 | π | Arm straight down |
| Forward reach | π/4 | π/2 | Arm reaching forward |

End-effector in **world frame**:
$$\mathbf{p}_{ee}^{world} = \mathbf{p}_{base} + R(\mathbf{q}) \cdot \mathbf{p}_{ee}^{body}$$

### Jacobian

The arm Jacobian relates joint velocities to end-effector velocity:

$$\dot{\mathbf{p}}_{ee} = J(\mathbf{q}_{arm}) \dot{\mathbf{q}}_{arm}$$

$$J = \begin{bmatrix} 
L_1 c_1 - L_2 c_{12} & -L_2 c_{12} \\
0 & 0 \\
L_1 s_1 - L_2 s_{12} & -L_2 s_{12}
\end{bmatrix}$$

where $c_1 = \cos(q_1)$, $s_1 = \sin(q_1)$, $c_{12} = \cos(q_1+q_2)$, $s_{12} = \sin(q_1+q_2)$.

### Inverse Kinematics

Given desired EE position in body frame $(x_d, z_d)$, solve for joint angles:

1. **Compute distance to target:**
   $$r = \sqrt{x_d^2 + z_d^2}$$

2. **Check reachability:**
   - Maximum reach: $r_{max} = L_1 + L_2 = 0.4$ m
   - Minimum reach: $r_{min} = |L_1 - L_2| = 0$ m

3. **Solve for $q_2$ (law of cosines):**
   $$\cos(q_2) = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$
   $$q_2 = \pm \arccos(\cos(q_2)) (elbow up/down)$$

4. **Solve for $q_1$:**
   $$\phi = \arctan2(x_d, -z_d)$$
   $$\alpha = \arccos\left(\frac{L_1^2 + r^2 - L_2^2}{2 L_1 r}\right)$$
   $$q_1 = \phi \mp \alpha (elbow up/down)$$

## State and Control Representation

### State Vector (17 states)

| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 0-2 | `x, y, z` | Base position in world frame | m |
| 3-5 | `vx, vy, vz` | Base linear velocity | m/s |
| 6-9 | `qx, qy, qz, qw` | Base orientation quaternion | - |
| 10-12 | `ωx, ωy, ωz` | Base angular velocity (body) | rad/s |
| 13-14 | `q1, q2` | Arm joint positions | rad |
| 15-16 | `dq1, dq2` | Arm joint velocities | rad/s |

### Control Vector (6 controls)

| Index | Symbol | Description | Units | Bounds |
|-------|--------|-------------|-------|--------|
| 0 | `T` | Total thrust force | N | [0, 40] |
| 1-3 | `τx, τy, τz` | Body torques | Nm | [-2, 2] |
| 4-5 | `ddq1, ddq2` | Arm joint accelerations | rad/s² | [-10, 10] |

## Dynamics Model

### Base Dynamics (6-DOF Rigid Body)

Same as `mpc_acados`:

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = \frac{1}{m} R(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} - \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$$

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}$$

$$\dot{\boldsymbol{\omega}} = J^{-1} (\boldsymbol{\tau} - \boldsymbol{\omega} \times J\boldsymbol{\omega})$$

### Arm Dynamics (Double Integrator)

$$\dot{q}_i = \dot{q}_i, \quad \ddot{q}_i = u_{\text{arm},i}$$

**Note:** Coupling effects (reaction torques, Coriolis terms) are ignored for simplicity.

## MPC Formulation

### Optimal Control Problem

$$\min_{x_{0:N}, u_{0:N-1}} \sum_{k=0}^{N-1} \ell(x_k, u_k, x_k^{\text{ref}}) + V_f(x_N, x_N^{\text{ref}})$$

### Cost Function

The key difference from `mpc_acados` is **higher arm position weights** to prioritize EE tracking:

| Component | Weight | Description |
|-----------|--------|-------------|
| Q_pos | **5.0** | Base position (lower than mpc_acados) |
| Q_vel | 1.0 | Base velocity |
| Q_att | 5.0 | Base attitude |
| Q_omega | 0.5 | Base angular velocity |
| Q_arm_pos | **20.0** | Arm joint position (HIGH for EE tracking) |
| Q_arm_vel | 1.0 | Arm joint velocity |
| R_thrust | 0.01 | Thrust regularization |
| R_torque | 0.1 | Torque regularization |
| R_arm_acc | **0.01** | Arm acceleration (LOW for fast motion) |

### Reference Trajectory Generation

1. Define desired **EE trajectory in world frame**: $\mathbf{p}_{ee}^{ref}(t)$
2. Define desired **base position**: $\mathbf{p}_{base}^{ref}(t)$
3. Compute **EE position in body frame**: $\mathbf{p}_{ee}^{body} = R^T (\mathbf{p}_{ee}^{ref} - \mathbf{p}_{base}^{ref})$
4. Solve **inverse kinematics** for joint angles: $(q_1^{ref}, q_2^{ref}) = IK(\mathbf{p}_{ee}^{body})$
5. Construct full state reference: $x^{ref} = [\mathbf{p}_{base}^{ref}, \mathbf{0}, \mathbf{q}_{att}, \mathbf{0}, q_1^{ref}, q_2^{ref}, 0, 0]$

### Solver Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Horizon (N) | 20 | Prediction steps |
| Time step (dt) | 0.05 s | Discretization |
| Total horizon | 1.0 s | N × dt |
| Solver | SQP_RTI | Real-time iteration |
| Integrator | ERK (RK4) | 4th order Runge-Kutta |
| QP solver | PARTIAL_CONDENSING_HPIPM | Efficient QP |

## Control Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  End-Effector MPC Controller                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────┐    ┌────────────┐    ┌────────┐    ┌──────────┐  │
│  │  State   │───▶│ EE Target  │───▶│  IK    │───▶│ Reference│  │
│  │ Estimator│    │ Trajectory │    │ Solver │    │ x_ref    │  │
│  └──────────┘    └────────────┘    └────────┘    └────┬─────┘  │
│        ▲                                              │        │
│        │                                              ▼        │
│   ┌────┴────┐                                  ┌───────────┐   │
│   │   IMU   │                                  │  Acados   │   │
│   │  Joints │                                  │   MPC     │   │
│   └─────────┘                                  └─────┬─────┘   │
│                                                      │         │
│                                                      ▼         │
│                                               ┌───────────┐    │
│                                               │  Control  │    │
│                                               │  Output   │    │
│                                               └─────┬─────┘    │
│                                                     │          │
└─────────────────────────────────────────────────────┼──────────┘
                                                      ▼
                                    ┌─────────────────────────────┐
                                    │  Base: Wrench (T, τ)        │
                                    │  Arm:  Position commands    │
                                    └─────────────────────────────┘
```

## EE Trajectory Modes

| Mode | Description | Motion |
|------|-------------|--------|
| `hover` | Keep EE stationary | EE at default position below base |
| `circle` | Circular motion | EE traces circle in XY plane |
| `line` | Linear oscillation | EE moves back/forth along X |
| `vertical` | Vertical oscillation | EE moves up/down |
| `reach` | Forward reach | Smooth reach motion forward |

### Default EE Position

When arm is extended down ($q_1=0, q_2=\pi$):

$$z_{ee} = z_{base} + z_{mount} - L_1 - L_2 = z_{base} - 0.45 \text{ m}$$

## File Structure

```
mpc_ef/
├── __init__.py           # Package exports
├── acados_model.py       # Dynamics + forward kinematics
├── acados_solver.py      # MPC solver + inverse kinematics
└── README.md             # This file

scripts/
└── mpc_controller_ef.py  # ROS2 controller node
```

## Usage

### Running the Controller

```bash
# Terminal 1: Start simulation
ros2 launch am_description am_launch.launch.py

# Terminal 2: Run EE tracking controller
ros2 run am_description mpc_controller_ef.py
```

### Interactive Commands

| Command | Description |
|---------|-------------|
| `start [mode]` | Start EE trajectory (hover/circle/line/vertical/reach) |
| `stop` | Stop controller |
| `plot` | Plot logged data (base + EE positions) |
| `stats` | Show solver statistics |
| `status` | Show current EE position and joints |
| `quit` | Exit |

### Example Session

```
>>> start circle
Started EE trajectory: circle
>>> status
Control enabled: True
Mode: circle
Base position: [0.00, 0.00, 2.00]
EE position: [0.15, 0.00, 1.55]
Arm joints: [0.12, 3.02]
>>> stats
EE Tracking Error (m):
  Mean: 0.0123
  Max:  0.0456
  RMS:  0.0189
```

## Comparison with mpc_acados

| Feature | mpc_acados | mpc_ef |
|---------|-----------|--------|
| Arm control | Zeroed post-solve | **Active** |
| Q_arm_pos | 1.0 | **20.0** |
| R_arm_acc | 0.1 | **0.01** |
| Reference | Base trajectory | **EE world trajectory + IK** |
| Use case | Base stabilization | **End-effector manipulation** |
| Output | Zero arm velocity | **Arm position commands** |

## Extending to Coupled Dynamics

For more accurate whole-body control, extend the dynamics model:

1. **Coupled inertia matrix:**
   $$M(\mathbf{q}_{arm}) \ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + g(\mathbf{q}) = \boldsymbol{\tau}$$

2. **Reaction torques on base:**
   $$\boldsymbol{\tau}_{reaction} = -J_{arm}^T \mathbf{f}_{arm}$$

3. **Full Coriolis/centrifugal terms**

4. **End-effector wrench in cost function:**
   Add EE position directly to cost (requires constraint formulation)

## Performance

Typical solve times:
- Mean: ~2-4 ms
- Max: ~8-12 ms
- Real-time feasible at 20 Hz (50 ms period)

The slightly higher solve times compared to `mpc_acados` are due to:
- More active arm motion (less trivial solution)
- IK computation overhead (negligible)

## Dependencies

- [acados](https://docs.acados.org/) with Python interface
- CasADi
- NumPy
- ROS2 (rclpy, sensor_msgs, geometry_msgs)

## References

1. Siciliano et al., "Robotics: Modelling, Planning and Control", Springer, 2009 (Kinematics)
2. Verschueren et al., "acados: a modular open-source framework for fast embedded optimal control", MPC, 2021
3. Lippiello et al., "Cartesian Impedance Control of a UAV with a Robotic Arm", IROS, 2012
