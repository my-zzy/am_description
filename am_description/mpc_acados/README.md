# MPC Acados Controller for Aerial Manipulator

Real-time Model Predictive Control (MPC) for aerial manipulator whole-body control using the [acados](https://docs.acados.org/) solver.

## Overview

This package implements a nonlinear MPC controller for an aerial manipulator (quadrotor with 2-DOF arm). The controller uses acados with Sequential Quadratic Programming Real-Time Iteration (SQP_RTI) for real-time feasibility.

**Current Configuration**: The arm joints are kept fixed by zeroing arm control inputs after the MPC solve. The full state/control vectors are maintained for future extension to active arm control.

## State and Control Representation

### State Vector (17 states)

| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 0-2 | `x, y, z` | Position in world frame | m |
| 3-5 | `vx, vy, vz` | Linear velocity in world frame | m/s |
| 6-9 | `qx, qy, qz, qw` | Orientation quaternion (body to world) | - |
| 10-12 | `ωx, ωy, ωz` | Angular velocity in body frame | rad/s |
| 13-14 | `q1, q2` | Arm joint positions | rad |
| 15-16 | `dq1, dq2` | Arm joint velocities | rad/s |

### Control Vector (6 controls)

| Index | Symbol | Description | Units | Bounds |
|-------|--------|-------------|-------|--------|
| 0 | `T` | Total thrust force | N | [0, 50] |
| 1-3 | `τx, τy, τz` | Body torques | Nm | [-5, 5] |
| 4-5 | `ddq1, ddq2` | Arm joint accelerations | rad/s² | [-10, 10] |

## Dynamics Model

### Base Dynamics (6-DOF Rigid Body)

The quadrotor base follows standard rigid body dynamics:

**Position dynamics:**
$$\dot{p} = v$$

**Velocity dynamics (Newton's equation):**
$$\dot{v} = \frac{1}{m} R(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} - \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}$$

where:
- $m$ = 2.5 kg (total mass)
- $R(\mathbf{q})$ = rotation matrix from body to world frame
- $T$ = thrust force along body z-axis
- $g$ = 9.81 m/s² (gravity)

**Quaternion dynamics:**
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}$$

where the quaternion product encodes the angular velocity.

**Angular velocity dynamics (Euler's equation):**
$$\dot{\boldsymbol{\omega}} = J^{-1} \left( \boldsymbol{\tau} - \boldsymbol{\omega} \times J\boldsymbol{\omega} \right)$$

where:
- $J$ = diag(0.05, 0.05, 0.1) kg·m² (inertia tensor)
- $\boldsymbol{\tau}$ = [τx, τy, τz] (body torques)

### Arm Dynamics (Double Integrator)

For simplicity, arm dynamics are modeled as decoupled double integrators:

$$\dot{q}_i = \dot{q}_i$$
$$\ddot{q}_i = u_{\text{arm},i}$$

**This ignores coupling effects between arm and base.** For accurate whole-body dynamics, this should be extended to include:
- Coriolis/centrifugal effects
- Reaction torques on base from arm motion
- Coupled inertia matrix

## MPC Formulation

### Optimal Control Problem

The MPC solves the following OCP at each control step:

$$\min_{x_{0:N}, u_{0:N-1}} \sum_{k=0}^{N-1} \ell(x_k, u_k, x_k^{\text{ref}}) + V_f(x_N, x_N^{\text{ref}})$$

subject to:
- $x_{k+1} = f(x_k, u_k)$ (discrete dynamics)
- $x_0 = x_{\text{current}}$ (initial state)
- $x_{\min} \leq x_k \leq x_{\max}$ (state bounds)
- $u_{\min} \leq u_k \leq u_{\max}$ (control bounds)

### Cost Function

**Stage cost (quadratic):**
$$\ell(x, u, x^{\text{ref}}) = \|x - x^{\text{ref}}\|_Q^2 + \|u - u^{\text{hover}}\|_R^2$$

**Cost weight matrices:**

| Component | Weight | Description |
|-----------|--------|-------------|
| Q_pos | 10.0 | Position tracking |
| Q_vel | 1.0 | Velocity tracking |
| Q_att | 5.0 | Attitude tracking |
| Q_omega | 0.5 | Angular velocity |
| Q_arm_pos | 1.0 | Arm position tracking |
| Q_arm_vel | 0.1 | Arm velocity tracking |
| R_thrust | 0.01 | Thrust regularization |
| R_torque | 0.1 | Torque regularization |
| R_arm_acc | 0.1 | Arm acceleration regularization |

**Terminal cost:** $V_f = 2 \times \ell$ (terminal weight factor)

### Solver Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Horizon (N) | 20 | Prediction steps |
| Time step (dt) | 0.05 s | Discretization |
| Total horizon | 1.0 s | N × dt |
| Solver | SQP_RTI | Real-time iteration |
| Integrator | ERK (RK4) | 4th order Runge-Kutta |
| QP solver | PARTIAL_CONDENSING_HPIPM | Efficient QP |
| RTI iterations | 1 | Single SQP iteration |

## Control Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    MPC Controller                       │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌───────────┐ │
│  │    State     │───▶│  Reference   │───▶│  Acados   │ │
│  │  Estimator   │    │  Generator   │    │  Solver   │ │
│  └──────────────┘    └──────────────┘    └───────────┘ │
│         ▲                                      │       │
│         │                                      ▼       │
│    ┌────┴────┐                          ┌───────────┐  │
│    │   IMU   │                          │  Control  │  │
│    │  Joints │                          │  Output   │  │
│    └─────────┘                          └───────────┘  │
│                                               │        │
└───────────────────────────────────────────────┼────────┘
                                                ▼
                              ┌─────────────────────────────┐
                              │  Base: Wrench (T, τ)        │
                              │  Arm:  Velocity (0, 0)      │
                              └─────────────────────────────┘
```

## File Structure

```
mpc_acados/
├── __init__.py           # Package exports and constants
├── acados_model.py       # Dynamics model and CasADi expressions
├── acados_solver.py      # Acados OCP solver wrapper
└── README.md             # This file

scripts/
└── mpc_controller_acados.py  # ROS2 controller node
```

## Usage

### Running the Controller

```bash
# Terminal 1: Start simulation
ros2 launch am_description am_launch.launch.py

# Terminal 2: Run MPC controller
ros2 run am_description mpc_controller_acados.py
```

### Interactive Commands

| Command | Description |
|---------|-------------|
| `start [mode]` | Start trajectory tracking |
| `stop` | Stop controller |
| `plot` | Plot logged data |
| `stats` | Show solver statistics |
| `status` | Show current state |
| `quit` | Exit |

**Trajectory modes:** `hover`, `takeoff`, `up`, `circle`, `square`, `figure8`

## Performance

Typical solve times on modern CPU:
- Mean: ~1-3 ms
- Max: ~5-10 ms
- Real-time feasible at 20 Hz control rate (50 ms period)

## Extending to Active Arm Control

To enable active arm control:

1. **Remove zero_arm_controls flag:**
   ```python
   u_opt, x_pred, info = self.mpc_solver.solve(x_current, x_ref, zero_arm_controls=False)
   ```

2. **Set arm reference trajectory:**
   ```python
   x_ref[k, 13:15] = desired_arm_positions
   x_ref[k, 15:17] = desired_arm_velocities
   ```

3. **Extract arm controls:**
   ```python
   ddq1, ddq2 = u_opt[0, 4], u_opt[0, 5]
   ```

4. **For accurate dynamics**, update `acados_model.py` to include:
   - Coupled inertia matrix
   - Reaction torques from arm motion
   - Full Coriolis/centrifugal terms

## Dependencies

- [acados](https://docs.acados.org/) with Python interface
- CasADi
- NumPy
- ROS2 (rclpy, sensor_msgs, geometry_msgs)

See [INSTALL_ACADOS.md](INSTALL_ACADOS.md) for installation instructions.

## References

1. Verschueren et al., "acados: a modular open-source framework for fast embedded optimal control", Mathematical Programming Computation, 2021
2. Diehl et al., "A Real-Time Iteration Scheme for Nonlinear Optimization in Optimal Feedback Control", SIAM Journal on Control and Optimization, 2005
