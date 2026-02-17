# Model Predictive Control (MPC) for Aerial Manipulator

## Overview

Model Predictive Control (MPC) is an advanced control strategy that computes optimal control inputs by solving an optimization problem at each time step. Unlike traditional PID controllers that react to current errors, MPC looks ahead into the future to predict system behavior and optimize control decisions.

## How MPC Works

### 1. Prediction Model

MPC uses a mathematical model of the system dynamics to predict future behavior:

```
x(k+1) = f(x(k), u(k))
```

Where:
- `x(k)` = system state at time k (position, velocity, attitude, arm joints)
- `u(k)` = control input at time k (thrust, torques, arm velocities)
- `f()` = dynamics model (how the system evolves)

For our aerial manipulator:
- **State (15 dimensions)**: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, θ1, θ2]
- **Control (6 dimensions)**: [thrust, τx, τy, τz, u_arm1, u_arm2]

### 2. Prediction Horizon

MPC predicts N steps into the future (prediction horizon):

```
Current time: k
Predictions: k+1, k+2, k+3, ..., k+N
```

For example, with N=20 and dt=0.05s, we predict 1 second ahead.

### 3. Cost Function

MPC minimizes a cost function that balances multiple objectives:

```
J = Σ(k=0 to N-1) [ ||x(k) - x_ref(k)||²_Q + ||u(k)||²_R ] + ||x(N) - x_ref(N)||²_P
```

Components:
- **Tracking error**: `||x - x_ref||²_Q` - How far are we from desired trajectory?
- **Control effort**: `||u||²_R` - How much energy are we using?
- **Terminal cost**: `||x(N) - x_ref(N)||²_P` - Final state accuracy

Weight matrices Q, R, P determine priorities:
- Large Q → prioritize tracking accuracy
- Large R → prioritize smooth, energy-efficient control
- Large P → prioritize reaching final target

### 4. Constraints

MPC explicitly handles physical limitations:

**Input constraints:**
- Thrust limits: 0 ≤ F ≤ 35 N
- Torque limits: -2 ≤ τ ≤ 2 Nm
- Arm velocity limits: -2 ≤ u_arm ≤ 2 rad/s

**State constraints:**
- Maximum tilt angle: |roll|, |pitch| ≤ 30°
- Workspace boundaries: (x,y,z) ∈ safe region
- Joint limits: -90° ≤ θ ≤ 90°

### 5. Optimization Problem

At each time step, MPC solves:

```
minimize:    J(x, u)
subject to:  x(k+1) = f(x(k), u(k))           (dynamics)
             u_min ≤ u(k) ≤ u_max              (input constraints)
             x_min ≤ x(k) ≤ x_max              (state constraints)
```

This is a constrained optimization problem solved using numerical methods (IPOPT, SQP, etc.).

### 6. Receding Horizon Principle

The key insight of MPC:

1. **Solve** the optimization for the full horizon N
2. **Apply** only the first control input u(0)
3. **Measure** the actual next state
4. **Shift** the horizon forward by one step
5. **Repeat** at the next time step

This creates a feedback loop that continuously re-optimizes based on actual measurements.

```
Time k:     [Solve N-step problem] → Apply u(0)
Time k+1:   [Solve N-step problem] → Apply u(0)
Time k+2:   [Solve N-step problem] → Apply u(0)
...
```

## MPC for Aerial Manipulator

### System Dynamics

Our aerial manipulator has coupled dynamics:

**Quadrotor base dynamics:**
```python
# Translational motion
m * a = R * [0, 0, F] - m*g + F_arm_reaction

# Rotational motion  
I * α = τ + τ_arm_reaction
```

**Arm dynamics:**
```python
# Arm affects the base through:
# 1. Changing center of mass
# 2. Reaction forces from arm motion
# 3. Added inertia terms
```

### Why MPC is Better Than PID for This System

1. **Handles coupling**: MPC accounts for how arm motion affects the base
2. **Predicts future**: Anticipates disturbances from planned arm movements
3. **Respects constraints**: Ensures safe operation within physical limits
4. **Optimizes globally**: Balances multiple objectives (tracking, smoothness, efficiency)
5. **Reference preview**: Uses knowledge of future desired trajectory

### Control Loop Flow

```
┌─────────────────────────────────────────────┐
│  1. Measure Current State                   │
│     - IMU: acceleration, angular velocity   │
│     - Encoders: arm joint angles            │
│     - State Estimator: fuse to get full x   │
└─────────────┬───────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────┐
│  2. Get Reference Trajectory                │
│     - Desired path: circle, figure-8, etc.  │
│     - Future N steps: x_ref[k:k+N]          │
└─────────────┬───────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────┐
│  3. MPC Optimization                        │
│     - Predict: x[k+1:k+N] using dynamics    │
│     - Minimize: tracking error + effort     │
│     - Subject to: constraints               │
│     - Output: u*[k:k+N-1]                   │
└─────────────┬───────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────┐
│  4. Apply First Control Input               │
│     - Thrust: u*[0] to motors               │
│     - Arm: u*[0] to joint controllers       │
└─────────────┬───────────────────────────────┘
              │
              ▼
         Wait dt = 0.05s
              │
              └──────> (Repeat)
```

## Tuning MPC Parameters

### Prediction Horizon (N)

- **Larger N**: Better long-term planning, but slower computation
- **Smaller N**: Faster, but more myopic (short-sighted)
- **Typical**: N = 10-30 for 50ms control period

### Cost Weights (Q, R)

**Position weights (Q_pos):**
- High (10-100): Tight trajectory tracking
- Low (0.1-1): Loose tracking, smoother motion

**Control weights (R):**
- High (1-10): Conservative, smooth control
- Low (0.01-0.1): Aggressive, responsive control

**Trade-off**: Q/R ratio determines aggressiveness
- Q >> R: Aggressive tracking, high control effort
- Q << R: Smooth control, slower tracking

### Sampling Time (dt)

- Must match control loop frequency
- Shorter dt: Better accuracy, more computation
- Typical: dt = 0.02-0.1s (10-50 Hz)

## Computational Considerations

### Solve Time

For real-time operation, solve time must be less than dt:

```
Solve time < dt = 0.05s
```

Factors affecting solve time:
- Horizon length N
- State dimension (15 states)
- Control dimension (6 inputs)
- Constraint complexity
- Solver algorithm

### Warm-Starting

Reuse previous solution to speed up convergence:

```python
# At time k, we solved for u*[k:k+N-1]
# At time k+1, initialize with:
u_init[k+1:k+N-1] = u*[k+1:k+N-1]  # Shift previous solution
u_init[k+N] = u*[k+N-1]            # Extend with last input
```

This significantly reduces iterations needed.

## Implementation Architecture

```
am_description/mpc/
├── dynamics.py          # System dynamics model f(x,u)
├── mpc_solver.py        # Optimization problem setup and solver
├── state_estimator.py   # Fuse IMU + encoders → state estimate
├── utils.py             # Quaternion math, coordinate transforms
└── README.md            # This file
```

## Comparison: MPC vs PID

| Aspect | PID | MPC |
|--------|-----|-----|
| **Control law** | Reactive (current error) | Predictive (future trajectory) |
| **Constraints** | Cannot handle | Explicitly enforced |
| **Coupling** | Ignores | Explicitly models |
| **Optimality** | Local, instantaneous | Global, over horizon |
| **Computation** | Negligible (~1μs) | Moderate (~10ms) |
| **Tuning** | 3 parameters per axis | Multiple weights + horizon |
| **Disturbance rejection** | Feedback only | Feedforward + feedback |

## References

- **MPC Theory**: Rawlings, J. B., & Mayne, D. Q. (2009). *Model Predictive Control: Theory and Design*
- **Aerial Robotics MPC**: Bangura, M., & Mahony, R. (2014). *Nonlinear Dynamic Modeling for High Performance Control of a Quadrotor*
- **CasADi Framework**: Andersson, J. A. E., et al. (2019). *CasADi: A software framework for nonlinear optimization*

## Next Steps

1. Implement dynamics model in `dynamics.py`
2. Set up optimization problem in `mpc_solver.py`
3. Create state estimator in `state_estimator.py`
4. Build ROS 2 node in `scripts/mpc_controller.py`
5. Tune parameters in `config/mpc_params.yaml`
6. Test and compare with PID controller
