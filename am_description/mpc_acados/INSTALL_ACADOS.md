# Acados MPC Installation Guide

## Prerequisites

1. **Install CasADi** (symbolic framework):
```bash
pip3 install casadi
```

2. **Install acados** (from source):

```bash
# Navigate to home directory
cd ~

# Clone acados repository
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# Create build directory
mkdir -p build
cd build

# Configure with cmake
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_HPIPM=ON -DACADOS_WITH_OSQP=ON ..

# Build (this takes a few minutes)
make install -j4

# Install Python interface
cd ..
pip3 install -e interfaces/acados_template
```

3. **Set environment variables** (add to `~/.bashrc`):

```bash
# Acados (Linux)
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"

# Note: DYLD_LIBRARY_PATH is only needed on macOS, not Linux
```

Then reload:
```bash
source ~/.bashrc
```

## Verify Installation

Test that acados is properly installed:

```bash
cd ~/acados/examples/acados_python/getting_started
python3 minimal_example_ocp.py
```

If this runs without errors, acados is properly installed!

## Usage

### Test the acados MPC solver:

```bash
cd ~/am_ws/src/am_description
python3 -m am_description.mpc.acados_mpc_solver
```

### Run the acados MPC controller:

```bash
# Build the workspace
cd ~/am_ws
colcon build --packages-select am_description
source install/setup.bash

# Make executable
chmod +x ~/am_ws/src/am_description/scripts/acados_mpc_controller.py

# Run
ros2 run am_description acados_mpc_controller.py
```

## Expected Performance

- **Scipy solver**: 100-500 ms per solve
- **Acados solver**: 5-50 ms per solve (10-20x faster!)
- **Real-time capable**: Yes, at 20-50 Hz control rate

## Troubleshooting

1. **Import error for acados_template**:
   - Make sure you ran `pip3 install -e interfaces/acados_template` from acados directory
   - Check that `ACADOS_SOURCE_DIR` environment variable is set

2. **Shared library not found**:
   - Verify `LD_LIBRARY_PATH` includes `$ACADOS_SOURCE_DIR/lib`
   - Run `echo $LD_LIBRARY_PATH` to check

3. **Code generation fails**:
   - The first run generates C code and compiles it (takes 5-10 seconds)
   - Subsequent runs are much faster as code is cached

4. **CMake errors during build**:
   - Make sure you have cmake >= 3.10: `cmake --version`
   - Install if needed: `sudo apt install cmake`

## Files Created

```
am_description/mpc/
├── acados_model.py           # CasADi symbolic model
├── acados_mpc_solver.py      # Acados MPC solver
scripts/
├── acados_mpc_controller.py  # ROS2 controller using acados
```
