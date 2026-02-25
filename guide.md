# RB-Y1 Development Workstation Setup Guide

## Context

This guide was generated during a setup session on a Windows 11 machine. The decision was made
to use **native Ubuntu (dual boot)** instead of WSL2 for all robot development. This document
captures the full plan so work can continue seamlessly on the Ubuntu side.

## Hardware / Environment

- **Robot:** RB-Y1 Model A (24 DOF, 2-wheel differential drive)
- **Workstation GPU:** NVIDIA (specific model TBD -- run `nvidia-smi` to confirm)
- **OS:** Ubuntu (dual boot with Windows 11)
- **Network:** Robot and workstation share the same LAN
- **Robot gRPC endpoint:** `<ROBOT_IP>:50051` (replace with actual IP)
- **SDK repo cloned at:** This repository (`rby1-sdk`, version 0.9.1, branch `koh-dev/windows`)

## Why Native Ubuntu Over WSL2

- Direct LAN access to the robot with zero networking abstraction
- Bare metal latency for the 500Hz (2ms) UDP real-time control loop
- Native CUDA driver -- no GPU passthrough overhead
- USB devices (master arm via `/dev/ttyUSB*`) work without `usbipd-win`
- All robotics tools (MuJoCo, ROS, etc.) are Linux-native

## Goals

1. **Policy training** -- both imitation learning and reinforcement learning
2. **Policy deployment** -- real-time control of the physical robot over LAN
3. **Data collection** -- teleoperation via master arm or scripted demonstrations

---

## Step 1: Verify NVIDIA Drivers + CUDA

The NVIDIA proprietary driver must be installed (not nouveau). Do NOT install CUDA drivers
from the NVIDIA .run installer -- use the package manager path to avoid breaking the display
driver.

```bash
# Check that the NVIDIA driver is loaded
nvidia-smi
```

If `nvidia-smi` shows your GPU, skip to installing the CUDA toolkit. If not:

```bash
sudo apt update
sudo apt install nvidia-driver-560
sudo reboot
```

After reboot, confirm with `nvidia-smi`, then install the CUDA toolkit:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-6
```

> **Note:** Adjust the repo URL if your Ubuntu version is not 24.04.
> For Ubuntu 22.04, replace `ubuntu2404` with `ubuntu2204`.

Add to `~/.bashrc`:

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

Then `source ~/.bashrc`.

## Step 2: Network Connectivity to Robot

Verify you can reach the robot on your LAN:

```bash
# Find your workstation IP
ip addr show

# Ping the robot (replace with actual IP)
ping <ROBOT_IP>

# Check that gRPC port is reachable
nc -zv <ROBOT_IP> 50051
```

If the robot is not discoverable, scan the subnet:

```bash
sudo apt install nmap
nmap -sn 192.168.1.0/24
```

## Step 3: Python Environment

Use conda to isolate the robot development environment:

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
```

Restart shell, then:

```bash
conda create -n rby1 python=3.11 -y
conda activate rby1
```

## Step 4: Install the RB-Y1 SDK

### Option A: From PyPI (recommended to start)

```bash
pip install rby1-sdk
```

Verify:

```bash
python -c "import rby1_sdk; print(rby1_sdk.__version__)"
```

### Option B: From source (if you need to modify C++ internals)

```bash
cd /path/to/rby1-sdk
git submodule update --init --recursive
sudo apt-get install build-essential cmake ninja-build
pip install conan>=2.4 scikit-build>=0.17.3 skbuild-conan pybind11-stubgen
pip install -e .
```

Building from source compiles gRPC 1.72.0 and all C++ dependencies. Expect 15-30+ minutes.

## Step 5: Verify Robot Connection

```python
import rby1_sdk

robot = rby1_sdk.create_robot("<ROBOT_IP>:50051", "a")
robot.connect()
print("Connected:", robot.is_connected())
print(robot.get_robot_info())
```

If this prints robot info, your SDK and network are working.

## Step 6: ML Training Stack

```bash
# PyTorch with CUDA
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124

# Core scientific stack
pip install numpy scipy matplotlib tensorboard h5py

# Simulation
pip install mujoco

# Verify GPU
python -c "import torch; print('CUDA:', torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

MuJoCo-ready robot models are included in this repo at `models/`.

### Policy frameworks (install as needed)

```bash
pip install robomimic       # Imitation learning (ACT, Diffusion Policy)
pip install gymnasium       # RL environment interface
```

## Step 7: Data Collection (Teleoperation)

The SDK supports the master arm for kinesthetic teaching. See:

- `examples/python/19_master_arm.py` -- master arm teleoperation
- `examples/python/17_teleoperation_with_joint_mapping.py` -- joint-mapped teleop
- `examples/python/record.py` -- record demonstrations
- `examples/python/replay.py` -- replay recorded demonstrations

The master arm connects via USB (Dynamixel servos). Ensure your user is in the `dialout` group:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```

## Step 8: Real-Time Control

For deploying learned policies, use the real-time control interface.
See `examples/python/28_real_time_control.py` for the pattern:

- 500Hz control loop over UDP
- `ControlState` provides current joint positions/velocities/torques
- `ControlInput` sends target positions + feedforward torques
- Trapezoidal motion generator for smooth trajectories

Key classes:
- `rby1_sdk.Robot_A` -- main robot handle
- `rby1_sdk.Robot_A_ControlState` -- state feedback in the control loop
- `rby1_sdk.Robot_A_ControlInput` -- command output from the control loop
- `rby1_sdk.math.TrapezoidalMotionGenerator` -- trajectory smoothing

## SDK Architecture Reference

| Component | Protocol | Latency | Use Case |
|-----------|----------|---------|----------|
| gRPC (port 50051) | TCP | ~1-10ms | Commands, state queries, power, config |
| UDP real-time | Custom binary + CRC8 | <1ms | 500Hz closed-loop control |
| Dynamixel (USB) | Serial | ~1ms | Master arm teleoperation |

## Robot Model A Joint Layout (24 DOF)

```
Index  Joint
0-1    right_wheel, left_wheel
2-7    torso_0 through torso_5
8-14   right_arm_0 through right_arm_6
15-21  left_arm_0 through left_arm_6
22-23  head_0, head_1
```

## Key File Locations in This Repo

```
examples/python/         -- 40+ Python examples (start here)
models/y1_model_a/       -- URDF and MuJoCo models for Model A
protos/                  -- gRPC/protobuf service definitions
include/rby1-sdk/        -- C++ public headers
python/                  -- pybind11 binding source
```

## Resources

- SDK Documentation: https://rainbowrobotics.github.io/rby1-dev/
- GitHub Discussions: https://github.com/RainbowRobotics/rby1-sdk/discussions
- Support Email: rby.support@rainbow-robotics.com
