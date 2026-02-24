# RB-Y1 Development Progress

## Environment Setup

### Prerequisites Installed
- **Docker Desktop** (v29.2.1) -- installed via `brew install --cask docker-desktop`
- **XQuartz** (v2.8.5) -- installed for X11 support
- **Conda env**: `rby1` with `rby1_sdk` installed

### Simulator: MuJoCo in Docker

The RB-Y1 simulator runs in Docker using the x86 image under Rosetta emulation (the ARM image has a missing `librby1-sdk.so` bug). A virtual framebuffer (Xvfb) + noVNC provides browser-based visualization.

**Start the simulator:**
```bash
docker rm -f rby1-sim 2>/dev/null; docker run -d \
  --platform linux/amd64 \
  --name rby1-sim \
  -p 50051:50051 -p 5173:5173 -p 6080:6080 \
  -e DEBIAN_FRONTEND=noninteractive \
  --entrypoint /bin/bash \
  rainbowroboticsofficial/rby1-sim:latest \
  -c 'apt-get update > /dev/null 2>&1; apt-get install -y -o Dpkg::Options::="--force-confdef" xvfb libgl1-mesa-dri x11vnc novnc websockify > /dev/null 2>&1; Xvfb :99 -screen 0 1280x960x24 & sleep 2; export DISPLAY=:99; export LIBGL_ALWAYS_SOFTWARE=1; x11vnc -display :99 -forever -nopw -shared -rfbport 5900 > /dev/null 2>&1 & websockify --web /usr/share/novnc 6080 localhost:5900 > /dev/null 2>&1 & sleep 1; cd /root/exe/app; ./app_main'
```

**View the simulator:** Open http://localhost:6080/vnc.html in your browser.

**Stop the simulator:**
```bash
docker rm -f rby1-sim
```

**Notes:**
- Package install takes ~60s on first start (Rosetta emulation overhead)
- Timing warnings in logs are expected (emulation can't guarantee 2ms real-time)
- gRPC server listens on `localhost:50051`
- Web UI port 5173 also mapped

### Network Options for Real Robot

| Method | MacBook WiFi | Robot Connection | Use Case |
|--------|-------------|-----------------|----------|
| Robot hotspot | Not available (no internet) | WiFi to 192.168.12.1 | Quick testing |
| USB-C Ethernet + WiFi | Connected (internet works) | Ethernet to robot | Development with internet |
| Docker simulator | Connected (internet works) | localhost:50051 | No physical robot needed |

For Ethernet setup: USB-C adapter, static IP `192.168.12.2/24` on the Ethernet interface, WiFi stays on home network.

## SDK Quick Reference

### Connection & Initialization

```python
import rby1_sdk as rby
import numpy as np
import time

robot = rby.create_robot_a('localhost:50051')  # sim
# robot = rby.create_robot_a('192.168.12.1:50051')  # real robot
robot.connect()

robot.power_on('.*')
time.sleep(0.5)
robot.servo_on('.*')
time.sleep(0.5)
robot.reset_fault_control_manager()
time.sleep(1)
robot.enable_control_manager(unlimited_mode_enabled=True)
time.sleep(1)
```

Important: Run the full init sequence in a single Python process. The control manager goes idle when the connection drops.

### Moving Arms (Joint Position)

```python
model = robot.model()

def move(torso, right, left, t=3):
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(rby.JointPositionCommandBuilder().set_minimum_time(t).set_position(torso))
            .set_right_arm_command(rby.JointPositionCommandBuilder().set_minimum_time(t).set_position(right))
            .set_left_arm_command(rby.JointPositionCommandBuilder().set_minimum_time(t).set_position(left))
        )
    )
    return robot.send_command(rc).get().finish_code

# Ready pose
move(np.deg2rad([0, 45, -90, 45, 0, 0]),
     np.deg2rad([0, -5, 0, -120, 0, 70, 0]),
     np.deg2rad([0, 5, 0, -120, 0, 70, 0]), t=5)

# Arms out
move(np.deg2rad([0, 30, -60, 30, 0, 0]),
     np.deg2rad([0, -90, 0, -45, 0, 30, 0]),
     np.deg2rad([0, 90, 0, -45, 0, 30, 0]))
```

### Reading Robot State

```python
state = robot.get_state()
model = robot.model()

print(state.position[model.torso_idx])      # 6 torso joints
print(state.position[model.right_arm_idx])   # 7 right arm joints
print(state.position[model.left_arm_idx])    # 7 left arm joints

# Available state properties:
# battery_state, center_of_mass, collisions, current, ft_sensor_left,
# ft_sensor_right, is_ready, joint_states, odometry, position, power_states,
# target_position, target_velocity, timestamp, tool_flange_left,
# tool_flange_right, torque, velocity
```

### Gripper Control (Real Robot Only)

Grippers use Dynamixel motors via `/dev/rby1_gripper` on the UPC. Not available in the simulator.

```bash
conda run -n rby1 python examples/python/gripper_control.py --address 192.168.12.1:50051
```

Interactive commands: `open`, `close`, `0.0`-`1.0` (normalized position), `read`, `quit`

See `examples/python/gripper_control.py` for the full implementation.

### Joint Name Reference

| Group | Joints | Count |
|-------|--------|-------|
| Wheels | right_wheel, left_wheel | 2 |
| Torso | torso_0 through torso_5 | 6 |
| Right arm | right_arm_0 through right_arm_6 | 7 |
| Left arm | left_arm_0 through left_arm_6 | 7 |
| Head | head_0, head_1 | 2 |
| **Total** | | **24** |

## What Works

- [x] Docker simulator running on Apple Silicon (Rosetta x86 emulation)
- [x] noVNC browser visualization at http://localhost:6080/vnc.html
- [x] SDK connection to simulator (`localhost:50051`)
- [x] Power on, servo on, control manager enable
- [x] Joint position commands (torso, arms)
- [x] Robot state reading
- [x] Gripper control script written (ready for real hardware)

## Next Steps

- [ ] Test gripper control on real robot (requires physical connection)
- [ ] Teleoperation setup with master arm (`examples/python/17_teleoperation_with_joint_mapping.py`)
- [ ] Cartesian/impedance control for task-space manipulation
- [ ] Command streaming for real-time control
- [ ] Kinematics/dynamics usage for planning
