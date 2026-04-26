# ▶️ Usage Guide

Extremely under progress

This guide explains how to run the F1TENTH stack on the real vehicle and in simulation, along with useful commands for debugging and development.

---

## 🚀 Quick Start

- Run the real setup by following the steps in the Real Vehicle Setup section.

- Run the full simulation stack using 5 terminals by following the steps in the simulation workflow section.


## Real Vehicle Setup

Follow the official setup tutorial:

```bash
docs/Tutorial_basico_f1tenth_V_Vinicius_260304.pdf
```
or [here](docs/Tutorial_basico_f1tenth_V_Vinicius_260304.pdf).


This includes:

- Hardware bringup
- Sensor configuration
- Calibration

Some ROS 2 stack and commands used in simulation apply to the real vehicle.


## Simulation Workflow
- Overview
- Terminal breakdown
- Modes (mapping, localization, MPC)

This refers to mapping
```bash
# Terminal 1 — Visualization (Foxglove)
ros2 run foxglove_bridge foxglove_bridge

# Terminal 2 — Simulator bridge
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 3 — Main stack bringup
ros2 launch f1tenth_stack bringup_launch_joy.py

# Terminal 4 — Localization (Cartographer)
ros2 launch cartographer_ros backpack_2d.launch.py

# Terminal 5 — Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Useful Commands
- ROS tools
- Debugging
- Build system

Setup launch commands:
```bash
ros2 run foxglove_bridge foxglove_bridge
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch f1tenth_stack bringup_launch_joy.py
ros2 launch cartographer_ros backpack_2d.launch.py

ros2 launch cartographer_ros backpack_2d_loc.launch.py
ros2 launch f1tenth_stack mpc.py MPC:=mpc_curv_ls_v3_AAMMDD
```

ROS Introspection
```bash
ros2 node list
ros2 topic list
ros2 topic echo /topic_name
ros2 topic hz /topic_name
```

Graph Visualization
```bash
rqt_graph
```

TF Debugging
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

Build System
```bash
colcon list
colcon build --packages-select <package_name>
colcon build --symlink-install
```