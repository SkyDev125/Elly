# elly_lp

Scripts for remote management of ROS 2 nodes on the MyAGV Jetson Nano. 

This repository uses a two-part script system to run robot processes in detached `screen` sessions on the Nano, controlled via SSH aliases on the local laptop.

## Setup

1. **On the Jetson Nano:**
   - Ensure `screen` is installed: `sudo apt install screen`
   - Place `brain.sh` in `~/scripts/brain.sh`.
   - Make it executable: `chmod +x ~/scripts/brain.sh`

2. **On the Laptop:**
   - Run the setup script to configure SSH keys, build meshes, and set up bash aliases:
     ```bash
     cd elly_lp
     bash scripts/setup.sh
     source ~/.bashrc
     ```

## Command Reference

Run `elly` in the terminal to view this list locally.

### Nano Processes (Background)
- `lidar_on` / `lidar_off` : Starts/stops the YDLidar and base driver.
- `camera_on` / `camera_off` : Starts/stops the Orbbec Astra depth camera.
- `map_2d_on` / `map_2d_off` : Starts/stops SLAM Toolbox (2D mapping).
- `map_3d_on` / `map_3d_off` : Starts/stops RTAB-Map (3D mapping).

### Laptop Processes (Local)
- `teleop` : Launches keyboard teleoperation.
- `start_rviz` : Opens RViz with the `elly_dash.rviz` configuration.

### System Utilities
- `robot_status` : Lists all active `screen` sessions running on the Nano.
- `robot_peek <session>` : Attaches to a remote screen session to view live logs (e.g., `robot_peek rtabmap`). Use `Ctrl+A` then `D` to detach without killing the process.
- `map2d_save` : Triggers a map save on the Nano and transfers it to the laptop.
- `map2d_load` : Loads a saved map for navigation.
