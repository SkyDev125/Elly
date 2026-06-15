# Elly

Remote ROS 2 control tools for an Elephant Robotics myAGV running ROS 2 Galactic on a Jetson Nano.

The laptop acts as the operator console. Hardware drivers, mapping, and movement controller run on the Jetson in detached `screen` sessions over SSH, orchestrated by a remote process switchboard and a unified python movement service.

## Setup

From the laptop:

```bash
cd ~/Elly
bash scripts/setup.sh
source ~/.bashrc
```

The setup script:

- Configures SSH access to the robot.
- Installs `screen` on the Jetson if missing.
- Deploys the switchboard (`brain.sh`) and the unified movement controller (`brain.py`).
- Builds the local robot-description package (`myagv_description`).
- Appends the Elly console commands and aliases to your laptop's `~/.bashrc`.

All Jetson movement and follow-me controller logic lives in `scripts/brain.py`;
movement recipes remain in `sequences/movements.py`.

---

## Process Toggles

Run `elly` to print the command reference. These toggle background screen sessions on the Jetson:

- `camera_on` / `camera_off`: Orbbec Astra camera.
- `lidar_on` / `lidar_off`: Base driver and YDLidar.
- `base_on` / `base_off`: Base driver without LiDAR.
- `map_2d_on` / `map_2d_off`: Gmapping (2D SLAM).
- `map_3d_on` / `map_3d_off`: RTAB-Map (3D SLAM).
- `motion_on` / `motion_off`: Persistent movement controller daemon.

---

## Movement Commands

The movement controller is a simple closed-loop system using odometry feedback (`/odom`).
First, ensure `motion_on` is running, then use `elly_move` to send commands.

### Option A: Edit and Run Python Routines (Recommended)

You can write and compose your movement sequences directly in Python on your laptop by editing the file:
👉 [sequences/movements.py](file:///home/sky/Elly/sequences/movements.py)

Each Python function in that file returns a list of step dictionaries. You can execute any of these routines by name:

```bash
# Run the polite routine defined in movements.py
elly_move polite

# Run the insistent routine defined in movements.py
elly_move insistent
```

#### Parameterizing your Routines
You can also pass arguments from your terminal directly to your Python functions!
For example, the template `high` routine accepts a `clearance` parameter:
```bash
# Runs high routine with clearance = 0.50
elly_move high 0.50
```

---

### Option B: Single Step Movements

Run a single step directly from the command line:

```bash
elly_move <direction> <amount> [speed]
```

- **`direction`**:
  - Linear: `forward` (or `fwd`), `back` (or `backward`, `bwd`), `left`, `right`
  - Angular: `rotate_left` (or `left_turn`, `ccw`), `rotate_right` (or `right_turn`, `cw`)
- **`amount`**: Distance in meters for linear, or angle in degrees for rotation.
- **`speed`** *(optional)*: Speed in m/s (default `0.1`) for linear, or rad/s (default `0.3`) for rotation.

**Examples:**
```bash
elly_move forward 0.2
elly_move left 0.15 0.08
elly_move rotate_left 90
```

---

### Option C: Multi-Step Sequences (JSON Streaming)

You can also stream a raw JSON string or JSON file directly:

```bash
# Option A: Send a raw JSON string
elly_move sequence '[{"direction": "forward", "amount": 0.2}, {"direction": "rotate_left", "amount": 90}]'

# Option B: Run a local JSON file
elly_move sequence dance.json
```

---

## Monitoring and Stopping

- `elly_move_status`: Queries the movement controller for current active action, error, and progress.
- `elly_move_stop`: Immediately aborts any running movement/sequence and stops the robot.
- `robot_status`: Lists active `screen` sessions running on the Jetson.
- `robot_peek <session_name>`: Attach to a screen session console (e.g. `robot_peek motion_service`). Detach with `Ctrl+A`, then `D`.

---

## Follow-Me

`follow_me` leads a person to a Nav2 map goal:

```bash
elly_move follow_me X Y YAW_DEG TURN_SPEED LOOK_INTERVAL DETECT_RANGE WAIT_TIMEOUT
```

Example:

```bash
elly_move follow_me -3.2 -1.7 190 0.4 20 0.5 15
```

Behavior:

1. The robot remains stopped and watches the LiDAR sector exposed by the body.
2. A presence within `DETECT_RANGE` for three consecutive scans starts navigation.
3. After each `LOOK_INTERVAL`, Nav2 is cancelled and the robot runs the fixed `rotate_180()` movement.
4. The robot waits up to `WAIT_TIMEOUT` for a persistent presence inside the visible LiDAR sector. If found, it turns another 180 degrees and resumes the goal with a fresh interval.
5. At the goal it turns 180 degrees and performs the same timed check. Success leaves the robot facing the person.

`follow_me()` is an orchestration of existing movement recipes:

- Every lookback calls `rotate_180()`: `rotate_left 180` at speed `1.0`, followed by `stop 1`.
- Its destination comes from `move_to_point(X, Y, YAW)`, including that recipe's final stop.
- Human detection is follow-me logic. It shares only the neutral visible-sector LiDAR clearance helper also used by `creep_in()`; it does not call or depend on the `creep_in()` movement.

`TURN_SPEED` remains accepted for compatibility but is not used. Nav2 path speed is configured in `config/myagv_nav2.yaml`. LiDAR cannot distinguish a person from furniture, so keep the visible detection sector clear.

Follow-me requires `lidar_on`, a loaded 2D map/Nav2 session, AMCL localization, and `motion_on`.

---

## Safety

Test with the robot raised or in a clear floor area first. Keep `elly_move_stop` ready, use conservative speeds, and do not run teleoperation concurrently with automated movement commands.
