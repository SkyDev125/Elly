#!/usr/bin/env python3
"""Live Elly OS command dashboard."""

import os
import re
import subprocess
import sys
import time


WIDTH = 78
SERVICES = (
    (
        "LiDAR + base",
        "lidar",
        "lidar_on / lidar_off",
        "Wheel driver, /cmd_vel, odometry, and 2D laser scans.",
    ),
    (
        "Depth camera",
        "camera",
        "camera_on / camera_off",
        "Astra Pro2 RGB/depth streams for perception and 3D mapping.",
    ),
    (
        "2D mapping",
        "slam_2d",
        "map_2d_on / map_2d_off",
        "Gmapping builds a 2D map from the live LiDAR scan.",
    ),
    (
        "3D mapping",
        "rtabmap",
        "map_3d_on / map_3d_off",
        "RTAB-Map fuses depth and LiDAR into a 3D database.",
    ),
    (
        "Motion service",
        "motion_service",
        "motion_on / motion_off",
        "Persistent controller that receives and executes elly_move commands.",
    ),
    (
        "Navigation",
        "navigation",
        "nav_off",
        "Nav2/AMCL session started by map_2d_load for loaded-map path planning.",
    ),
)

SERVICE_REQUIREMENTS = {
    "lidar": ("LiDAR + base", "lidar_on", "lidar_on"),
    "camera": ("Depth camera", "camera_on", "camera_on"),
    "slam_2d": ("2D mapping", "map_2d_on", "map_2d_on"),
    "rtabmap": ("3D mapping", "map_3d_on", "map_on"),
    "motion_service": ("Motion service", "motion_on", "motion_on"),
    "navigation": (
        "Navigation",
        "map_2d_load MAP_NAME",
        None,
    ),
}


def colors_enabled():
    return sys.stdout.isatty() and "NO_COLOR" not in os.environ


if colors_enabled():
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    CYAN = "\033[38;5;45m"
    GREEN = "\033[38;5;42m"
    YELLOW = "\033[38;5;214m"
    RED = "\033[38;5;196m"
else:
    RESET = BOLD = DIM = CYAN = GREEN = YELLOW = RED = ""


def parse_screen_sessions(output):
    return set(re.findall(r"\d+\.([^\s]+)\s+\(", output))


def fetch_screen_sessions(user, host):
    remote = f"{user}@{host}"
    command = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=4",
        remote,
        "screen -ls 2>/dev/null || true",
    ]
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=7,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return set(), str(error)
    if result.returncode != 0:
        message = result.stderr.strip() or f"SSH exited with code {result.returncode}"
        return set(), message
    return parse_screen_sessions(result.stdout), None


def start_remote_service(user, host, remote_command):
    result = subprocess.run(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            f"{user}@{host}",
            f"bash ~/scripts/brain.sh {remote_command}",
        ],
        check=False,
    )
    return result.returncode == 0


def ensure_services(service_names, user=None, host=None):
    host = host or os.environ.get("NANO_IP", "192.168.0.200")
    user = user or os.environ.get("NANO_USER", "er")
    sessions, error = fetch_screen_sessions(user, host)
    if error:
        print(f"{RED}[x]{RESET} Cannot check robot services: {error}", file=sys.stderr)
        return False

    for service_name in service_names:
        if service_name in sessions:
            continue
        label, operator_command, remote_command = SERVICE_REQUIREMENTS[service_name]
        print(f"\n{YELLOW}[!]{RESET} {label} is required but currently OFF.")
        if remote_command is None:
            print(f"    Start it first with: {BOLD}{operator_command}{RESET}")
            return False
        if not sys.stdin.isatty():
            print(f"    Start it first with: {BOLD}{operator_command}{RESET}")
            return False

        answer = input(f"    Start {label} now? [Y/n] ").strip().lower()
        if answer not in ("", "y", "yes"):
            print("    Command cancelled.")
            return False
        if not start_remote_service(user, host, remote_command):
            print(
                f"{RED}[x]{RESET} Could not start {label}. "
                f"Try {operator_command} directly for details.",
                file=sys.stderr,
            )
            return False

        for _ in range(20):
            sessions, error = fetch_screen_sessions(user, host)
            if error is None and service_name in sessions:
                print(f"{GREEN}[ok]{RESET} {label} is ON.")
                break
            time.sleep(0.25)
        else:
            print(f"{RED}[x]{RESET} {label} did not become ready.", file=sys.stderr)
            return False
    return True


def rule(character="-"):
    print(f"  {DIM}{character * WIDTH}{RESET}")


def heading(title):
    print(f"\n  {BOLD}{CYAN}{title}{RESET}")
    rule()


def render_header(remote, online):
    state = f"{GREEN}ONLINE" if online else f"{RED}OFFLINE"
    print(f"\n{BOLD}{CYAN}  +{'-' * WIDTH}+{RESET}")
    print(f"{BOLD}{CYAN}  |{'ELLY OS':^{WIDTH}}|{RESET}")
    print(f"{BOLD}{CYAN}  |{'Robot Operator Console':^{WIDTH}}|{RESET}")
    print(f"{BOLD}{CYAN}  +{'-' * WIDTH}+{RESET}")
    print(f"  Robot  {remote:<37} Status  {BOLD}{state}{RESET}")


def render_services(sessions, online):
    heading("SERVICES")
    for label, screen_name, command, description in SERVICES:
        if not online:
            badge = f"{YELLOW}[ --]{RESET}"
        elif screen_name in sessions:
            badge = f"{GREEN}{BOLD}[ON ]{RESET}"
        else:
            badge = f"{DIM}[OFF]{RESET}"
        print(f"  {badge}  {label:<17} {DIM}{command}{RESET}")
        print(f"         {DIM}{description}{RESET}")


def render_commands():
    heading("MOVEMENT")
    print("  elly_move ROUTINE [args...]        Run a recipe from sequences/movements.py")
    print("  elly_move DIRECTION AMOUNT [...]   Run one bounded movement primitive")
    print("  elly_move_status                   Show controller state and sensor details")
    print("  elly_move_stop                     Abort movement and publish zero velocity")

    heading("NAVIGATION")
    print("  elly_nav X Y [yaw_deg]             Local ROS client sending a Nav2 goal")
    print("  elly_autofind                      Move autonomously until AMCL converges")
    print("  nav_off                            Stop the background Nav2/AMCL session")

    heading("MAPS AND OPERATOR TOOLS")
    print("  map_2d_save / map_2d_load          Save a Gmapping map or load it into Nav2")
    print("  map_3d_save / map_3d_load          Save or restore the RTAB-Map database")
    print("  teleop                             Publish keyboard /cmd_vel commands directly")
    print("  start_rviz                         Open the local robot/map visualization")
    print("  robot_status                       Print raw Jetson screen sessions")
    print("  robot_peek                         Attach to a Jetson screen session")
    print("  elly_deploy                        Validate, upload, and restart the controller")
    print()


def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == "require" and len(sys.argv) > 2:
            unknown = [name for name in sys.argv[2:] if name not in SERVICE_REQUIREMENTS]
            if unknown:
                print(f"Unknown service requirement: {', '.join(unknown)}", file=sys.stderr)
                return 2
            return 0 if ensure_services(sys.argv[2:]) else 1
        print("Usage: elly [require SERVICE ...]", file=sys.stderr)
        return 2

    host = os.environ.get("NANO_IP", "192.168.0.200")
    user = os.environ.get("NANO_USER", "er")
    remote = f"{user}@{host}"
    sessions, error = fetch_screen_sessions(user, host)

    render_header(remote, error is None)
    render_services(sessions, error is None)
    if error:
        print(f"\n  {YELLOW}[!] Live status unavailable:{RESET} {error}")
    render_commands()
    return 0


if __name__ == "__main__":
    sys.exit(main())
