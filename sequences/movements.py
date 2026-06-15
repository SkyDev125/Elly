"""User-defined movement routines for Elly.

You can edit this file freely on your laptop to define new movement routines.
Each dictionary should have:
  - "direction": "forward" | "back" | "left" | "right" | "rotate_left" | "rotate_right" | "stop" | "hold" | "navigate"
  - "amount": distance in meters (for translation), angle in degrees (for rotation), seconds (for stop/hold), or [x, y, yaw_deg] (for navigate)
  - "speed": (optional) speed in m/s (default 0.1) or rad/s (default 0.3)
"""

def low():
    """Simple polite cue: advance 0.10m and stop."""
    return [
        {"direction": "forward", "amount": 0.1, "speed": 0.5},
        {"direction": "stop", "amount": 1},
    ]


def medium():
    """Insistent cue: perform 3 quick center-left-right-center sweeps."""
    return low() + [
        {"direction": "rotate_left", "amount": 10, "speed": 0.5},
        {"direction": "rotate_right", "amount": 20, "speed": 0.5},
        {"direction": "rotate_left", "amount": 20, "speed": 0.5},
        {"direction": "rotate_right", "amount": 20, "speed": 0.5},
        {"direction": "rotate_left", "amount": 10, "speed": 0.5},
        {"direction": "stop", "amount": 1},
    ]


def high(clearance=0.40):
    """High cue: advance 0.20m, sweep, return, and hold.
    
    This function accepts an optional parameter so you can customize it from the command line!
    Example: elly_move high 0.5
    """
    return [
        {"direction": "forward", "amount": 0.20, "speed": 1.0},
        {"direction": "rotate_left", "amount": 20, "speed": 1},
        {"direction": "rotate_right", "amount": 40, "speed": 1},
        {"direction": "rotate_left", "amount": 40, "speed": 1},
        {"direction": "rotate_right", "amount": 40, "speed": 1},
        {"direction": "rotate_left", "amount": 15, "speed": 1},
        {"direction": "stop", "amount": 1},
    ]

def rotate_180():
    return [
        {"direction": "rotate_left", "amount": 150, "speed": 1.0},
        {"direction": "stop", "amount": 1},
    ]

def move_to_point(x=-3.2, y=-1.7, yaw_deg=30):
    """Navigate to an exact map coordinate and stop."""
    return [
        {"direction": "navigate", "amount": [x, y, yaw_deg]},
        {"direction": "stop", "amount": 1},
    ]


def navigate_to_point():
    """Backward-compatible name for the original fixed destination."""
    return move_to_point()


def move_behind_human(x=-3.2, y=-1.7, yaw_deg=30):
    """Backward-compatible descriptive name."""
    return move_to_point(x, y, yaw_deg)


def move_to_behind_human_point():
    """Backward-compatible name for the original fixed destination."""
    return move_behind_human()


def creep_in(target_distance=0.5, speed=0.05, duration=30.0):
    """Slowly creep forward, stopping when close to human's legs or obstacles.
    
    If the human moves away, the robot continues creeping forward to maintain target_distance.
    
    Example: elly_move creep_in 0.35 0.05 45
    """
    return [
        {
            "direction": "creep",
            "amount": target_distance,
            "speed": speed,
            "duration": duration
        }
    ]


def follow_me(
    x=-3.2,
    y=-1.7,
    yaw_deg=30,
    turn_speed=1.0,
    look_interval=20.0,
    detect_range=0.5,
    wait_timeout=15.0,
):
    """Lead a person to a Nav2 goal with periodic LiDAR lookbacks.

    The person enters the rear detection zone to start. Every look_interval
    seconds, Nav2 is cancelled, the robot turns 180 degrees, and waits up to
    wait_timeout seconds. The final goal uses the same timed person check.

    turn_speed is retained for command compatibility. Follow-me deliberately
    uses the proven rotate_180() routine at speed 1.0. Nav2 path speed is
    configured in config/myagv_nav2.yaml.

    Example: elly_move follow_me 1.5 -0.5 90 0.4 20 0.5 15
    """
    destination_steps = move_to_point(x, y, yaw_deg)
    return [
        {
            "direction": "lead",
            "amount": destination_steps[0]["amount"],
            "speed": turn_speed,
            "look_interval": look_interval,
            "detect_range": detect_range,
            "wait_timeout": wait_timeout,
            "rotation_steps": rotate_180(),
            "destination_steps": destination_steps,
        }
    ]


def temp():
    return follow_me(-3.2, -1.7, 190, 0.4, 30, 0.5, 15.0)

def stop():
    """Stop and hold position for 5 seconds."""
    return [
        {"direction": "stop", "amount": 5},
    ]
