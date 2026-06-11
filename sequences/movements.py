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
        {"direction": "hold", "amount": 1},
    ]


def medium():
    """Insistent cue: perform 3 quick center-left-right-center sweeps."""
    return [
        {"direction": "rotate_left", "amount": 10, "speed": 0.5},
        {"direction": "rotate_right", "amount": 20, "speed": 0.5},
        {"direction": "rotate_left", "amount": 20, "speed": 0.5},
        {"direction": "rotate_right", "amount": 20, "speed": 0.5},
        {"direction": "rotate_left", "amount": 20, "speed": 0.5},
        {"direction": "rotate_right", "amount": 20, "speed": 0.5},
        {"direction": "hold", "amount": 1},
        {"direction": "rotate_left", "amount": 10, "speed": 0.5},
        {"direction": "hold", "amount": 1},
    ]


def high(clearance=0.40):
    """High cue: advance 0.20m, sweep, return, and hold.
    
    This function accepts an optional parameter so you can customize it from the command line!
    Example: elly_move high 0.5
    """
    return [
        {"direction": "forward", "amount": 0.20, "speed": 1.0},
        {"direction": "rotate_left", "amount": 15, "speed": 1.0},
        {"direction": "rotate_right", "amount": 30, "speed": 1.0},
        {"direction": "rotate_left", "amount": 30, "speed": 1.0},
        {"direction": "hold", "amount": 1},
        {"direction": "rotate_right", "amount": 15, "speed": 1.0},
        {"direction": "backward", "amount": 0.2, "speed": 1.0},
        {"direction": "hold", "amount": 1},
    ]


def navigate_and_scan(x, y, yaw_deg=0.0):
    """Navigate to a coordinate [x, y, yaw_deg] and perform a quick search scan.
    
    Example: elly_move navigate_and_scan 1.2 -0.5 90
    """
    return [
        {"direction": "navigate", "amount": [x, y, yaw_deg]},
        {"direction": "hold", "amount": 1.0},
        {"direction": "rotate_left", "amount": 30, "speed": 0.5},
        {"direction": "rotate_right", "amount": 60, "speed": 0.5},
        {"direction": "rotate_left", "amount": 30, "speed": 0.5},
        {"direction": "hold", "amount": 1.0},
    ]
