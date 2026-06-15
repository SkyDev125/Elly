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


def navigate_to_point():
    """Navigate to a coordinate [x, y, yaw_deg] and perform a quick search scan.
    
    Example: elly_move navigate_and_scan 1.2 -0.5 90
    """
    return [
        # Look at the human's location
        
        # Human Location:
        # {"direction": "navigate", "amount": [-2.4, -1.4, 190]},
        
        # In front of human
        #{"direction": "navigate", "amount": [-1.6, -1.1, 190]},

        # Get in front of human's path (Broadside)
        #{"direction": "navigate", "amount": [0, 0, 120]},

        # Get behind Human
        {"direction": "navigate", "amount": [-3.2, -1.7, 30]},

        {"direction": "stop", "amount": 1},
    ]


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


def follow_me(x, y, yaw_deg=0.0, speed=0.4, look_interval=8.0, detect_range=1.5):
    """Guided navigation: path to [x, y, yaw_deg], stopping to look back for the human every look_interval seconds.
    
    If the human falls behind (further than detect_range), the robot waits until they catch up before resuming.
    
    Example: elly_move follow_me 1.5 -0.5 90 0.4 6.0 1.2
    """
    return [
        {
            "direction": "lead",
            "amount": [x, y, yaw_deg],
            "speed": speed,
            "look_interval": look_interval,
            "detect_range": detect_range
        }
    ]

def temp() :
    return follow_me(-3.2, -1.7, 190, 0.4, 12.0, 0.5)

def stop():
    """Stop and hold position for 5 seconds."""
    return [
        {"direction": "stop", "amount": 5},
    ]
