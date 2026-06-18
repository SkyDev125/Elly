"""User-defined movement routines for Elly.

You can edit this file freely on your laptop to define new movement routines.
Each dictionary should have:
  - "direction": "forward" | "back" | "left" | "right" | "rotate_left" | "rotate_right" | "stop" | "hold" | "navigate"
  - "amount": distance in meters (for translation), angle in degrees (for rotation), seconds (for stop/hold), or [x, y, yaw_deg] (for navigate)
  - "speed": (optional) speed in m/s (default 0.1) or rad/s (default 0.3)



"""

from dataclasses import dataclass

@dataclass
class Position:
    x: float
    y: float
    yaw: float

robot_starting = Position(0.85, 1.9, 90)
human_objective = Position(-2.4, -1.4, 190)
behind_human_objective = Position(-3.2, -1.7, 20)
front_human_objective = Position(-1.5, -0.9, 190)
blocking_human_path = Position(0, 0, 120)

"""
Primitive Routines
"""
def stop(seconds=1):
    return [{"direction": "stop", "amount": seconds},]

def forward(distance=0.1, speed=1.0):
    return [{"direction": "forward", "amount": distance, "speed": speed}]

def backward(distance=0.1, speed=1.0):
    return [{"direction": "back", "amount": distance, "speed": speed}]

def rotate_left(degrees=10, speed=0.5):
    return [{"direction": "rotate_left", "amount": degrees, "speed": speed}]

def rotate_right(degrees=10, speed=0.5):
    return [{"direction": "rotate_right", "amount": degrees, "speed": speed}]

def move_to_point(position):
    """Navigate to an exact map coordinate and stop."""
    if not isinstance(position, Position):
        raise TypeError("position must be a Position object")
    return [{"direction": "navigate", "amount": [position.x, position.y, position.yaw]}] + stop()

def shove(distance=0.4, speed=0.05, duration=30.0):
    return [
        {
            "direction": "creep",
            "amount": distance,
            "speed": speed,
            "duration": duration
        }
    ]

def follow_me(position, rotation_speed=1.0, lookback_interval=20.0, detect_range=0.5, idle_timeout=15.0):
    destination_steps = move_to_point(position)
    return [{
            "direction": "lead",
            "amount": destination_steps[0]["amount"],
            "speed": rotation_speed,
            "look_interval": lookback_interval,
            "detect_range": detect_range,
            "wait_timeout": idle_timeout,
            "rotation_steps": rotate_left(150, 1.0),
            "destination_steps": destination_steps,
        }]


"""
Interaction Initiation Routines
"""
def low():
    return forward(0.1, 0.5) + stop()


def medium():
    return low() + rotate_left() + rotate_right(20) + rotate_left(20) + rotate_right(20) + rotate_left() + stop()


def high():
    return forward(0.2) + rotate_left(20,1) + rotate_right(40, 1) + rotate_left(40, 1) + rotate_right(40, 1) + rotate_left(15, 1) + stop()

def test():
    return move_to_point(blocking_human_path)
