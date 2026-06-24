"""User-defined movement routines for Elly.

You can edit this file freely on your laptop to define new movement routines.
Each dictionary should have:
  - "direction": "forward" | "back" | "left" | "right" | "turn_left" | "turn_right" | "rotate_left" | "rotate_right" | "stop" | "hold" | "navigate"
  - "amount": distance in meters (for translation), angle in degrees (for rotation), seconds (for stop/hold), or [x, y, yaw_deg] (for navigate)
  - "speed": (optional) speed in m/s (default 0.1) or rad/s (default 0.3)



"""

from dataclasses import dataclass

@dataclass
class Position:
    x: float
    y: float
    yaw: float

robot_starting = Position(0.945119, -1.48997, -32.4422)
human_objective = Position(0.0866141, 3.21257, 87.1589)
behind_human_objective = Position(0.275744, 3.82844, -94.4796)
front_human_objective = Position(-0.0226278, 2.20267, 88.6927)
blocking_human_path = Position(-0.287587, 0.340654, -6.6172)

"""
Primitive Routines
"""
def stop(seconds=1):
    return [{"direction": "stop", "amount": seconds},]

def forward(distance=0.1, speed=1.0):
    return [{"direction": "forward", "amount": distance, "speed": speed}]

def backward(distance=0.1, speed=1.0):
    return [{"direction": "back", "amount": distance, "speed": speed}]

def left(distance=0.1, speed=1.0):
    return [{"direction": "left", "amount": distance, "speed": speed}]

def right(distance=0.1, speed=1.0):
    return [{"direction": "right", "amount": distance, "speed": speed}]

def rotate_left(degrees=10, speed=0.5):
    return [{"direction": "rotate_left", "amount": degrees, "speed": speed}]

def rotate_right(degrees=10, speed=0.5):
    return [{"direction": "rotate_right", "amount": degrees, "speed": speed}]

def turn_left(degrees=90, speed=0.3, radius=0.22, finish_tolerance=8.0):
    return [{
        "direction": "turn_left",
        "amount": degrees,
        "speed": speed,
        "radius": radius,
        "finish_tolerance": finish_tolerance,
    }]

def turn_right(degrees=90, speed=0.3, radius=0.22, finish_tolerance=8.0):
    return [{
        "direction": "turn_right",
        "amount": degrees,
        "speed": speed,
        "radius": radius,
        "finish_tolerance": finish_tolerance,
    }]

def move_to_point(position):
    """Navigate to an exact map coordinate and stop."""
    if not isinstance(position, Position):
        raise TypeError("position must be a Position object")
    return [{"direction": "navigate", "amount": [position.x, position.y, position.yaw]}] + stop()

def creep(distance=0.4, speed=0.05, duration=30.0):
    return [
        {
            "direction": "creep",
            "amount": distance,
            "speed": speed,
            "duration": duration
        }
    ]

def trace_circle(size=0.1, speed=0.3):
    """Trace a small circular path using the moving turn primitive."""
    return turn_left(330, speed, size) + stop(1)

def trace_eight(size=0.1, speed=0.3):
    """Trace two opposite circular lobes using moving turn primitives."""
    return turn_left(330, speed, size) + turn_right(330, speed, size) + stop(1)

def lead(position, rotation_speed=1.0, lookback_interval=30.0, detect_range=0.5, idle_timeout=15.0):
    destination_steps = move_to_point(position)
    return [{
            "direction": "lead",
            "amount": destination_steps[0]["amount"],
            "speed": rotation_speed,
            "look_interval": lookback_interval,
            "detect_range": detect_range,
            "wait_timeout": idle_timeout,
            "rotation_steps": rotate_left(150, 1.0) + stop(1),
            "destination_steps": destination_steps,
        }]


"""
Interaction Initiation Routines
"""
def low():
    return backward(0.1, 0.5) + stop()

def medium():
    return forward(0.1, 0.5) + rotate_left() + rotate_right(20) + rotate_left(20) + rotate_right(20) + rotate_left() + stop()

def high():
    return forward(0.2) + rotate_left(20,1) + rotate_right(40, 1) + rotate_left(40, 1) + rotate_right(40, 1) + rotate_left(10, 1) + stop()

"""
Movement Routines
"""

def look_at_this(trace_size=0.2, speed=0.3):
    return low() + move_to_point(human_objective) + backward(trace_size, speed) + rotate_right(90) + trace_circle(trace_size, speed) + trace_circle(trace_size, speed) + rotate_left(90) + backward(trace_size, speed) + stop()

def follow_me():
    return move_to_point(robot_starting) + lead(behind_human_objective) + stop()

def stop_person():
    return move_to_point(blocking_human_path) + stop()

def shove():
    return move_to_point(behind_human_objective) + creep() + stop()

# Could do refusal, where robot actively avoids looking at the person, if the person gets in front of him, he just turns away, and runs.
def test():
    return move_to_point(behind_human_objective) + stop()