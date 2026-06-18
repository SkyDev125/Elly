#!/usr/bin/env python3
"""Local runner to parse Python movement definitions and stream them to Elly."""

import json
import os
import subprocess
import sys
import importlib.util

# Add scripts directory to path to import directions
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brain import LINEAR_DIRECTIONS, TURN_DIRECTIONS, ANGULAR_DIRECTIONS, HOLD_DIRECTIONS


def load_movements():
    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    movements_path = os.path.join(repo_dir, "sequences", "movements.py")
    if not os.path.exists(movements_path):
        print(f"[x] Error: movements.py not found at {movements_path}", file=sys.stderr)
        sys.exit(1)
        
    spec = importlib.util.spec_from_file_location("movements", movements_path)
    movements = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(movements)
    return movements


def main():
    if len(sys.argv) < 2:
        print("Usage:", file=sys.stderr)
        print("  elly_move <routine_name> [args...]           - Run a Python routine from sequences/movements.py", file=sys.stderr)
        print("  elly_move <direction> <amount> [speed]       - Run a single movement step", file=sys.stderr)
        print("  elly_move sequence <json_string_or_filepath> - Run a JSON sequence directly", file=sys.stderr)
        sys.exit(1)
        
    routine_name = sys.argv[1]
    func_args = sys.argv[2:]
    
    # Cast args to floats/ints if possible
    typed_args = []
    for arg in func_args:
        try:
            if '.' in arg:
                typed_args.append(float(arg))
            else:
                typed_args.append(int(arg))
        except ValueError:
            typed_args.append(arg)
            
    steps = None
    
    if routine_name in LINEAR_DIRECTIONS or routine_name in TURN_DIRECTIONS or routine_name in ANGULAR_DIRECTIONS:
        if len(typed_args) < 1:
            print(f"[x] Error: Usage: elly_move {routine_name} <amount> [speed]", file=sys.stderr)
            sys.exit(1)
        amount = typed_args[0]
        speed = typed_args[1] if len(typed_args) > 1 else None
        radius = typed_args[2] if routine_name in TURN_DIRECTIONS and len(typed_args) > 2 else None
        finish_tolerance = typed_args[3] if routine_name in TURN_DIRECTIONS and len(typed_args) > 3 else None
        step = {"direction": routine_name, "amount": amount}
        if speed is not None:
            step["speed"] = speed
        if radius is not None:
            step["radius"] = radius
        if finish_tolerance is not None:
            step["finish_tolerance"] = finish_tolerance
        steps = [step]
        
    elif routine_name in HOLD_DIRECTIONS:
        if len(typed_args) < 1:
            print(f"[x] Error: Usage: elly_move {routine_name} <duration>", file=sys.stderr)
            sys.exit(1)
        duration = typed_args[0]
        steps = [{"direction": routine_name, "amount": duration}]
        
    # 2. Check if the command is direct JSON sequence streaming
    elif routine_name == "sequence":
        if len(func_args) < 1:
            print("[x] Error: Usage: elly_move sequence <json_string_or_filepath>", file=sys.stderr)
            sys.exit(1)
        data = func_args[0].strip()
        try:
            if os.path.isfile(data):
                with open(data, 'r') as f:
                    steps = json.load(f)
            else:
                steps = json.loads(data)
        except Exception as e:
            print(f"[x] Error: Invalid JSON input or file path: {e}", file=sys.stderr)
            sys.exit(1)
            
    # 3. Otherwise, load from sequences/movements.py
    else:
        movements = load_movements()
        if not hasattr(movements, routine_name):
            print(f"[x] Error: Routine '{routine_name}' not defined in sequences/movements.py", file=sys.stderr)
            sys.exit(1)
            
        func = getattr(movements, routine_name)
        if not callable(func):
            print(f"[x] Error: '{routine_name}' is not callable in sequences/movements.py", file=sys.stderr)
            sys.exit(1)
            
        try:
            steps = func(*typed_args)
        except Exception as e:
            print(f"[x] Error executing '{routine_name}': {e}", file=sys.stderr)
            sys.exit(1)
            
    if not isinstance(steps, list):
        print(f"[x] Error: Routine must return a list of step dictionaries", file=sys.stderr)
        sys.exit(1)
        
    # Validate steps format
    for i, step in enumerate(steps):
        if not isinstance(step, dict) or "direction" not in step or "amount" not in step:
            print(f"[x] Error: Step {i+1} is invalid. Must be a dict with 'direction' and 'amount'", file=sys.stderr)
            sys.exit(1)
            
    json_str = json.dumps(steps)
    
    nano_ip = os.environ.get("NANO_IP")
    nano_user = os.environ.get("NANO_USER", "er")
    
    if not nano_ip:
        print("[x] Error: NANO_IP environment variable not set. Run source ~/.bashrc", file=sys.stderr)
        sys.exit(1)
        
    remote = f"{nano_user}@{nano_ip}"
    cmd = ["ssh", remote, f"python3 ~/scripts/brain.py sequence '{json_str}'"]
    
    res = subprocess.run(cmd)
    sys.exit(res.returncode)


if __name__ == "__main__":
    main()
