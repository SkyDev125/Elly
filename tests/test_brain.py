#!/usr/bin/env python3
import inspect
import math
import unittest
from types import SimpleNamespace
from typing import Any
from unittest.mock import call, patch

from scripts import brain
from scripts import elly as elly_cli
from scripts.brain import (
    BrainFollowRuntime,
    FollowEvent,
    FollowMeConfig,
    FollowMeRunner,
    FollowState,
    FollowStateMachine,
    normalize_angle,
    unwrap_angle,
)
from sequences import movements


class BrainTests(unittest.TestCase):
    def test_dashboard_parses_screen_session_names(self):
        output = """
            101.base (Detached)
            202.motion_service (Detached)
            303.slam_2d (Detached)
        """
        self.assertEqual(
            elly_cli.parse_screen_sessions(output),
            {"base", "motion_service", "slam_2d"},
        )

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(math.radians(370)), math.radians(10))
        self.assertAlmostEqual(normalize_angle(math.radians(-370)), math.radians(-10))
        self.assertAlmostEqual(normalize_angle(math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-math.pi), -math.pi)

    def test_unwrap_angle(self):
        previous_wrapped = math.radians(179)
        previous_unwrapped = previous_wrapped
        current_wrapped = math.radians(-179)
        current_unwrapped = unwrap_angle(previous_wrapped, current_wrapped, previous_unwrapped)
        self.assertAlmostEqual(current_unwrapped, math.radians(181))

        # Rotate back
        self.assertAlmostEqual(unwrap_angle(current_wrapped, previous_wrapped, current_unwrapped), previous_wrapped)

    def test_front_proximity_is_a_neutral_shared_lidar_check(self):
        brain_node: Any = SimpleNamespace(
            get_front_clearance=lambda cone: 0.45,
        )

        clearance, detected = brain.BrainNode.check_front_proximity(
            brain_node,
            0.5,
            30.0,
        )

        self.assertEqual(clearance, 0.45)
        self.assertTrue(detected)

    @patch.object(brain, "execute_single_step")
    def test_follow_me_rotate_180_uses_proven_sequence(self, execute_step):
        execute_step.return_value = {"ok": True}
        node: Any = SimpleNamespace(
            active_movement={"type": "follow_me"},
            last_turn=None,
            status_message="",
            stop_requested=False,
        )
        runtime: Any = BrainFollowRuntime.__new__(BrainFollowRuntime)
        runtime.node = node
        runtime.current_phase = "turning_for_check"
        runtime.config = SimpleNamespace(detect_range=0.5)
        runtime.rotation_steps = [
            {"direction": "rotate_left", "amount": 180.0, "speed": 1.0},
            {"direction": "stop", "amount": 1.0},
        ]

        self.assertTrue(runtime.rotate_180())
        self.assertEqual(
            execute_step.call_args_list,
            [
                call(node, "rotate_left", 180.0, speed=1.0),
                call(node, "stop", 1.0),
            ],
        )

    def test_turn_left_sets_arc_motion_with_radius(self):
        class Done:
            def clear(self):
                pass

            def wait(self):
                node.success = True

        node: Any = SimpleNamespace(
            movement_done=Done(),
            stop_requested=False,
            error_message="",
            success=False,
            current_pose=(0.0, 0.0, 0.0),
            latest_scan=None,
            last_scan_time=0.0,
        )
        node.get_current_pose_best = lambda: (node.current_pose, "odom_sub")

        result = brain.execute_single_step(
            node,
            "turn_left",
            360,
            speed=0.3,
            radius=0.2,
        )

        self.assertTrue(result["ok"])
        self.assertEqual(node.active_movement["type"], "turn")
        self.assertEqual(node.active_movement["sign"], 1.0)
        self.assertEqual(node.active_movement["speed"], 0.3)
        self.assertEqual(node.active_movement["radius"], 0.2)
        self.assertAlmostEqual(node.active_movement["angular_speed"], 1.5)
        self.assertEqual(node.active_movement["target_angle_rad"], math.radians(360))
        self.assertEqual(node.active_movement["turn_progress_rad"], 0.0)
        self.assertEqual(node.active_movement["feedback_frame"], "odom_sub")
        self.assertAlmostEqual(
            node.active_movement["finish_tolerance_rad"],
            math.radians(8.0),
        )

    def test_turn_arc_keeps_constant_speed_until_stop(self):
        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0)
                self.angular = SimpleNamespace(z=0.0)

        node: Any = brain.BrainNode.__new__(brain.BrainNode)
        node.active_movement = {
            "type": "turn",
            "sign": 1.0,
            "target_angle_rad": math.radians(720),
            "turn_progress_rad": math.radians(700),
            "finish_tolerance_rad": math.radians(8),
            "speed": 0.3,
            "angular_speed": 3.0,
            "radius": 0.1,
            "deadline": 9999999999,
        }
        node.stop_requested = False
        node.latest_scan = None
        node.current_pose = None
        published = []
        node.cmd_pub = SimpleNamespace(publish=published.append)

        with patch.dict("sys.modules", {"geometry_msgs.msg": type("Msg", (), {"Twist": Twist})}):
            brain.BrainNode.control_loop_cycle(node)

        self.assertAlmostEqual(published[-1].linear.x, 0.3)
        self.assertAlmostEqual(published[-1].angular.z, 3.0)


class FakeRuntime:
    def __init__(
        self,
        person_results,
        navigation_events=None,
        cancel_ok=True,
        rotation_results=None,
    ):
        self.person_results = list(person_results)
        self.navigation_events = list(navigation_events or [])
        self.cancel_ok = cancel_ok
        self.rotation_results = list(rotation_results or [])
        self.states = []
        self.navigation_starts = 0
        self.cancellations = 0
        self.rotations = []
        self.person_checks = []
        self.holds = []
        self.destination_finishes = 0
        self.stopped = False

    def set_state(self, phase, message):
        self.states.append((phase, message))

    def stop_requested(self):
        return self.stopped

    def hold_stopped(self, duration):
        self.holds.append(duration)
        return not self.stopped

    def wait_for_person(self, sector, timeout, require_foreground):
        self.person_checks.append((sector, timeout, require_foreground))
        return self.person_results.pop(0)

    def start_navigation(self, goal):
        self.navigation_starts += 1
        return {"goal": goal}

    def wait_for_navigation(self, navigation, look_interval):
        event = self.navigation_events.pop(0)
        return event, event.value

    def cancel_navigation(self, navigation):
        self.cancellations += 1
        return self.cancel_ok

    def rotate_180(self):
        self.rotations.append(180.0)
        if self.rotation_results:
            return self.rotation_results.pop(0)
        return True

    def complete_destination(self):
        self.destination_finishes += 1
        return True


class FollowStateMachineTests(unittest.TestCase):
    def test_periodic_check_then_goal_success(self):
        machine = FollowStateMachine()
        self.assertEqual(machine.transition(FollowEvent.HUMAN_FOUND), FollowState.NAVIGATING)
        self.assertEqual(machine.transition(FollowEvent.LOOK_DUE), FollowState.TURNING_TO_HUMAN)
        self.assertEqual(
            machine.transition(FollowEvent.ROTATION_COMPLETE),
            FollowState.WAITING_LOOKBACK,
        )
        self.assertEqual(machine.transition(FollowEvent.HUMAN_FOUND), FollowState.TURNING_TO_PATH)
        self.assertEqual(
            machine.transition(FollowEvent.ROTATION_COMPLETE),
            FollowState.NAVIGATING,
        )
        self.assertEqual(machine.transition(FollowEvent.GOAL_REACHED), FollowState.TURNING_AT_GOAL)
        self.assertEqual(
            machine.transition(FollowEvent.ROTATION_COMPLETE),
            FollowState.WAITING_AT_GOAL,
        )
        self.assertEqual(machine.transition(FollowEvent.HUMAN_FOUND), FollowState.COMPLETED)

    def test_invalid_transition_is_rejected(self):
        machine = FollowStateMachine()
        with self.assertRaises(ValueError):
            machine.transition(FollowEvent.GOAL_REACHED)


class FollowMeRunnerTests(unittest.TestCase):
    def setUp(self):
        self.config = FollowMeConfig(
            look_interval=5.0,
            detect_range=1.0,
            wait_timeout=4.0,
        )
        self.goal = (1.0, 2.0, 90.0)

    def test_full_periodic_check_and_goal_flow(self):
        runtime = FakeRuntime(
            person_results=[True, True, True],
            navigation_events=[FollowEvent.LOOK_DUE, FollowEvent.GOAL_REACHED],
        )
        result = FollowMeRunner(runtime, self.goal, self.config).run()
        self.assertTrue(result.ok)
        self.assertEqual(result.reason, "completed")
        self.assertEqual(runtime.navigation_starts, 2)
        self.assertEqual(runtime.cancellations, 1)
        self.assertEqual(runtime.rotations, [180.0, 180.0, 180.0])
        self.assertEqual(runtime.holds, [0.75])
        self.assertEqual(runtime.destination_finishes, 1)
        self.assertEqual(runtime.person_checks[0], ("front", 4.0, False))
        self.assertEqual(runtime.states[-1][0], FollowState.COMPLETED.value)

    def test_person_lost_ends_without_turning_back(self):
        runtime = FakeRuntime(
            person_results=[True, False],
            navigation_events=[FollowEvent.LOOK_DUE],
        )
        result = FollowMeRunner(runtime, self.goal, self.config).run()
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "human_lost")
        self.assertEqual(runtime.rotations, [180.0])

    def test_initial_wait_timeout_never_starts_navigation(self):
        runtime = FakeRuntime(person_results=[False])
        result = FollowMeRunner(runtime, self.goal, self.config).run()
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "initial_human_timeout")
        self.assertEqual(runtime.navigation_starts, 0)

    def test_cancel_failure_stops_before_rotation(self):
        runtime = FakeRuntime(
            person_results=[True],
            navigation_events=[FollowEvent.LOOK_DUE],
            cancel_ok=False,
        )
        result = FollowMeRunner(runtime, self.goal, self.config).run()
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "navigation_cancel_failed")
        self.assertEqual(runtime.rotations, [])

    def test_rotation_timeout_is_reported(self):
        runtime = FakeRuntime(
            person_results=[True],
            navigation_events=[FollowEvent.GOAL_REACHED],
            rotation_results=[False],
        )
        result = FollowMeRunner(runtime, self.goal, self.config).run()
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "rotation_failed")


class MovementRecipeTests(unittest.TestCase):
    supported_directions = (
        set(brain.LINEAR_DIRECTIONS)
        | set(brain.TURN_DIRECTIONS)
        | set(brain.ANGULAR_DIRECTIONS)
        | set(brain.HOLD_DIRECTIONS)
        | {"creep", "navigate", "lead"}
    )

    def assert_valid_steps(self, steps, routine_name):
        self.assertIsInstance(steps, list, routine_name)
        self.assertTrue(steps, routine_name)

        for index, step in enumerate(steps, start=1):
            label = f"{routine_name} step {index}"
            self.assertIsInstance(step, dict, label)
            self.assertIn("direction", step, label)
            self.assertIn("amount", step, label)
            self.assertIn(step["direction"], self.supported_directions, label)

            amount = step["amount"]
            if step["direction"] in {"navigate", "lead"}:
                self.assertIsInstance(amount, list, label)
                self.assertIn(len(amount), (2, 3), label)
                self.assertTrue(all(isinstance(value, (int, float)) for value in amount), label)
            else:
                self.assertIsInstance(amount, (int, float), label)
                self.assertGreater(amount, 0, label)

            for field in ("speed", "duration", "radius", "finish_tolerance"):
                if field in step:
                    self.assertIsInstance(step[field], (int, float), label)
                    self.assertGreater(step[field], 0, label)

            for field in ("rotation_steps", "destination_steps"):
                if field in step:
                    self.assert_valid_steps(step[field], f"{label}.{field}")

    def test_all_public_movement_routines_return_valid_steps(self):
        for name, routine in inspect.getmembers(movements, inspect.isfunction):
            if routine.__module__ != movements.__name__ or name.startswith("_"):
                continue

            args = []
            for parameter in inspect.signature(routine).parameters.values():
                if parameter.default is inspect.Parameter.empty:
                    self.assertEqual(parameter.name, "position", name)
                    args.append(movements.robot_starting)

            with self.subTest(routine=name):
                self.assert_valid_steps(routine(*args), name)

    def test_follow_me_contains_a_lead_step(self):
        steps = movements.follow_me()
        self.assertEqual(
            sum(step["direction"] == "lead" for step in steps),
            1,
        )


if __name__ == "__main__":
    unittest.main()
