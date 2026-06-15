#!/usr/bin/env python3
import math
import unittest
import sys
import os
from unittest.mock import call, patch

# Add the scripts directory to the path so we can import brain
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))
import brain
from brain import (
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
        brain_node = type("FakeBrainNode", (), {})()
        brain_node.get_front_clearance = lambda cone: 0.45

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
        node = type(
            "FakeNode",
            (),
            {
                "active_movement": {"type": "follow_me"},
                "last_turn": None,
                "status_message": "",
                "stop_requested": False,
            },
        )()
        runtime = BrainFollowRuntime.__new__(BrainFollowRuntime)
        runtime.node = node
        runtime.current_phase = "turning_for_check"
        runtime.config = type("Config", (), {"detect_range": 0.5})()
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


class MovementCompositionTests(unittest.TestCase):
    def test_follow_me_reuses_only_rotation_and_navigation_recipes(self):
        step = movements.follow_me(
            x=1.0,
            y=2.0,
            yaw_deg=90.0,
            detect_range=0.5,
            wait_timeout=12.0,
        )[0]

        self.assertEqual(step["rotation_steps"], movements.rotate_180())
        self.assertEqual(
            step["destination_steps"],
            movements.move_to_point(1.0, 2.0, 90.0),
        )
        self.assertNotIn("detection_step", step)
        self.assertEqual(step["detect_range"], 0.5)
        self.assertEqual(step["wait_timeout"], 12.0)

    def test_old_destination_name_remains_compatible(self):
        self.assertEqual(
            movements.move_to_behind_human_point(),
            movements.move_to_point(),
        )
        self.assertEqual(movements.move_behind_human(), movements.move_to_point())


if __name__ == "__main__":
    unittest.main()
