#!/usr/bin/env python3
import math
import unittest
import sys
import os

# Add the scripts directory to the path so we can import brain
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))
from brain import normalize_angle, unwrap_angle


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


if __name__ == "__main__":
    unittest.main()
