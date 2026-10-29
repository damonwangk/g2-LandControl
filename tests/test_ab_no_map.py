import contextlib
import io
import json
import unittest
from unittest.mock import patch

import ab_no_map as ab


class FakeController:
    def __init__(self, cancel_at=None):
        self.events = []
        self.cancel_at = cancel_at

    def run_velocity(self, velocity, seconds):
        if len(self.events) == self.cancel_at:
            raise InterruptedError('cancelled')
        self.events.append((velocity.vx, velocity.vy, velocity.wz, seconds))

    def stop(self):
        self.events.append('zero')


class NoMapTests(unittest.TestCase):
    def test_reject_incomplete_and_unsafe(self):
        for value in ({}, {'segments': []}, {'segments': [{'vx': .1}]},
                      {'segments': [{'vx': 0, 'vy': .05, 'wz': .1, 'seconds': 1}]}):
            with self.assertRaises(ValueError):
                ab.parse_config(value)

    def test_relative_nominal_and_sequence(self):
        s = ab.relative_segment(.3, .4, .1)
        self.assertAlmostEqual(s[0][1], 5)
        self.assertAlmostEqual(s[0][0].vx, .06)
        self.assertAlmostEqual(s[0][0].vy, .08)
        segments = ab.parse_config({'segments': [
            {'vx': .05, 'vy': 0, 'wz': 0, 'seconds': 2},
            {'vx': 0, 'vy': 0, 'wz': .1, 'seconds': 1}]})
        c = FakeController()
        ab.run_segments(c, segments)
        self.assertEqual(c.events, [(.05, 0, 0, 2), (0, 0, .1, 1), 'zero'])

    def test_cancel_stops(self):
        c = FakeController(cancel_at=1)
        s = ab.relative_segment(.1, 0, .1) * 2
        with self.assertRaises(InterruptedError):
            ab.run_segments(c, s)
        self.assertEqual(c.events[-1], 'zero')
        self.assertEqual(len(c.events), 2)

    def test_default_preview_never_constructs_controller(self):
        with patch.object(ab, 'ChassisController', side_effect=AssertionError('SDK path')), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(ab.main([]), 0)
            self.assertEqual(json.loads(out.getvalue())['mode'], 'template')
        with patch.object(ab, 'ChassisController', side_effect=AssertionError('SDK path')), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(ab.main(['--dx', '.1', '--dy', '0', '--speed', '.05']), 0)
            self.assertEqual(json.loads(out.getvalue())['mode'], 'preview')


if __name__ == '__main__':
    unittest.main()
