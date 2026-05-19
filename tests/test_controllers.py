import unittest

import numpy as np

from artemis_vicon.controllers import LQRController, PIDController
from artemis_vicon.vehicle import LineTrackController, YawHoldController


class LineTrackControllerTest(unittest.TestCase):
    def test_scan_uses_c_weight_average(self) -> None:
        controller = LineTrackController()
        self.assertTrue(controller.scan((1, 1, 0, 0, 0, 0, 0, 0)))
        self.assertAlmostEqual(controller.new_error, -3.5)

    def test_track_pid_turn_sign_matches_c_formula(self) -> None:
        controller = LineTrackController()
        controller.scan((0, 0, 0, 0, 0, 0, 1, 1))
        self.assertLess(controller.compute_turn(7.0), 0.0)

    def test_track_pid_does_not_grow_on_constant_error(self) -> None:
        controller = LineTrackController()
        controller.scan((0, 0, 0, 0, 0, 0, 1, 1))
        first_turn = abs(controller.compute_turn(7.0))

        controller.scan((0, 0, 0, 0, 0, 0, 1, 1))
        second_turn = abs(controller.compute_turn(7.0))

        self.assertLessEqual(second_turn, first_turn)

    def test_scan_keeps_last_line_error_when_line_is_lost(self) -> None:
        controller = LineTrackController()
        controller.scan((0, 0, 0, 0, 0, 0, 1, 1))

        self.assertFalse(controller.scan((0, 0, 0, 0, 0, 0, 0, 0)))

        self.assertAlmostEqual(controller.new_error, 3.5)


class YawHoldControllerTest(unittest.TestCase):
    def test_yaw_error_wraps_across_zero(self) -> None:
        controller = YawHoldController()
        turn, _ = controller.compute(current_angle_deg=1.0, target_angle_deg=359.0)
        self.assertGreater(turn, 0.0)


class PIDControllerTest(unittest.TestCase):
    def test_stream_maintains_state(self) -> None:
        controller = PIDController(ki=1.0, kp=1.0, kd=1.0)
        setpoints = iter(
            (
                np.float32(1.0),
                np.float32(1.0),
            )
        )
        observations = iter((np.float32(0.0), np.float32(0.0)))

        results = list(controller.infer_stream(setpoints, observations))
        outputs = [float(output) for output, _ in results]
        state = results[-1][1]

        self.assertEqual(outputs, [3.0, 3.0])
        np.testing.assert_array_equal(state, np.array([2.0, 1.0, 1.0], dtype=np.float32))

    def test_stream_uses_setpoint_minus_observation(self) -> None:
        controller = PIDController(ki=0.0, kp=1.0, kd=0.0)

        outputs = [
            float(output)
            for output, _ in controller.infer_stream(
                iter((np.float32(3.0),)),
                iter((np.float32(1.0),)),
            )
        ]

        self.assertEqual(outputs, [2.0])

    def test_stream_stops_when_shorter_iterator_ends(self) -> None:
        controller = PIDController(ki=0.0, kp=1.0, kd=0.0)

        outputs = [
            float(output)
            for output, _ in controller.infer_stream(
                iter((np.float32(1.0), np.float32(2.0))),
                iter((np.float32(0.0),)),
            )
        ]

        self.assertEqual(outputs, [1.0])

    def test_output_limit_is_applied_inside_controller(self) -> None:
        controller = PIDController(ki=1.0, kp=1.0, kd=1.0, output_limit=2.0)
        outputs = [
            float(output)
            for output, _ in controller.infer_stream(
                iter((np.float32(1.0), np.float32(1.0))),
                iter((np.float32(0.0), np.float32(0.0))),
            )
        ]

        self.assertEqual(outputs, [2.0, 2.0])


class LQRControllerTest(unittest.TestCase):
    def test_stream_uses_gain_matrix_and_default_error(self) -> None:
        controller = LQRController(gain_matrix=np.array([[2.0, 0.5]], dtype=np.float32))

        results = list(
            controller.infer_stream(
                iter((np.array([3.0, 1.0], dtype=np.float32),)),
                iter((np.array([1.0, -1.0], dtype=np.float32),)),
            )
        )

        np.testing.assert_array_equal(results[0][0], np.array([5.0], dtype=np.float32))
        np.testing.assert_array_equal(results[0][1], np.array([], dtype=np.float32))

    def test_stream_uses_custom_error_fn(self) -> None:
        controller = LQRController(gain_matrix=np.eye(2, dtype=np.float32))

        outputs = [
            output
            for output, _ in controller.infer_stream(
                iter((np.array([1.0, 2.0], dtype=np.float32),)),
                iter((np.array([3.0, 5.0], dtype=np.float32),)),
                error_fn=lambda setpoint, observation: observation - setpoint,
            )
        ]

        np.testing.assert_array_equal(outputs[0], np.array([2.0, 3.0], dtype=np.float32))

    def test_output_limit_is_applied_inside_controller(self) -> None:
        controller = LQRController(
            gain_matrix=np.eye(2, dtype=np.float32),
            output_limit=np.array([1.0, 2.0], dtype=np.float32),
        )

        outputs = [
            output
            for output, _ in controller.infer_stream(
                iter((np.array([3.0, -4.0], dtype=np.float32),)),
                iter((np.array([0.0, 0.0], dtype=np.float32),)),
            )
        ]

        np.testing.assert_array_equal(outputs[0], np.array([1.0, -2.0], dtype=np.float32))

    def test_stream_stops_when_shorter_iterator_ends(self) -> None:
        controller = LQRController(gain_matrix=np.eye(2, dtype=np.float32))

        outputs = [
            output
            for output, _ in controller.infer_stream(
                iter(
                    (
                        np.array([1.0, 0.0], dtype=np.float32),
                        np.array([2.0, 0.0], dtype=np.float32),
                    )
                ),
                iter((np.array([0.0, 0.0], dtype=np.float32),)),
            )
        ]

        self.assertEqual(len(outputs), 1)
        np.testing.assert_array_equal(outputs[0], np.array([1.0, 0.0], dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
