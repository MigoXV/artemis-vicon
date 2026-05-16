import unittest

from artemis_vicon.firmware import ArtemisM0FirmwareController, FirmwareObservation, MotorMode, TrackController, YawController


class TrackControllerTest(unittest.TestCase):
    def test_scan_uses_c_weight_average(self) -> None:
        controller = TrackController()
        self.assertTrue(controller.scan((1, 1, 0, 0, 0, 0, 0, 0)))
        self.assertAlmostEqual(controller.new_error, -3.5)

    def test_track_pid_turn_sign_matches_c_formula(self) -> None:
        controller = TrackController()
        controller.scan((0, 0, 0, 0, 0, 0, 1, 1))
        self.assertLess(controller.pid(7.0), 0.0)


class YawControllerTest(unittest.TestCase):
    def test_yaw_error_wraps_across_zero(self) -> None:
        controller = YawController()
        turn, _ = controller.compute(current_angle_deg=1.0, target_angle_deg=359.0)
        self.assertGreater(turn, 0.0)


class ArtemisM0FirmwareControllerTest(unittest.TestCase):
    def test_task0_distance_completion_advances_to_turn(self) -> None:
        controller = ArtemisM0FirmwareController("0")
        first = controller.step(self._obs(distance_cm=0.0))
        self.assertEqual(first.rear_left_mode, MotorMode.DEG)
        controller.step(self._obs(sequence_id=2, sim_time_s=1.0, distance_cm=98.0))
        turn = controller.step(self._obs(sequence_id=3, sim_time_s=1.02, distance_cm=98.0))
        self.assertEqual(turn.velocity, 0.0)
        self.assertEqual(turn.rear_left_mode, MotorMode.DEG)

    def test_task1_finishes_after_line_confirmation(self) -> None:
        controller = ArtemisM0FirmwareController("1")
        controller.step(self._obs(digital=(0, 0, 0, 0, 0, 0, 0, 0)))
        controller.step(self._obs(sequence_id=2, sim_time_s=0.02, digital=(0, 0, 0, 1, 1, 0, 0, 0)))
        command = controller.step(self._obs(sequence_id=3, sim_time_s=0.04, digital=(0, 0, 0, 1, 1, 0, 0, 0)))
        self.assertTrue(command.completed)

    def test_task3_starts_on_diagonal_without_right_angle_turn(self) -> None:
        controller = ArtemisM0FirmwareController("3")
        self.assertEqual(controller.actions[0].kind, "drive_until_line")
        self.assertEqual(controller.actions[1].kind, "track_until_lost")
        command = controller.step(self._obs())
        self.assertEqual(command.velocity, 7.0)
        self.assertLess(abs(command.turn), 3.0)

    @staticmethod
    def _obs(
        *,
        sequence_id: int = 1,
        sim_time_s: float = 0.0,
        yaw_deg: float = 0.0,
        digital: tuple[int, ...] = (0, 0, 0, 0, 0, 0, 0, 0),
        distance_cm: float = 0.0,
    ) -> FirmwareObservation:
        return FirmwareObservation(
            sequence_id=sequence_id,
            sim_time_s=sim_time_s,
            yaw_deg=yaw_deg,
            digital_values=digital,
            forward_distance_cm=distance_cm,
        )


if __name__ == "__main__":
    unittest.main()
