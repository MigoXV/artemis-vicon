import unittest
from pathlib import Path

from artemis_vicon.schemas import MotorMode, Observation
from artemis_vicon.vehicle import MissionStateMachine, load_task_actions


class MissionStateMachineTest(unittest.TestCase):
    def test_task0_distance_completion_advances_to_turn(self) -> None:
        controller = self._controller("0")
        first = controller.step(self._obs(distance_cm=0.0))
        self.assertEqual(first.rear_left_mode, MotorMode.DEG)
        controller.step(self._obs(sequence_id=2, sim_time_s=1.0, distance_cm=98.0))
        turn = controller.step(self._obs(sequence_id=3, sim_time_s=1.02, distance_cm=98.0))
        self.assertEqual(turn.velocity, 0.0)
        self.assertEqual(turn.rear_left_mode, MotorMode.DEG)

    def test_default_route_tasks_use_full_default_route(self) -> None:
        expected = load_task_actions(Path("examples/m0/task0.json"))
        for task_id in (0, 1, 2, 4):
            with self.subTest(task_id=task_id):
                actions = load_task_actions(Path(f"examples/m0/task{task_id}.json"))
                self.assertEqual(actions, expected)

    def test_task3_starts_on_diagonal_without_right_angle_turn(self) -> None:
        controller = self._controller("3")
        self.assertEqual(controller.actions[0].kind, "drive_until_line")
        self.assertEqual(controller.actions[1].kind, "track_until_lost")
        command = controller.step(self._obs())
        self.assertEqual(command.velocity, 7.0)
        self.assertLess(abs(command.turn), 3.0)

    @staticmethod
    def _controller(task_id: str) -> MissionStateMachine:
        return MissionStateMachine(load_task_actions(Path(f"examples/m0/task{task_id}.json")))

    @staticmethod
    def _obs(
        *,
        sequence_id: int = 1,
        sim_time_s: float = 0.0,
        yaw_deg: float = 0.0,
        digital: tuple[int, ...] = (0, 0, 0, 0, 0, 0, 0, 0),
        distance_cm: float = 0.0,
    ) -> Observation:
        return Observation(
            sequence_id=sequence_id,
            sim_time_s=sim_time_s,
            yaw_deg=yaw_deg,
            digital_values=digital,
            forward_distance_cm=distance_cm,
        )


if __name__ == "__main__":
    unittest.main()
