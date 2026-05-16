from __future__ import annotations
"""artemis-m0 任务状态机控制器。"""

import math

from artemis_vicon.firmware.math_utils import normalize_360
from artemis_vicon.firmware.track import TrackController
from artemis_vicon.firmware.types import FirmwareAction, FirmwareCommand, FirmwareObservation, MotorMode
from artemis_vicon.firmware.yaw import YawController


class ArtemisM0FirmwareController:
    """按 artemis-m0 任务语义输出后轮目标速度。"""

    line_detect_confirm_frames = 2
    line_loss_confirm_frames = 50

    def __init__(self, task_id: str) -> None:
        self.task_id = task_id.lower().replace("task", "")
        self.track = TrackController()
        self.yaw = YawController()
        self.base_yaw_deg: float | None = None
        self.actions = self._build_actions(self.task_id)
        self.action_index = 0
        self.action_started_at: float | None = None
        self.distance_started_at_cm: float | None = None
        self.confirm_count = 0
        self.line_seen = False

    def reset(self) -> None:
        """重置任务状态。"""

        self.track.reset()
        self.yaw.reset()
        self.base_yaw_deg = None
        self.action_index = 0
        self.action_started_at = None
        self.distance_started_at_cm = None
        self.confirm_count = 0
        self.line_seen = False

    def step(self, observation: FirmwareObservation) -> FirmwareCommand:
        """根据一帧硬件观测推进任务状态机。"""

        if self.base_yaw_deg is None:
            self.base_yaw_deg = observation.yaw_deg
        if self.action_index >= len(self.actions):
            return self._stop_command(observation.sequence_id, completed=True)

        while True:
            action = self.actions[self.action_index]
            if action.kind == "finish":
                return self._stop_command(observation.sequence_id, completed=True)
            if self.action_started_at is None:
                self._enter_action(observation)
            command = self._command_for_action(action, observation)
            if not self._action_completed(action, observation):
                return command
            self._advance_action()
            if self.action_index >= len(self.actions):
                return self._stop_command(observation.sequence_id, completed=True)

    def _enter_action(self, observation: FirmwareObservation) -> None:
        self.action_started_at = observation.sim_time_s
        self.distance_started_at_cm = observation.forward_distance_cm
        self.confirm_count = 0
        self.line_seen = False
        self.track.reset()
        self.yaw.stable_counter = 0

    def _advance_action(self) -> None:
        self.action_index += 1
        self.action_started_at = None
        self.distance_started_at_cm = None
        self.confirm_count = 0
        self.line_seen = False

    def _command_for_action(self, action: FirmwareAction, observation: FirmwareObservation) -> FirmwareCommand:
        if action.kind == "track_until_lost":
            return self._track_command(observation.sequence_id, observation.digital_values, action.velocity)
        if action.kind == "finish":
            return self._stop_command(observation.sequence_id, completed=True)
        velocity = action.velocity if action.kind in {"drive_distance", "drive_until_line", "drive_for"} else 0.0
        distance_scale = 1.0
        if action.kind == "drive_distance" and self.distance_started_at_cm is not None:
            walked = observation.forward_distance_cm - self.distance_started_at_cm
            remaining = action.distance_cm - walked
            if 0.0 < remaining < 5.0:
                distance_scale = remaining / 5.0
        return self._deg_command(
            sequence_id=observation.sequence_id,
            current_yaw_deg=observation.yaw_deg,
            target_yaw_deg=self._target_yaw(action.yaw_offset_deg),
            velocity=velocity,
            target_scale=distance_scale,
        )

    def _action_completed(self, action: FirmwareAction, observation: FirmwareObservation) -> bool:
        started_at = observation.sim_time_s if self.action_started_at is None else self.action_started_at
        elapsed = observation.sim_time_s - started_at
        if action.kind == "drive_distance":
            distance_started_at = (
                observation.forward_distance_cm
                if self.distance_started_at_cm is None
                else self.distance_started_at_cm
            )
            walked = observation.forward_distance_cm - distance_started_at
            return walked >= action.distance_cm - 3.0
        if action.kind in {"drive_for", "turn_settle"}:
            return elapsed >= action.duration_s
        if action.kind == "drive_until_line":
            line_detected = self.track.scan(observation.digital_values)
            self.confirm_count = self.confirm_count + 1 if line_detected else 0
            return self.confirm_count >= self.line_detect_confirm_frames or (
                action.max_duration_s > 0.0 and elapsed >= action.max_duration_s
            )
        if action.kind == "track_until_lost":
            line_detected = self.track.scan(observation.digital_values)
            if line_detected:
                self.line_seen = True
                self.confirm_count = 0
            else:
                self.confirm_count += 1
            return (
                self.line_seen
                and self.confirm_count >= self.line_loss_confirm_frames
                or (action.max_duration_s > 0.0 and elapsed >= action.max_duration_s)
            )
        return False

    def _target_yaw(self, offset_deg: float) -> float:
        assert self.base_yaw_deg is not None
        return normalize_360(self.base_yaw_deg + offset_deg)

    def _deg_command(
        self,
        *,
        sequence_id: int,
        current_yaw_deg: float,
        target_yaw_deg: float,
        velocity: float,
        target_scale: float = 1.0,
    ) -> FirmwareCommand:
        turn, stable = self.yaw.compute(current_yaw_deg, target_yaw_deg)
        left = (velocity + turn) * target_scale
        right = (velocity - turn) * target_scale
        if stable and velocity == 0.0:
            left = 0.0
            right = 0.0
        return FirmwareCommand(
            sequence_id=sequence_id,
            velocity=velocity,
            turn=turn,
            rear_left_target_speed=left,
            rear_right_target_speed=right,
            rear_left_mode=MotorMode.DEG,
            rear_right_mode=MotorMode.DEG,
        )

    def _track_command(self, sequence_id: int, digital_values: tuple[int, ...], velocity: float) -> FirmwareCommand:
        self.track.scan(digital_values)
        turn = self.track.pid(velocity)
        return FirmwareCommand(
            sequence_id=sequence_id,
            velocity=velocity,
            turn=turn,
            rear_left_target_speed=velocity - turn,
            rear_right_target_speed=velocity + turn,
            rear_left_mode=MotorMode.TRACK,
            rear_right_mode=MotorMode.TRACK,
        )

    @staticmethod
    def _stop_command(sequence_id: int, *, completed: bool = False) -> FirmwareCommand:
        return FirmwareCommand(
            sequence_id=sequence_id,
            velocity=0.0,
            turn=0.0,
            rear_left_target_speed=0.0,
            rear_right_target_speed=0.0,
            rear_left_mode=MotorMode.STOP,
            rear_right_mode=MotorMode.STOP,
            completed=completed,
        )

    def _build_actions(self, task_id: str) -> tuple[FirmwareAction, ...]:
        if task_id == "0":
            return (
                FirmwareAction("drive_distance", yaw_offset_deg=0.0, distance_cm=100.0),
                FirmwareAction("turn_settle", yaw_offset_deg=-90.0, duration_s=1.2),
                FirmwareAction("drive_distance", yaw_offset_deg=-90.0, distance_cm=70.0),
                FirmwareAction("turn_settle", yaw_offset_deg=-180.0, duration_s=1.2),
                FirmwareAction("drive_distance", yaw_offset_deg=-180.0, distance_cm=100.0),
                FirmwareAction("turn_settle", yaw_offset_deg=-270.0, duration_s=1.2),
                FirmwareAction("drive_distance", yaw_offset_deg=-270.0, distance_cm=70.0),
                FirmwareAction("finish"),
            )
        if task_id == "1":
            return (
                FirmwareAction("drive_until_line", yaw_offset_deg=0.0, max_duration_s=4.0),
                FirmwareAction("finish"),
            )
        if task_id == "2":
            return (
                FirmwareAction("drive_until_line", yaw_offset_deg=0.0, max_duration_s=4.0),
                FirmwareAction("track_until_lost", max_duration_s=8.0),
                FirmwareAction("turn_settle", yaw_offset_deg=-178.0, duration_s=1.5),
                FirmwareAction("drive_until_line", yaw_offset_deg=-178.0, max_duration_s=4.0),
                FirmwareAction("track_until_lost", max_duration_s=8.0),
                FirmwareAction("finish"),
            )
        if task_id == "3":
            return self._task3_cycle() + (FirmwareAction("finish"),)
        if task_id == "4":
            actions: list[FirmwareAction] = []
            for _ in range(4):
                actions.extend(self._task4_cycle())
            actions.append(FirmwareAction("finish"))
            return tuple(actions)
        raise ValueError(f"Unsupported task id: {task_id!r}")

    def _task3_cycle(self) -> tuple[FirmwareAction, ...]:
        first_diagonal_heading_deg = math.degrees(math.atan2(-0.8, 1.0))
        second_diagonal_heading_deg = math.degrees(math.atan2(-0.8, -1.0))
        return (
            FirmwareAction("drive_until_line", yaw_offset_deg=first_diagonal_heading_deg, max_duration_s=5.0),
            FirmwareAction("track_until_lost", max_duration_s=8.0),
            FirmwareAction("turn_settle", yaw_offset_deg=second_diagonal_heading_deg, duration_s=1.2),
            FirmwareAction("drive_until_line", yaw_offset_deg=second_diagonal_heading_deg, max_duration_s=5.0),
            FirmwareAction("track_until_lost", max_duration_s=8.0),
        )

    def _task4_cycle(self) -> tuple[FirmwareAction, ...]:
        return (
            FirmwareAction("drive_for", yaw_offset_deg=0.0, duration_s=1.2),
            FirmwareAction("turn_settle", yaw_offset_deg=-90.0, duration_s=1.2),
            FirmwareAction("drive_for", yaw_offset_deg=-90.0, duration_s=1.5),
            FirmwareAction("turn_settle", yaw_offset_deg=0.0, duration_s=1.1),
            FirmwareAction("drive_until_line", yaw_offset_deg=0.0, max_duration_s=4.0),
            FirmwareAction("track_until_lost", max_duration_s=8.0),
            FirmwareAction("drive_for", yaw_offset_deg=0.0, duration_s=0.1),
            FirmwareAction("turn_settle", yaw_offset_deg=180.0, duration_s=0.6),
            FirmwareAction("drive_for", yaw_offset_deg=180.0, duration_s=1.2),
            FirmwareAction("turn_settle", yaw_offset_deg=270.0, duration_s=1.1),
            FirmwareAction("drive_for", yaw_offset_deg=270.0, duration_s=1.5),
            FirmwareAction("turn_settle", yaw_offset_deg=180.0, duration_s=1.1),
            FirmwareAction("drive_until_line", yaw_offset_deg=180.0, max_duration_s=4.0),
            FirmwareAction("track_until_lost", max_duration_s=8.0),
            FirmwareAction("drive_for", yaw_offset_deg=180.0, duration_s=0.2),
        )
