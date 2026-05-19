from __future__ import annotations
"""小车动作状态机控制器。"""

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field

from artemis_vicon.schemas import ControlCommand, Observation, TaskAction
from artemis_vicon.vehicle.line_tracking import LineTrackController
from artemis_vicon.vehicle.motion import MotionController
from artemis_vicon.vehicle.yaw import YawHoldController


IntArray = NDArray[np.int_]


class TaskActionBlockConfig(BaseModel):
    """任务 JSON 中可重复的动作块。"""

    model_config = ConfigDict(extra="forbid")

    repeat: int = Field(default=1, ge=1)
    actions: list[TaskAction] = Field(min_length=1)

    def expand(self) -> tuple[TaskAction, ...]:
        return tuple(self.actions) * self.repeat


class MissionConfig(BaseModel):
    """任务 JSON 根配置。"""

    model_config = ConfigDict(extra="forbid")

    actions: list[TaskAction] = Field(default_factory=list)
    blocks: list[TaskActionBlockConfig] = Field(default_factory=list)

    def to_actions(self) -> tuple[TaskAction, ...]:
        actions: list[TaskAction] = []
        for block in self.blocks:
            actions.extend(block.expand())
        actions.extend(self.actions)
        return tuple(actions)


def load_task_actions(path: Path) -> tuple[TaskAction, ...]:
    """从 JSON 文件加载任务动作。"""

    config = MissionConfig.model_validate_json(path.read_text(encoding="utf-8"))
    actions = config.to_actions()
    if not actions:
        raise ValueError(f"Task action file is empty: {path}")
    return actions


class MissionStateMachine:
    """根据任务动作序列输出后轮目标速度。"""

    line_detect_confirm_frames = 2
    line_loss_confirm_frames = 50

    def __init__(self, actions: tuple[TaskAction, ...]) -> None:
        self.actions = actions
        self.line_tracker = LineTrackController()
        self.yaw = YawHoldController()
        self.motion = MotionController()
        self.base_yaw_deg: float | None = None
        self.action_index = 0
        self.action_started_at: float | None = None
        self.distance_started_at_cm: float | None = None
        self.confirm_count = 0
        self.line_seen = False

    def reset(self) -> None:
        """重置任务状态。"""

        self.line_tracker.reset()
        self.yaw.reset()
        self.base_yaw_deg = None
        self.action_index = 0
        self.action_started_at = None
        self.distance_started_at_cm = None
        self.confirm_count = 0
        self.line_seen = False

    def step(self, observation: Observation) -> ControlCommand:
        """根据一帧观测推进任务状态机。"""

        if self.base_yaw_deg is None:
            self.base_yaw_deg = observation.yaw_deg
        if self.action_index >= len(self.actions):
            return self.motion.stop_command(observation.sequence_id, completed=True)

        while True:
            action = self.actions[self.action_index]
            if action.kind == "finish":
                return self.motion.stop_command(observation.sequence_id, completed=True)
            if self.action_started_at is None:
                self._enter_action(observation)
            command = self._command_for_action(action, observation)
            if not self._action_completed(action, observation):
                return command
            self._advance_action()
            if self.action_index >= len(self.actions):
                return self.motion.stop_command(observation.sequence_id, completed=True)

    def _enter_action(self, observation: Observation) -> None:
        self.action_started_at = observation.sim_time_s
        self.distance_started_at_cm = observation.forward_distance_cm
        self.confirm_count = 0
        self.line_seen = False
        self.line_tracker.reset()
        self.yaw.reset_stability()

    def _advance_action(self) -> None:
        self.action_index += 1
        self.action_started_at = None
        self.distance_started_at_cm = None
        self.confirm_count = 0
        self.line_seen = False

    def _command_for_action(self, action: TaskAction, observation: Observation) -> ControlCommand:
        if action.kind == "track_until_lost":
            return self._track_command(observation.sequence_id, observation.digital_values, action.velocity)
        if action.kind == "finish":
            return self.motion.stop_command(observation.sequence_id, completed=True)

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

    def _action_completed(self, action: TaskAction, observation: Observation) -> bool:
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
            line_detected = self.line_tracker.scan(observation.digital_values)
            self.confirm_count = self.confirm_count + 1 if line_detected else 0
            return self.confirm_count >= self.line_detect_confirm_frames or (
                action.max_duration_s > 0.0 and elapsed >= action.max_duration_s
            )
        if action.kind == "track_until_lost":
            line_detected = self.line_tracker.scan(observation.digital_values)
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
        return (self.base_yaw_deg + offset_deg) % 360.0

    def _deg_command(
        self,
        *,
        sequence_id: int,
        current_yaw_deg: float,
        target_yaw_deg: float,
        velocity: float,
        target_scale: float = 1.0,
    ) -> ControlCommand:
        turn, stable = self.yaw.compute(current_yaw_deg, target_yaw_deg)
        return self.motion.deg_command(
            sequence_id=sequence_id,
            velocity=velocity,
            turn=turn,
            target_scale=target_scale,
            stable=stable,
        )

    def _track_command(self, sequence_id: int, digital_values: IntArray, velocity: float) -> ControlCommand:
        self.line_tracker.scan(digital_values)
        turn = self.line_tracker.compute_turn(velocity)
        return self.motion.track_command(sequence_id=sequence_id, velocity=velocity, turn=turn)
