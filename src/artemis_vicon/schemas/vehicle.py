from __future__ import annotations
"""小车控制相关数据结构。"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict


IntArray = NDArray[np.int_]
FloatScalar = float | np.float32


class MotorMode(IntEnum):
    """对齐 C 端 pid_mode_t。"""

    STOP = 0
    SPEED = 1
    DIR = 2
    TRACK = 3
    DEG = 4


@dataclass(frozen=True)
class Observation:
    """控制器需要的一帧观测。"""

    sequence_id: int
    sim_time_s: FloatScalar
    yaw_deg: FloatScalar
    digital_values: IntArray
    forward_distance_cm: FloatScalar


@dataclass(frozen=True)
class ControlCommand:
    """控制器输出的速度目标命令。"""

    sequence_id: int
    velocity: FloatScalar
    turn: FloatScalar
    rear_left_target_speed: FloatScalar
    rear_right_target_speed: FloatScalar
    rear_left_mode: MotorMode
    rear_right_mode: MotorMode
    completed: bool = False


class TaskAction(BaseModel):
    """任务状态机中的一个动作。"""

    model_config = ConfigDict(extra="forbid")

    kind: Literal["drive_distance", "drive_until_line", "track_until_lost", "drive_for", "turn_settle", "finish"]
    yaw_offset_deg: float = 0.0
    velocity: float = 7.0
    duration_s: float = 0.0
    distance_cm: float = 0.0
    max_duration_s: float = 0.0
