from __future__ import annotations
"""固件控制器的数据结构。"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Literal


class MotorMode(IntEnum):
    """对齐 C 端 pid_mode_t。"""

    STOP = 0
    SPEED = 1
    DIR = 2
    TRACK = 3
    DEG = 4


@dataclass(frozen=True)
class FirmwareObservation:
    """固件控制器需要的一帧硬件观测。"""

    sequence_id: int
    sim_time_s: float
    yaw_deg: float
    digital_values: tuple[int, ...]
    forward_distance_cm: float


@dataclass(frozen=True)
class FirmwareCommand:
    """固件控制器输出的速度目标命令。"""

    sequence_id: int
    velocity: float
    turn: float
    rear_left_target_speed: float
    rear_right_target_speed: float
    rear_left_mode: MotorMode
    rear_right_mode: MotorMode
    completed: bool = False


ActionKind = Literal["drive_distance", "drive_until_line", "track_until_lost", "drive_for", "turn_settle", "finish"]


@dataclass(frozen=True)
class FirmwareAction:
    """任务状态机中的一个动作。"""

    kind: ActionKind
    yaw_offset_deg: float = 0.0
    velocity: float = 7.0
    duration_s: float = 0.0
    distance_cm: float = 0.0
    max_duration_s: float = 0.0
