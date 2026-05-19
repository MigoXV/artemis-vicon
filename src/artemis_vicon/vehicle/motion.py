from __future__ import annotations
"""左右后轮目标速度命令合成。"""

from artemis_vicon.schemas import ControlCommand, MotorMode


class MotionController:
    """将速度和转向量合成为后轮控制命令。"""

    def deg_command(
        self,
        *,
        sequence_id: int,
        velocity: float,
        turn: float,
        target_scale: float = 1.0,
        stable: bool = False,
    ) -> ControlCommand:
        left = (velocity + turn) * target_scale
        right = (velocity - turn) * target_scale
        if stable and velocity == 0.0:
            left = 0.0
            right = 0.0
        return ControlCommand(
            sequence_id=sequence_id,
            velocity=velocity,
            turn=turn,
            rear_left_target_speed=left,
            rear_right_target_speed=right,
            rear_left_mode=MotorMode.DEG,
            rear_right_mode=MotorMode.DEG,
        )

    def track_command(self, *, sequence_id: int, velocity: float, turn: float) -> ControlCommand:
        return ControlCommand(
            sequence_id=sequence_id,
            velocity=velocity,
            turn=turn,
            rear_left_target_speed=velocity - turn,
            rear_right_target_speed=velocity + turn,
            rear_left_mode=MotorMode.TRACK,
            rear_right_mode=MotorMode.TRACK,
        )

    def stop_command(self, sequence_id: int, *, completed: bool = False) -> ControlCommand:
        return ControlCommand(
            sequence_id=sequence_id,
            velocity=0.0,
            turn=0.0,
            rear_left_target_speed=0.0,
            rear_right_target_speed=0.0,
            rear_left_mode=MotorMode.STOP,
            rear_right_mode=MotorMode.STOP,
            completed=completed,
        )
