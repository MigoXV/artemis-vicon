from __future__ import annotations
"""8 路循迹误差与 PID 控制。"""

from artemis_vicon.firmware.math_utils import clamp


class TrackController:
    """对齐 bsp_track.c 的 8 路循迹误差与 PID。"""

    weights = (-4, -3, -2, -1, 1, 2, 3, 4)

    def __init__(self) -> None:
        self.new_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

    def reset(self) -> None:
        self.new_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

    def scan(self, digital_values: tuple[int, ...]) -> bool:
        """读取传感器状态并计算 C 端 new_error。"""

        detected = [(value > 0) for value in digital_values[: len(self.weights)]]
        detected_count = sum(1 for value in detected if value)
        if detected_count > 0:
            total_error = sum(weight for weight, active in zip(self.weights, detected) if active)
            self.new_error = float(total_error / detected_count)
            return True
        self.new_error = self.previous_error
        return False

    def pid(self, velocity: float) -> float:
        """计算 C 端 track_pid 输出。"""

        p_term = self.new_error
        self.integral = clamp(self.integral + self.new_error, -30.0, 30.0)
        d_term = self.new_error - self.previous_error
        pid_value = 25.0 * p_term + 0.0 * self.integral + 3.5 * d_term
        self.previous_error = self.new_error
        calibration = 150.0 / (velocity + 1.0)
        return float(-pid_value / calibration)
