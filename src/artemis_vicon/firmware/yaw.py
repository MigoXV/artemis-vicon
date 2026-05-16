from __future__ import annotations
"""Yaw 航向保持控制。"""

from artemis_vicon.firmware.math_utils import clamp, wrap_deg


class YawController:
    """对齐 bsp_ms901m.c 的航向保持 PID。"""

    def __init__(self) -> None:
        self.filtered_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.stable_counter = 0

    def reset(self) -> None:
        self.filtered_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.stable_counter = 0

    def compute(self, current_angle_deg: float, target_angle_deg: float) -> tuple[float, bool]:
        """返回 Turn 输出和稳定标志。"""

        raw_error = wrap_deg(current_angle_deg - target_angle_deg)
        self.filtered_error = 0.3 * self.filtered_error + 0.7 * raw_error
        error_d = self.filtered_error - self.previous_error
        pid_output = 0.3 * self.filtered_error + 0.015 * error_d
        self.integral = clamp(self.integral + self.filtered_error, -100.0, 100.0)
        self.previous_error = self.filtered_error
        if abs(self.filtered_error) < 2.0:
            self.stable_counter += 1
        else:
            self.stable_counter = 0
        return float(pid_output / 4.5), self.stable_counter >= 10
