from __future__ import annotations
"""Yaw 航向保持控制。"""

import numpy as np

from artemis_vicon.controllers.pid import PIDController


class YawHoldController:
    """对齐 bsp_ms901m.c 的航向保持控制。"""

    def __init__(self, pid_controller: PIDController | None = None) -> None:
        self.pid_controller = pid_controller or PIDController(ki=0.0, kp=0.3, kd=0.015)
        self.pid_state = self.pid_controller.initial_state()
        self.filtered_error = np.float32(0.0)
        self.stable_counter = 0

    @property
    def previous_error(self) -> np.float32:
        return self.pid_state[1]

    @property
    def integral(self) -> np.float32:
        return self.pid_state[0]

    def reset(self) -> None:
        self.pid_state = self.pid_controller.initial_state()
        self.filtered_error = np.float32(0.0)
        self.stable_counter = 0

    def reset_stability(self) -> None:
        self.stable_counter = 0

    def compute(self, current_angle_deg: float, target_angle_deg: float) -> tuple[np.float32, bool]:
        """返回 Turn 输出和稳定标志。"""

        def yaw_error(target: np.float32, current: np.float32) -> np.float32:
            raw_error = _wrap_deg(target - current)
            self.filtered_error = np.float32(0.3) * self.filtered_error + np.float32(0.7) * raw_error
            return self.filtered_error

        pid_output, self.pid_state = next(
            self.pid_controller.infer_stream(
                iter((np.float32(target_angle_deg),)),
                iter((np.float32(current_angle_deg),)),
                initial_state=self.pid_state,
                error_fn=yaw_error,
            )
        )
        if abs(self.filtered_error) < 2.0:
            self.stable_counter += 1
        else:
            self.stable_counter = 0
        return -pid_output / np.float32(4.5), self.stable_counter >= 10


def _wrap_deg(angle_deg: float) -> np.float32:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return np.float32(180.0 if wrapped == -180.0 else wrapped)
