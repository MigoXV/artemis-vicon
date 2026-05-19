from __future__ import annotations
"""8 路循迹误差与 PID 控制。"""

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from artemis_vicon.controllers.pid import PIDController


IntArray = NDArray[np.int_]


class LineTrackController:
    """对齐 bsp_track.c 的 8 路循迹误差与 PID。"""

    weights = np.array([-4, -3, -2, -1, 1, 2, 3, 4], dtype=np.float32)

    def __init__(self, pid_controller: PIDController | None = None) -> None:
        self.pid_controller = pid_controller or PIDController(ki=0.0, kp=25.0, kd=3.5)
        self.pid_state = self.pid_controller.initial_state()
        self.new_error = np.float32(0.0)

    @property
    def previous_error(self) -> np.float32:
        return self.pid_state[1]

    @property
    def integral(self) -> np.float32:
        return self.pid_state[0]

    def reset(self) -> None:
        self.new_error = np.float32(0.0)
        self.pid_state = self.pid_controller.initial_state()

    def scan(self, digital_values: Sequence[int] | IntArray) -> bool:
        """读取传感器状态并计算 C 端 new_error。"""

        values = np.asarray(digital_values, dtype=np.int_)
        detected = values[: self.weights.size] > 0
        detected_count = np.count_nonzero(detected)
        if detected_count > 0:
            self.new_error = self.weights[detected].sum() / detected_count
            return True
        return False

    def compute_turn(self, velocity: float) -> np.float32:
        """计算 C 端 track_pid 输出。"""

        pid_value, self.pid_state = next(
            self.pid_controller.infer_stream(
                iter((np.float32(0.0),)),
                iter((self.new_error,)),
                initial_state=self.pid_state,
            )
        )
        calibration = np.float32(150.0) / (np.float32(velocity) + np.float32(1.0))
        return pid_value / calibration
