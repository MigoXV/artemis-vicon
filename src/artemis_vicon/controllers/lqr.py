from __future__ import annotations

from collections.abc import Generator, Iterator

import numpy as np

from artemis_vicon.controllers.base import BaseController, ControlValue, ErrorFunction
from artemis_vicon.controllers.utils import default_error_fn


class LQRController(BaseController):
    """基于外部增益矩阵 K 的无状态 LQR 控制器。"""

    def __init__(self, gain_matrix: np.ndarray, output_limit: float | np.ndarray | None = None):
        """初始化 LQR 参数。

        Args:
            gain_matrix: 外部计算好的 LQR 增益矩阵 K。
            output_limit: 输出限幅，None 表示不限制。
        """

        self.gain_matrix = np.asarray(gain_matrix, dtype=np.float32)
        self.output_limit = None if output_limit is None else np.asarray(output_limit, dtype=np.float32)

    def initial_state(self) -> np.ndarray:
        """LQR 推理不需要运行状态，返回空状态以满足统一接口。"""

        return np.array([], dtype=np.float32)

    def infer_stream(
        self,
        setpoint_stream: Iterator[ControlValue],
        observation_stream: Iterator[ControlValue],
        initial_state: np.ndarray | None = None,
        error_fn: ErrorFunction | None = None,
    ) -> Generator[tuple[np.ndarray, np.ndarray], None, None]:
        """按流式输入计算 LQR 控制输出。"""

        state_x = self.initial_state() if initial_state is None else initial_state.astype(np.float32)
        calculate_error = default_error_fn if error_fn is None else error_fn
        for setpoint, observation in zip(setpoint_stream, observation_stream):
            setpoint_value = np.asarray(setpoint, dtype=np.float32)
            observation_value = np.asarray(observation, dtype=np.float32)
            error = np.asarray(calculate_error(setpoint_value, observation_value), dtype=np.float32)
            control = self.gain_matrix @ error
            if self.output_limit is not None:
                control = np.clip(control, -self.output_limit, self.output_limit)
            yield np.asarray(control, dtype=np.float32), state_x
