from __future__ import annotations

from collections.abc import Generator, Iterator

import numpy as np

from artemis_vicon.controllers.base import BaseController, ErrorFunction
from artemis_vicon.controllers.utils import default_error_fn


class PIDController(BaseController):
    """无状态 PID 控制器。

    控制器只保存增益和输出限制；运行状态由调用方通过 `initial_state` 显式传入和接收。
    """

    # 状态向量为 [积分项, 当前误差, 上一帧误差]。
    a = np.array([[1, 0, 0], [0, 0, 0], [0, 1, 0]], dtype=np.float32)
    b = np.array([1, 1, 0], dtype=np.float32)

    def __init__(self, ki: float, kp: float, kd: float, output_limit: float | None = None):
        """初始化 PID 参数。

        Args:
            ki: 积分增益。
            kp: 比例增益。
            kd: 微分增益。
            output_limit: 输出限幅，None 表示不限制。
        """

        self.gains = np.array([ki, kp, kd], dtype=np.float32)
        self.c = np.array([ki, kp + kd, -kd], dtype=np.float32)
        self.output_limit = None if output_limit is None else np.float32(output_limit)

    def initial_state(self) -> np.ndarray:
        """创建一份新的 PID 初始状态。"""

        return np.zeros(3, dtype=np.float32)

    def infer_stream(
        self,
        setpoint_stream: Iterator[np.float32],
        observation_stream: Iterator[np.float32],
        initial_state: np.ndarray | None = None,
        error_fn: ErrorFunction | None = None,
    ) -> Generator[tuple[np.float32, np.ndarray], None, None]:
        """按流式输入计算控制输出。

        该方法不会在控制器对象上保存运行状态，而是把状态随每次输出一起返回。
        调用方如果要连续控制，应把上一次返回的状态作为下一次的 `initial_state` 传回。
        给定值流和观测值流会逐项配对计算，任一迭代器结束后输出流也结束。

        Args:
            setpoint_stream: 给定值迭代器，例如目标速度、目标角度或目标位置。
            observation_stream: 观测值迭代器，例如实测速度、当前角度或当前位置。
            initial_state: 上一次调用返回的状态；不传则创建零状态。
            error_fn: 误差函数，默认使用 `setpoint - observation`；角度环绕等场景可传入自定义函数。

        Yields:
            每一步的控制输出和新的 PID 状态，状态向量为 `[积分项, 当前误差, 上一帧误差]`。
        """

        # 复制或创建状态，避免把运行状态隐式保存在控制器实例中。
        state_x = self.initial_state() if initial_state is None else initial_state.astype(np.float32)

        # error_fn 是单次调用的策略参数，不进入构造函数，保持控制器对象无运行状态。
        calculate_error = default_error_fn if error_fn is None else error_fn

        # zip 负责按时间步对齐给定值和观测值；长度不一致时按较短的流结束。
        for setpoint, observation in zip(setpoint_stream, observation_stream):
            setpoint_value = np.float32(setpoint)
            observation_value = np.float32(observation)
            error = calculate_error(setpoint_value, observation_value)

            # 状态转移：
            #   新积分项 = 旧积分项 + 当前误差
            #   当前误差 = error
            #   上一帧误差 = 旧当前误差
            state_x = self.a @ state_x + self.b * np.float32(error)

            # 等价于 ki * integral + kp * error + kd * (error - previous_error)。
            control = self.c @ state_x
            if self.output_limit is not None:
                # 输出限幅在控制器内部统一处理，调用方无需重复裁剪。
                control = np.clip(control, -self.output_limit, self.output_limit)

            yield np.float32(control), state_x
