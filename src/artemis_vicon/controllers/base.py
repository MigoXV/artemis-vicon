from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Generator, Iterator

import numpy as np


ControlValue = np.float32 | np.ndarray
ErrorFunction = Callable[[ControlValue, ControlValue], ControlValue]


class BaseController(ABC):
    """无状态控制器抽象基类。"""

    @abstractmethod
    def initial_state(self) -> np.ndarray:
        """创建控制器初始状态。"""

    @abstractmethod
    def infer_stream(
        self,
        setpoint_stream: Iterator[ControlValue],
        observation_stream: Iterator[ControlValue],
        initial_state: np.ndarray | None = None,
        error_fn: ErrorFunction | None = None,
    ) -> Generator[tuple[ControlValue, np.ndarray], None, None]:
        """按给定值流和观测值流计算控制输出。"""
