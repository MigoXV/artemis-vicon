from __future__ import annotations
"""外部引擎客户端边界。"""

from dataclasses import dataclass
from typing import Any, Protocol

from artemis_vicon.config import EngineConfig, StartConfig
from artemis_vicon.schemas import ControlCommand

JsonObject = dict[str, Any]


@dataclass(frozen=True)
class EngineStarted:
    """episode 启动结果。"""

    time_limit_s: float
    control_period_s: float
    observation: JsonObject


@dataclass(frozen=True)
class EngineObservation:
    """episode 中间观测。"""

    observation: JsonObject


@dataclass(frozen=True)
class EngineFinished:
    """episode 结束结果。"""

    reason: str
    summary: JsonObject


class EngineClient(Protocol):
    """控制器运行时使用的外部引擎协议。"""

    def start(self, config: StartConfig) -> EngineStarted:
        """启动一个 episode。"""

    def step(self, command: ControlCommand) -> EngineObservation | EngineFinished:
        """发送一帧控制命令并获得下一帧结果。"""

    def stop(self, reason: str) -> EngineFinished:
        """主动停止 episode。"""

    def close(self) -> None:
        """释放连接资源。"""


def create_engine_client(config: EngineConfig) -> EngineClient:
    """根据配置创建外部引擎客户端。"""

    if config.kind == "mudri_zmq":
        from artemis_vicon.engine.mudri_zmq import MudriZmqEngineClient

        return MudriZmqEngineClient(config.endpoint)
    if config.kind == "grpc":
        from artemis_vicon.engine.grpc import GrpcEngineClient

        return GrpcEngineClient(config.endpoint)
    raise ValueError(f"Unsupported engine kind: {config.kind!r}.")
