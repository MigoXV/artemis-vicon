from __future__ import annotations
"""未来外部 gRPC 引擎占位适配器。"""

from artemis_vicon.config import StartConfig
from artemis_vicon.engine.base import EngineFinished, EngineObservation, EngineStarted
from artemis_vicon.schemas import ControlCommand

_MESSAGE = "gRPC external engine ABI is not defined yet."


class GrpcEngineClient:
    """占位 gRPC 引擎客户端。

    当前 artemis-mudri 仿真服务不再使用 gRPC；本类仅保留未来外部引擎接入位置。
    """

    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        raise NotImplementedError(_MESSAGE)

    def start(self, config: StartConfig) -> EngineStarted:
        raise NotImplementedError(_MESSAGE)

    def step(self, command: ControlCommand) -> EngineObservation | EngineFinished:
        raise NotImplementedError(_MESSAGE)

    def stop(self, reason: str) -> EngineFinished:
        raise NotImplementedError(_MESSAGE)

    def close(self) -> None:
        return None
