from __future__ import annotations
"""控制器运行时。"""

import logging
from dataclasses import dataclass
from pathlib import Path

from artemis_vicon.config import PidConfig, RunConfig, load_run_config
from artemis_vicon.engine import (
    EngineClient,
    EngineFinished,
    EngineObservation,
    MudriObservationAdapter,
    create_engine_client,
)
from artemis_vicon.controllers.pid import PIDController
from artemis_vicon.schemas import Observation
from artemis_vicon.vehicle import LineTrackController, MissionStateMachine, YawHoldController, load_task_actions

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ClientResult:
    """客户端运行结果。"""

    reason: str
    reached_goal: bool
    elapsed_time_s: float


@dataclass(frozen=True)
class ArtemisViconClient:
    """运行 artemis-m0 风格任务控制器。"""

    config: RunConfig
    engine_client: EngineClient | None = None

    @classmethod
    def from_config_path(cls, path: Path) -> "ArtemisViconClient":
        """从 YAML 模型配置文件创建控制器运行时。"""

        return cls(load_run_config(path))

    def run(self) -> ClientResult:
        """运行一次 episode。"""

        engine = self.engine_client or create_engine_client(self.config.engine)
        adapter = MudriObservationAdapter(
            line_sensor_darkness_threshold=self.config.controller.line_sensor_darkness_threshold,
        )
        controller = MissionStateMachine(
            load_task_actions(self.config.controller.task_path),
            line_tracker=LineTrackController(_pid_controller(self.config.controller.line_tracking_pid)),
            yaw=YawHoldController(_pid_controller(self.config.controller.yaw_hold_pid)),
        )

        try:
            started = engine.start(self.config.start)
            logger.info(
                "Connected to engine kind=%s endpoint=%s task=%s time_limit=%s control_period=%s",
                self.config.engine.kind,
                self.config.engine.endpoint,
                self.config.controller.task_path,
                started.time_limit_s,
                started.control_period_s,
            )
            observation = adapter.from_wire(started.observation)
            while True:
                command = controller.step(observation)
                if command.completed:
                    return _client_result(engine.stop("task_completed"))
                response = engine.step(command)
                if isinstance(response, EngineFinished):
                    return _client_result(response)
                observation = _observation_from_response(adapter, response)
        finally:
            engine.close()


def _observation_from_response(adapter: MudriObservationAdapter, response: EngineObservation) -> Observation:
    return adapter.from_wire(response.observation)


def _client_result(finished: EngineFinished) -> ClientResult:
    return ClientResult(
        reason=finished.reason,
        reached_goal=bool(finished.summary.get("reached_goal", False)),
        elapsed_time_s=float(finished.summary.get("elapsed_time_s", 0.0)),
    )


def _pid_controller(config: PidConfig) -> PIDController:
    return PIDController(
        ki=config.ki,
        kp=config.kp,
        kd=config.kd,
        output_limit=config.output_limit,
    )
