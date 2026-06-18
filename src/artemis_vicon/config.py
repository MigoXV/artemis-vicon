from __future__ import annotations
"""模型配置加载。"""

from pathlib import Path
from typing import Literal

from omegaconf import OmegaConf
from pydantic import BaseModel, ConfigDict, Field


class EngineConfig(BaseModel):
    """外部引擎连接配置。"""

    model_config = ConfigDict(extra="forbid")

    kind: Literal["mudri_zmq", "grpc"]
    endpoint: str = Field(min_length=1)


class PoseConfig(BaseModel):
    """二维初始位姿。"""

    model_config = ConfigDict(extra="forbid")

    x_m: float
    y_m: float
    yaw_rad: float


class StartConfig(BaseModel):
    """仿真 episode 启动配置。"""

    model_config = ConfigDict(extra="forbid")

    max_time_s: float | None = None
    control_period_s: float | None = 0.02
    initial_pose: PoseConfig | None = None
    initial_progress_index: int = 0
    random_seed: int | None = None


class PidConfig(BaseModel):
    """PID 参数配置。"""

    model_config = ConfigDict(extra="forbid")

    ki: float
    kp: float
    kd: float
    output_limit: float | None = None


class ControllerConfig(BaseModel):
    """控制器配置。"""

    model_config = ConfigDict(extra="forbid")

    task_path: Path
    line_sensor_darkness_threshold: float = 0.55
    line_tracking_pid: PidConfig = Field(default_factory=lambda: PidConfig(ki=0.0, kp=25.0, kd=3.5))
    yaw_hold_pid: PidConfig = Field(default_factory=lambda: PidConfig(ki=0.0, kp=0.3, kd=0.015))


class RunConfig(BaseModel):
    """artemis-vicon 单次控制模型配置。"""

    model_config = ConfigDict(extra="forbid")

    engine: EngineConfig
    start: StartConfig = Field(default_factory=StartConfig)
    controller: ControllerConfig


def load_run_config(path: Path) -> RunConfig:
    """使用 OmegaConf 从 YAML 文件加载模型配置。"""

    raw_config = OmegaConf.load(path)
    data = OmegaConf.to_container(raw_config, resolve=True)
    return RunConfig.model_validate(data)
