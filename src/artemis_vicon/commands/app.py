from __future__ import annotations
"""自动行驶小车客户端服务 CLI。"""

import logging
from typing import Annotated

import typer

from artemis_vicon.services import run_client

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger(__name__)

app = typer.Typer(add_completion=False, help="运行自动行驶小车 gRPC 客户端服务。")


@app.command()
def main(
    target: Annotated[
        str,
        typer.Option(
            "--target",
            help="仿真服务地址。",
            envvar="ARTEMIS_SIM_TARGET",
            show_envvar=True,
        ),
    ] = "127.0.0.1:50051",
    task: Annotated[
        str,
        typer.Option(
            "--task",
            help="要运行的任务编号，支持 0-4。",
            envvar="ARTEMIS_TASK",
            show_envvar=True,
        ),
    ] = "1",
    max_time: Annotated[
        float | None,
        typer.Option(
            "--max-time",
            help="覆盖任务时间上限，单位为秒。",
            envvar="ARTEMIS_MAX_TIME_S",
            show_envvar=True,
        ),
    ] = None,
    control_period: Annotated[
        float,
        typer.Option(
            "--control-period",
            help="控制周期，单位为秒。",
            envvar="ARTEMIS_CONTROL_PERIOD_S",
            show_envvar=True,
        ),
    ] = 0.02,
    random_seed: Annotated[
        int | None,
        typer.Option(
            "--seed",
            help="初始航向随机扰动种子。",
            envvar="ARTEMIS_RANDOM_SEED",
            show_envvar=True,
        ),
    ] = None,
    initial_yaw_noise: Annotated[
        float,
        typer.Option(
            "--initial-yaw-noise",
            help="初始航向均匀扰动范围，单位度。",
            envvar="ARTEMIS_INITIAL_YAW_NOISE_DEG",
            show_envvar=True,
        ),
    ] = 5.0,
) -> None:
    """连接仿真服务并运行 artemis-m0 风格小车固件控制器。"""

    logger.info(
        "Starting artemis-vicon client target=%s task=%s seed=%s yaw_noise=%s",
        target,
        task,
        random_seed,
        initial_yaw_noise,
    )
    result = run_client(
        target=target,
        task_id=task,
        max_time_s=max_time,
        control_period_s=control_period,
        random_seed=random_seed,
        initial_yaw_noise_deg=initial_yaw_noise,
    )
    typer.echo(f"Finished: reason={result.reason} reached_goal={result.reached_goal} elapsed={result.elapsed_time_s:.3f}s")


if __name__ == "__main__":
    app()
