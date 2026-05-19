from __future__ import annotations
"""自动行驶小车客户端服务 CLI。"""

import logging
from pathlib import Path
from typing import Annotated

import typer

from artemis_vicon.client import ArtemisViconClient

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
        typer.Argument(
            help="仿真服务地址。",
            envvar="ARTEMIS_SIM_TARGET",
            show_envvar=True,
        ),
    ],
    task_path: Annotated[
        Path,
        typer.Argument(
            help="本地任务动作 JSON 文件路径。",
            envvar="ARTEMIS_TASK_PATH",
            show_envvar=True,
        ),
    ],
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
            help="传给服务端用于 episode/noise 复现的随机种子。",
            envvar="ARTEMIS_RANDOM_SEED",
            show_envvar=True,
        ),
    ] = None,
) -> None:
    """连接仿真服务并运行 artemis-m0 风格小车任务控制器。"""

    logger.info(
        "Starting artemis-vicon client target=%s task=%s seed=%s",
        target,
        task_path,
        random_seed,
    )
    result = ArtemisViconClient(
        target=target,
        task_path=task_path,
        max_time_s=max_time,
        control_period_s=control_period,
        random_seed=random_seed,
    ).run()
    typer.echo(f"Finished: reason={result.reason} reached_goal={result.reached_goal} elapsed={result.elapsed_time_s:.3f}s")


if __name__ == "__main__":
    app()
