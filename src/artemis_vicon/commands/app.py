from __future__ import annotations
"""自动行驶小车控制器运行时 CLI。"""

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

app = typer.Typer(add_completion=False, help="运行自动行驶小车控制器。")


@app.command()
def main(
    config: Annotated[
        Path,
        typer.Option(
            "--config",
            "-c",
            exists=True,
            dir_okay=False,
            readable=True,
            envvar="CONFIG_PATH",
            show_envvar=True,
            help="显式指定 OmegaConf YAML 模型配置。",
        ),
    ],
) -> None:
    """连接外部引擎并运行 artemis-m0 风格小车任务控制器。"""

    logger.info("Starting artemis-vicon runtime config=%s", config)
    result = ArtemisViconClient.from_config_path(config).run()
    typer.echo(f"Finished: reason={result.reason} reached_goal={result.reached_goal} elapsed={result.elapsed_time_s:.3f}s")


if __name__ == "__main__":
    app()
