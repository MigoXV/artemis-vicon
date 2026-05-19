from __future__ import annotations

import logging
from collections.abc import Iterator

import numpy as np
import pandas as pd
import seaborn as sns

from artemis_vicon.controllers.pid import PIDController

from matplotlib import pyplot as plt

logger = logging.getLogger(__name__)

SIM_DURATION_S = 8.0
CONTROL_PERIOD_S = 0.02
PID_KP = np.float32(32)
PID_KI = np.float32(0.02)
PID_KD = np.float32(0.35)
PID_OUTPUT_LIMIT = np.float32(2.0)


def target_at(time_s: float) -> np.float32:
    """构造一段包含正负阶跃的目标曲线。"""

    if time_s < 1.0:
        return np.float32(0.0)
    if time_s < 5.5:
        return np.float32(1.0)
    return np.float32(-0.45)


def simulate_pid(
    duration_s: float,
    control_period_s: float,
    kp: np.float32,
    ki: np.float32,
    kd: np.float32,
    output_limit: np.float32,
) -> pd.DataFrame:
    controller = PIDController(ki=ki, kp=kp, kd=kd, output_limit=output_limit)
    measured = np.float32(0.0)
    tau = np.float32(0.45)
    steps = int(duration_s / control_period_s)
    rows: list[dict[str, float | str]] = []

    def target_stream() -> Iterator[np.float32]:
        for index in range(steps):
            time_s = np.float32(index * control_period_s)
            yield target_at(float(time_s))

    def measured_stream() -> Iterator[np.float32]:
        nonlocal measured
        while True:
            yield measured

    pid_stream = controller.infer_stream(target_stream(), measured_stream())
    for index, (raw_control, state) in zip(range(steps), pid_stream):
        time_s = np.float32(index * control_period_s)
        target = target_at(float(time_s))
        error = state[1]
        control = raw_control
        measured += (control - measured) * np.float32(control_period_s) / tau

        rows.extend(
            [
                {"time_s": float(time_s), "name": "target", "value": float(target)},
                {"time_s": float(time_s), "name": "measured", "value": float(measured)},
                {"time_s": float(time_s), "name": "error", "value": float(error)},
                {"time_s": float(time_s), "name": "control", "value": float(control)},
                {"time_s": float(time_s), "name": "integral", "value": float(state[0])},
            ]
        )

    return pd.DataFrame(rows)


def plot_pid_curves(data: pd.DataFrame) -> None:
    sns.set_theme(style="whitegrid", context="talk")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    response_data = data[data["name"].isin(["target", "measured", "error"])]
    control_data = data[data["name"] == "control"]

    sns.lineplot(
        data=response_data,
        x="time_s",
        y="value",
        hue="name",
        linewidth=2.2,
        ax=axes[0],
    )
    axes[0].set_title("PID Step Response")
    axes[0].set_ylabel("Position / Error")
    axes[0].set_xlabel("")

    sns.lineplot(
        data=control_data,
        x="time_s",
        y="value",
        hue="name",
        linewidth=2.2,
        color="tab:red",
        legend=False,
        ax=axes[1],
    )
    axes[1].set_title("Control Output")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Output")

    fig.tight_layout()
    plt.show()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )
    logger.info("开始生成 PID 可视化数据")
    data = simulate_pid(
        duration_s=SIM_DURATION_S,
        control_period_s=CONTROL_PERIOD_S,
        kp=PID_KP,
        ki=PID_KI,
        kd=PID_KD,
        output_limit=PID_OUTPUT_LIMIT,
    )
    plot_pid_curves(data)
    logger.info("PID 可视化曲线窗口已关闭")


if __name__ == "__main__":
    main()
