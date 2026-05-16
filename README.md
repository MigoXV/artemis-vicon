# Artemis Vicon

`artemis-vicon` 是 `artemis-m0` 小车固件控制逻辑的 Python/gRPC 客户端。仿真服务由 `artemis-mudri` 提供，本项目负责接收仿真观测帧，按 m0 固件语义计算 `Velocity`、`Turn`、左右后轮目标速度和电机模式，再通过 gRPC 流回传控制命令。

## 功能

- `artemis_vicon.firmware` 复刻 artemis-m0 的 8 路循迹、Yaw 校正、距离控制和 task0-4 任务状态机。
- `artemis_vicon.services` 负责连接仿真服务、维护双向流、转换观测帧和控制命令。
- `artemis_vicon.commands.app` 提供 Typer CLI 入口。
- `artemis_vicon.simulation.v1` 保存和 `artemis-mudri` 对接的 protobuf 生成代码。
- 保留 `pyserial` 依赖，供后续接入真实电机驱动板串口协议。

## 安装

```bash
poetry install
```

## 运行

先在 `artemis-mudri` 中启动仿真服务：

```bash
poetry run python -m artemis_mudri.commands.app serve --host 127.0.0.1 --port 50051 --task 1 --no-render
```

再在本项目中启动客户端：

```bash
poetry run python -m artemis_vicon.commands.app --target 127.0.0.1:50051 --task 1 --seed 7
```

也可以使用脚本入口：

```bash
poetry run artemis-vicon --target 127.0.0.1:50051 --task 1 --seed 7
```

## 配置

- `ARTEMIS_SIM_TARGET`：仿真服务地址，默认 `127.0.0.1:50051`。
- `ARTEMIS_TASK`：任务编号，支持 `0` 到 `4`。
- `ARTEMIS_MAX_TIME_S`：任务时间上限。
- `ARTEMIS_CONTROL_PERIOD_S`：控制周期，默认 `0.02` 秒。
- `ARTEMIS_RANDOM_SEED`：初始航向随机扰动种子。
- `ARTEMIS_INITIAL_YAW_NOISE_DEG`：初始航向均匀扰动范围，默认 `5` 度。

## 测试

```bash
poetry run python -m unittest discover -s tests -v
```
