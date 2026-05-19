# Artemis Vicon

`artemis-vicon` 是 `artemis-m0` 小车控制逻辑的 Python/gRPC 客户端。仿真服务由 `artemis-mudri` 提供，本项目负责接收仿真观测帧，按 m0 任务语义计算 `Velocity`、`Turn`、左右后轮目标速度和电机模式，再通过 gRPC 流回传控制命令。

## 功能

- `artemis_vicon.controllers` 提供通用 PID 控制算法。
- `artemis_vicon.schemas` 保存观测帧、控制命令、任务动作和电机模式等数据结构。
- `artemis_vicon.vehicle` 提供 8 路循迹、Yaw 航向保持、动作状态机和左右后轮命令合成等小车业务控制器。
- `artemis_vicon.client` 封装 gRPC 客户端，负责连接仿真服务、维护双向流、转换观测帧和控制命令。
- `artemis_vicon.commands.app` 提供 Typer CLI 入口。
- `artemis_vicon.protos.simulation.v1` 保存和 `artemis-mudri` 对接的 protobuf 定义及生成代码。
- `examples/m0` 保存 artemis-m0 task0-4 的 JSON 动作序列。
- 保留 `pyserial` 依赖，供后续接入真实电机驱动板串口协议。

## 安装

```bash
poetry install
```

## 生成 Proto 代码

```bash
bash scripts/run_grpcio_tools.sh
```

脚本会使用 Poetry 环境运行 `grpcio-tools`，并生成 `*_pb2.py`、`*_pb2.pyi`、`*_pb2_grpc.py` 和 `*_pb2_grpc.pyi`。

## 运行

先在 `artemis-mudri` 中启动仿真服务：

```bash
poetry run python -m artemis_mudri.commands.app serve --host 127.0.0.1 --port 50051 --no-render
```

如需启用服务端现实噪声，在 `artemis-mudri` 中通过 `--noise-config` 指定噪声配置：

```bash
poetry run python -m artemis_mudri.commands.app serve \
  --host 127.0.0.1 \
  --port 50051 \
  --no-render \
  --noise-config exampes/configs/noise/weak.yaml
```

再在本项目中启动客户端：

```bash
poetry run python -m artemis_vicon.commands.app 127.0.0.1:50051 examples/m0/task1.json --seed 7
```

也可以使用脚本入口：

```bash
poetry run artemis-vicon 127.0.0.1:50051 examples/m0/task1.json --seed 7
```

## 配置

- `ARTEMIS_SIM_TARGET`：仿真服务地址，默认 `127.0.0.1:50051`。
- `ARTEMIS_TASK_PATH`：本地任务动作 JSON 文件路径，默认 `examples/m0/task1.json`；该值不会发送给仿真服务。
- `ARTEMIS_MAX_TIME_S`：任务时间上限。
- `ARTEMIS_CONTROL_PERIOD_S`：控制周期，默认 `0.02` 秒。
- `ARTEMIS_RANDOM_SEED`：传给服务端用于 episode/noise 复现的随机种子。

噪声参数不再由 `artemis-vicon` 通过 gRPC 请求单独指定。新版 `artemis-mudri` 统一在服务端通过 `--noise-config` 或 `ARTEMIS_NOISE_CONFIG` 加载初始位姿、巡线传感器、IMU、编码器和执行器噪声配置；客户端只发送 seed、控制周期、时间上限和控制命令。

## 测试

```bash
poetry run python -m unittest discover -s tests -v
```
