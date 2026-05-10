# Artemis Vicon

`artemis-vicon` 是自动行驶小车的独立控制客户端服务。仿真服务由 `artemis-mudri` 提供，本项目负责连接仿真服务、运行循线控制器，并通过 gRPC 流回传左右电机驱动板命令。

## 功能内容

- `artemis_vicon.control` 提供 PID、PD、LQR 循线控制器和混合任务状态机。
- `artemis_vicon.services` 负责 gRPC 仿真服务连接与流式编排。
- `artemis_vicon.domain` 保存电机命令映射等领域对象。
- 使用 Typer 提供标准 CLI 入口。
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
poetry run python -m artemis_vicon.commands.app --target 127.0.0.1:50051 --task 1 --controller pid
```

也可以使用脚本入口：

```bash
poetry run artemis-vicon --target 127.0.0.1:50051 --task 1 --controller pid
```

## 配置

- `ARTEMIS_SIM_TARGET`：仿真服务地址。
- `ARTEMIS_TASK`：任务编号。
- `ARTEMIS_LINE_FOLLOW_CONTROLLER`：循线控制器，可选 `pid`、`pd`、`lqr`。
- `ARTEMIS_MAX_TIME_S`：任务时间上限。
- `ARTEMIS_CONTROL_PERIOD_S`：控制周期。
- `ARTEMIS_SPEED_SCALE`：速度倍率。

## 测试

```bash
poetry run python -m unittest discover -s tests -v
```
