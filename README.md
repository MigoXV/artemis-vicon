# Artemis Vicon

`artemis-vicon` 是面向自动行驶小车的控制器运行时。当前主线用于连接 `artemis-mudri` 提供的仿真 ABI：控制器接收观测帧，按 artemis-m0 任务语义计算左右后轮目标速度，并通过 ZeroMQ JSON ABI 推进仿真 episode。

本项目不负责 MuJoCo 仿真、赛道几何、传感器噪声或 viewer。`artemis-mudri` 负责环境和 episode 生命周期，`artemis-vicon` 负责任务动作、巡线、航向保持、运动命令合成和控制循环。

gRPC 运行依赖仍然保留，用于后续接入强化学习引擎或其它外部引擎；但当前 `artemis-mudri` 仿真服务不再通过 gRPC 连接，旧的 mudri gRPC simulation proto 绑定已移除。

## 功能

- `artemis_vicon.vehicle` 提供 8 路循迹、Yaw 航向保持、动作状态机和左右后轮命令合成。
- `artemis_vicon.controllers` 提供通用 PID/LQR 控制算法。
- `artemis_vicon.engine` 定义外部引擎边界，并提供当前可用的 `mudri_zmq` 实现。
- `artemis_vicon.config` 使用 OmegaConf 加载显式 YAML 模型配置。
- `artemis_vicon.client` 负责装配控制器、连接引擎并运行 episode。
- `artemis_vicon.commands.app` 提供 Typer CLI 入口。
- `examples/m0` 保存 artemis-m0 task0-4 的 JSON 动作序列。
- 保留 `pyserial` 依赖，供后续接入真实电机驱动板串口协议。

## 安装

```bash
poetry install
```

## 运行

先在 `artemis-mudri` 中启动 ZMQ JSON 仿真服务：

```bash
poetry run python -m artemis_mudri.commands.app serve --bind tcp://127.0.0.1:5556 --no-render
```

在本项目中显式指定 YAML 配置运行控制器：

```bash
poetry run artemis-vicon --config examples/configs/mudri_model.yaml
```

也可以使用模块入口：

```bash
poetry run python -m artemis_vicon.commands.app --config examples/configs/mudri_model.yaml
```

### MCU 串口桥接

`artemis-vicon-serial-bridge` 用作单片机和 `artemis-mudri` 之间的桥接程序：下层通过 `pyserial` 读取 MCU 串口行协议，上层复用 `mudri_zmq` 客户端访问 `artemis-mudri` 的 ZeroMQ JSON ABI。

先启动 `artemis-mudri` ZMQ JSON 服务，然后启动桥接：

```bash
poetry run artemis-vicon-serial-bridge --port COM3 --baudrate 115200 --endpoint tcp://127.0.0.1:5556
```

也可以通过环境变量配置：

```env
ARTEMIS_SERIAL_PORT=COM3
ARTEMIS_SERIAL_BAUDRATE=115200
ARTEMIS_MUDRI_ENDPOINT=tcp://127.0.0.1:5556
```

串口协议采用换行结尾的 ASCII 文本，便于 MCU 调试和示波器/串口助手排查：

```text
START max_time_s=120 control_period_s=0.02
STEP 0 7.0 7.0
STOP task_completed
```

桥接响应同样是一行 ASCII：

```text
STARTED time_limit_s=120 control_period_s=0.02 seq=0 t=0 yaw=0 distance_cm=0 digital=00000000
OBS seq=1 t=0.02 yaw=0.1 distance_cm=1.2 digital=00111100
FINISHED reason=task_completed reached_goal=1 elapsed_time_s=3.42
ERR message=unknown_command:_PING
```

`STEP` 会被桥接为 `artemis-mudri` 的 `step` 请求，其中三个位置参数分别是 `sequence_id`、`rear_left_target_speed`、`rear_right_target_speed`。也支持键值形式：`STEP seq=0 left=7.0 right=7.0`。

## 模型配置

模型配置通过 `--config` 或 `CONFIG_PATH` 指定。VS Code 调试时会读取 `.env`，因此 `.env` 里至少需要提供模型配置文件路径、仿真服务地址和 m0 任务路径：

```env
CONFIG_PATH=examples/configs/mudri_model.yaml
ARTEMIS_MUDRI_ENDPOINT=tcp://127.0.0.1:5556
ARTEMIS_M0_TASK_PATH=examples/m0/task1.json
```

示例：

```yaml
engine:
  kind: mudri_zmq
  endpoint: ${oc.env:ARTEMIS_MUDRI_ENDPOINT,tcp://127.0.0.1:5556}

start:
  max_time_s: 120
  control_period_s: 0.02
  initial_pose: null
  initial_progress_index: 0
  random_seed: null

controller:
  task_path: ${oc.env:ARTEMIS_M0_TASK_PATH}
  line_sensor_darkness_threshold: 0.55
  line_tracking_pid:
    ki: 0.0
    kp: 25.0
    kd: 3.5
    output_limit: null
  yaw_hold_pid:
    ki: 0.0
    kp: 0.3
    kd: 0.015
    output_limit: null
```

`controller.task_path` 不提供默认值，缺少 `ARTEMIS_M0_TASK_PATH` 时配置加载会失败。`line_tracking_pid` 和 `yaw_hold_pid` 默认值与当前控制器代码原始参数一致，可以在 YAML 中显式调整。

`engine.kind` 当前支持：

- `mudri_zmq`：连接 `artemis-mudri` 的 ZeroMQ JSON ABI。
- `grpc`：未来外部 gRPC 引擎占位入口；当前会明确报错，因为强化学习或其它 gRPC 引擎 ABI 尚未定义。

`start` 字段会透传给 `artemis-mudri` 的 `start` 请求。服务端噪声配置仍由 `artemis-mudri` 通过 `--noise-config` 或 `ARTEMIS_NOISE_CONFIG` 管理，当前模型配置不承载噪声模型。

## ABI 映射

`artemis-vicon` 从 `mudri` observation 中提取控制器需要的最小观测：

- `sequence_id` 和 `sim_time_s` 直接使用 ABI 字段。
- `yaw_deg` 来自 `imu.yaw_deg`。
- `digital_values` 优先使用 `line_sensor.digital`；缺失时使用 `line_sensor_darkness >= line_sensor_darkness_threshold` 计算。
- `forward_distance_cm` 来自 `encoder.forward_distance_cm`，不再由客户端根据 kinematics 自行积分。

控制器输出只编码为 `step` 请求中的：

- `sequence_id`
- `rear_left_target_speed`
- `rear_right_target_speed`

## 测试

```bash
poetry run python -m unittest discover -s tests -v
```
