# 单片机实现 S 型走位需要了解的接口信息

## 目标

你的目标是让单片机生成一组 S 型走位控制命令，并把命令传入仿真。

当前通信链路是：

```text
MCU <--串口 ASCII 行协议--> artemis-vicon-serial-bridge <--ZeroMQ JSON--> artemis-mudri 仿真服务
```

单片机不需要实现 ZeroMQ，也不需要直接发送 JSON。单片机只需要通过串口向 `artemis-vicon-serial-bridge` 发送文本命令。

## 启动桥接服务

先启动 `artemis-mudri` 仿真服务，再启动串口桥接：

```bash
poetry run artemis-vicon-serial-bridge --port COM3 --baudrate 115200 --endpoint tcp://127.0.0.1:5556
```

其中：

```text
COM3                         单片机连接到电脑后的串口号
115200                       串口波特率
tcp://127.0.0.1:5556         artemis-mudri 的 ZMQ JSON 服务地址
```

## 串口基础规则

```text
编码: ASCII
结束符: \n
交互方式: MCU 发送一行命令，桥接服务返回一行响应
推荐流程: START -> 等待 STARTED -> 循环 STEP/OBS -> STOP
```

单片机端不要连续无等待地刷 `STEP`。建议每发送一条 `STEP`，等待收到 `OBS` 或 `FINISHED` 后，再发送下一条。

## S 型走位的核心控制量

仿真接收的是左右后轮目标速度：

```text
rear_left_target_speed      左后轮目标速度
rear_right_target_speed     右后轮目标速度
```

S 型走位本质上是周期性改变左右轮速度差：

```text
直行: left_speed == right_speed
左转: left_speed <  right_speed
右转: left_speed >  right_speed
```

例如：

```text
STEP 0 6.0 9.0     左转段
STEP 1 6.0 9.0
STEP 2 6.0 9.0

STEP 3 9.0 6.0     右转段
STEP 4 9.0 6.0
STEP 5 9.0 6.0
```

速度单位沿用 `artemis-mudri` 对 `rear_left_target_speed` / `rear_right_target_speed` 的定义。当前桥接层只负责透传数值，不做单位换算。

## MCU 发送命令

### 1. START

启动一次仿真 episode。

格式：

```text
START [max_time_s=秒数] [control_period_s=秒数] [initial_progress_index=序号] [random_seed=种子]
```

推荐 S 型测试命令：

```text
START max_time_s=20 control_period_s=0.02
```

字段含义：

```text
max_time_s               本次仿真最长运行时间，单位秒
control_period_s         控制周期，单位秒，建议先用 0.02
initial_progress_index   初始任务进度，通常不用传
random_seed              随机种子，通常不用传
```

### 2. STEP

发送一帧左右轮目标速度。

位置参数格式：

```text
STEP <sequence_id> <left_speed> <right_speed>
```

示例：

```text
STEP 0 6.0 9.0
```

也支持键值格式：

```text
STEP seq=0 left=6.0 right=9.0
```

字段含义：

```text
sequence_id     单片机维护的控制帧序号，从 0 开始递增
left_speed      左后轮目标速度
right_speed     右后轮目标速度
```

桥接层会把它转换成仿真的 JSON：

```json
{
  "type": "step",
  "sequence_id": 0,
  "rear_left_target_speed": 6.0,
  "rear_right_target_speed": 9.0
}
```

### 3. STOP

结束仿真。

格式：

```text
STOP [reason]
```

示例：

```text
STOP s_curve_done
```

## 桥接服务返回响应

### 1. STARTED

`START` 成功后返回。

格式：

```text
STARTED time_limit_s=<秒数> control_period_s=<秒数> seq=<序号> t=<仿真时间> yaw=<航向角> distance_cm=<距离> digital=<巡线数字量>
```

示例：

```text
STARTED time_limit_s=20 control_period_s=0.02 seq=0 t=0 yaw=0 distance_cm=0 digital=00000000
```

### 2. OBS

每次 `STEP` 后，如果仿真还在运行，会返回一帧观测。

格式：

```text
OBS seq=<序号> t=<仿真时间> yaw=<航向角> distance_cm=<距离> digital=<巡线数字量>
```

示例：

```text
OBS seq=1 t=0.02 yaw=0.4 distance_cm=1.1 digital=00000000
```

S 型走位最关心这些字段：

```text
seq          仿真返回的观测序号
t            当前仿真时间，单位秒
yaw          当前车身航向角，单位度
distance_cm  前进距离，单位厘米
```

如果你只做开环 S 型，可以只用 `OBS` 作为下一帧 `STEP` 的同步信号。

如果你要做闭环 S 型，可以用：

```text
yaw          判断当前转向幅度是否够
distance_cm  判断当前 S 型段是否走完
```

### 3. FINISHED

仿真结束时返回。

格式：

```text
FINISHED reason=<原因> reached_goal=<0或1> elapsed_time_s=<秒数>
```

示例：

```text
FINISHED reason=s_curve_done reached_goal=1 elapsed_time_s=8.4
```

### 4. ERR

命令格式错误或仿真服务错误时返回。

格式：

```text
ERR message=<错误信息>
```

示例：

```text
ERR message=unknown_command:_PING
```

收到 `ERR` 后，桥接服务不会自动退出。单片机可以修正命令后继续发送。

## 开环 S 型控制示例

下面示例假设：

```text
control_period_s = 0.02
每个 S 型半周期持续 25 帧，也就是 0.5 秒
左转段速度: left=6.0, right=9.0
右转段速度: left=9.0, right=6.0
```

交互示例：

```text
MCU -> Bridge: START max_time_s=20 control_period_s=0.02
Bridge -> MCU: STARTED time_limit_s=20 control_period_s=0.02 seq=0 t=0 yaw=0 distance_cm=0 digital=00000000

MCU -> Bridge: STEP 0 6.0 9.0
Bridge -> MCU: OBS seq=1 t=0.02 yaw=0.3 distance_cm=0.8 digital=00000000

MCU -> Bridge: STEP 1 6.0 9.0
Bridge -> MCU: OBS seq=2 t=0.04 yaw=0.6 distance_cm=1.6 digital=00000000

...

MCU -> Bridge: STEP 24 6.0 9.0
Bridge -> MCU: OBS seq=25 t=0.5 yaw=8.0 distance_cm=20.0 digital=00000000

MCU -> Bridge: STEP 25 9.0 6.0
Bridge -> MCU: OBS seq=26 t=0.52 yaw=7.7 distance_cm=20.8 digital=00000000

...

MCU -> Bridge: STOP s_curve_done
Bridge -> MCU: FINISHED reason=s_curve_done reached_goal=1 elapsed_time_s=8.4
```

## MCU 侧伪代码

```c
send_line("START max_time_s=20 control_period_s=0.02\n");
wait_line();  // STARTED

int seq = 0;
int segment_len = 25;
int total_segments = 8;

for (int segment = 0; segment < total_segments; segment++) {
    float left;
    float right;

    if ((segment % 2) == 0) {
        left = 6.0f;
        right = 9.0f;
    } else {
        left = 9.0f;
        right = 6.0f;
    }

    for (int i = 0; i < segment_len; i++) {
        printf_to_serial("STEP %d %.2f %.2f\n", seq, left, right);
        wait_line();  // OBS or FINISHED
        seq++;
    }
}

send_line("STOP s_curve_done\n");
wait_line();  // FINISHED
```

## 你实现 S 型前需要确认的参数

```text
1. 串口号，例如 COM3
2. 波特率，默认 115200
3. control_period_s，建议先用 0.02
4. left_speed / right_speed 的合理范围
5. 每个左转或右转段持续多少帧
6. 是否只做开环，还是根据 yaw / distance_cm 做闭环修正
7. sequence_id 是否从 0 开始逐帧递增
```

最小可运行方案是开环控制：单片机只按固定周期交替发送左右轮差速 `STEP`，收到 `OBS` 后再发下一帧。
