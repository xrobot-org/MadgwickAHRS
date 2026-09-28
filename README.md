# MadgwickAHRS

基于 Madgwick 梯度下降算法的姿态和航向参考系统（AHRS）模块。
An Attitude and Heading Reference System (AHRS) module based on the Madgwick
gradient-descent filter.

模块创建 `ahrs` 线程（`HIGH` 优先级）：每收到一条陀螺仪消息执行一次更新，使用最近收到的
加速度计消息。加速度计向量为零时只积分陀螺仪，不做校正。结果以
`LibXR::Quaternion<float>` 和 `LibXR::EulerAngle<float>`（rad）发布。

The module starts the `ahrs` thread (`HIGH` priority). It runs one update per
gyroscope message, using the most recently received accelerometer message. When the accelerometer vector is zero the gyroscope is only
integrated, without correction. The result is published as
`LibXR::Quaternion<float>` and `LibXR::EulerAngle<float>` (rad).

| Topic | 方向 / Direction | 类型 / Type | 说明 / Meaning |
| --- | --- | --- | --- |
| `imu_gyro` | 订阅 / subscribed | `Eigen::Matrix<float, 3, 1>` | 角速度，rad/s / angular rate, rad/s |
| `imu_accl` | 订阅 / subscribed | `Eigen::Matrix<float, 3, 1>` | 加速度（归一化后使用，单位不限）/ acceleration (normalized, any unit) |
| `ahrs_quaternion` | 发布 / published | `LibXR::Quaternion<float>` | 姿态四元数 / attitude quaternion |
| `ahrs_euler` | 发布 / published | `LibXR::EulerAngle<float>` | 欧拉角，rad / Euler angles, rad |

`OnMonitor()`：四元数出现 NaN 或 Inf 时输出告警。/ Logs a warning when the quaternion contains NaN or Inf.

### 时间戳约定 / Timestamp convention

AHRS 以 gyro topic 作为主时间线。每次姿态更新使用 gyro 消息的 Topic envelope
timestamp 计算 `dt`（第一次更新 `dt = 0`），并用同一个 timestamp 发布
`ahrs_quaternion` 与 `ahrs_euler`。

The gyro topic is the time base. Each update computes `dt` from the Topic
envelope timestamps of consecutive gyro messages (`dt = 0` on the first update)
and publishes `ahrs_quaternion` and `ahrs_euler` with that same timestamp.

### RamFS 命令 / RamFS command

模块在 `ramfs` 中注册命令 `ahrs`。/ The module adds the command `ahrs` to `ramfs`.

```sh
ahrs show <time_ms> <interval_ms>        # 周期打印欧拉角、四元数和 dt / print Euler angles, quaternion and dt
ahrs print_quat <time_ms> <interval_ms>  # 以 VOFA+ 格式打印 w,x,y,z / print w,x,y,z in VOFA+ format
ahrs test                                # 静止 3 s 后测 10 s 航向漂移，输出 °/min / keep still: after 3 s measures yaw drift over 10 s, prints °/min
```

`interval_ms` 被限制在 2..1000。/ `interval_ms` is clamped to 2..1000.

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。陀螺仪和加速度计数据来自发布上述 Topic 的 IMU 模块。
No other Modules; LibXR only. The gyroscope and accelerometer data come from
any IMU Module that publishes the topics above.

## 构造接口 / Constructor

```cpp
struct Param
{
  float beta;
  const char* gyro_topic_name;
  const char* accl_topic_name;
  const char* quaternion_topic_name;
  const char* euler_topic_name;
  uint32_t task_stack_depth;
};

MadgwickAHRS(LibXR::RamFS& ramfs,
             const Param& param = {.beta = 0.05f,
                                   .gyro_topic_name = "imu_gyro",
                                   .accl_topic_name = "imu_accl",
                                   .quaternion_topic_name = "ahrs_quaternion",
                                   .euler_topic_name = "ahrs_euler",
                                   .task_stack_depth = 2048});
```

依赖 / Dependencies:

- `ramfs`：注册 `ahrs` 命令的 RamFS。/ RamFS that receives the `ahrs` command.

配置 / Configuration (`Param`):

- `beta`：Madgwick 滤波增益，默认 0.05。/ Madgwick filter gain, default 0.05.
- `gyro_topic_name`、`accl_topic_name`：订阅的 Topic 名，默认 `imu_gyro`、`imu_accl`。
  / Subscribed topic names, defaults `imu_gyro`, `imu_accl`.
- `quaternion_topic_name`、`euler_topic_name`：发布的 Topic 名，默认 `ahrs_quaternion`、`ahrs_euler`。
  / Published topic names, defaults `ahrs_quaternion`, `ahrs_euler`.
- `task_stack_depth`：线程栈大小，默认 2048。/ Thread stack size, default 2048.

## 使用 / Use

```sh
xrobot module add xrobot-org/MadgwickAHRS
xrobot setup
xrobot instance add xrobot-org/MadgwickAHRS
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `ramfs` 填为 BSP 中用 `XR_REGISTER` 注册的 RamFS 对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set `ramfs` to the name of a RamFS object
the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/MadgwickAHRS
    id: madgwickahrs_0
    args:
      - ramfs: ramfs
      - param:
          beta: 0.05f
          gyro_topic_name: '"imu_gyro"'
          accl_topic_name: '"imu_accl"'
          quaternion_topic_name: '"ahrs_quaternion"'
          euler_topic_name: '"ahrs_euler"'
          task_stack_depth: '2048'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/MadgwickAHRS`
（在 BSP 中）打印当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/MadgwickAHRS` in a BSP, prints the current
constructor.
