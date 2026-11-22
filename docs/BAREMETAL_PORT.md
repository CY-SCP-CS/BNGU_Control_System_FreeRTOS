# 控制功能与裸机工程对应说明

参考工程为 `D:/STM32project/BNGU_Control_System`，参考提交 `a9a9647`。
本工程保留 App/Bsp/Drv/Lib/Os、既有任务编号和通信对象，算法在任务中运行。

## 功能归属

| 功能 | FreeRTOS 位置 | 运行与边界 |
|---|---|---|
| 双 yaw / pitch、坐标变换、PID | App/Vehicles/Sentry/Gimbal | 云台 1 ms；输入 SI，内部适配保留原增益与机械零位 |
| 摩擦轮、拨弹与前馈 | App/Vehicles/Sentry/Gimbal/Shoot | 5 ms；拨弹 PID 85/150/0，前馈 ±3000，急停复位 |
| 遥控开关 | App/Modules/dbus | S2 上 1=DBUS，中 3=CAN，下 2=急停；S1 为发射模式 |
| 反馈与板间指令 | App/Modules/can | 仍由 CAN 任务独占发送；不从旧算法直接操作 HAL |
| Mahony/EKF/VQF、安装补偿 | Drv/Drv_imu | 默认 VQF；TIM14 1 kHz 触发，SPI DMA 完成后 IMU 任务解算 |
| IST8310 磁力计 | Drv/Drv_mag、Bsp/bsp_i2c、bsp_gpio | EXTI 仅记录；IMU 任务推进异步 I2C 状态机与恢复 |
| 四全向轮导航 | App/Vehicles/Navigation/Chassis | 独立导航任务 1 ms 调度，每 5 次执行一次原 5 ms 控制 |
| 麦轮、关节框架 | App/Vehicles/Armored/Chassis | 反馈注入、结果复制/回调输出；未自动启动整车控制 |
| 裁判 UI | App/Modules/ui | 可选；裁判任务每 40 ms 最多一帧，UART6 中断发送，忙时保留脏标志 |
| 达妙电机协议 | Drv/Drv_motor | 控制、寄存器、保存参数构帧及反馈解析；实例分别存储 |
| 运行诊断 | App/Modules/health、log | 保留 RTOS 心跳、漏周期、栈/堆日志；磁力计加入在线表 |

车型、板型及完成程度见 [车型边界](VEHICLE_BOUNDARIES.md)。

## 选择配置

```powershell
cmake --preset GIMBAL_TEXT
cmake --build --preset GIMBAL_TEXT
python tools/sync_keil_project.py --preset GIMBAL_TEXT
```

哨兵四种配置为 GIMBAL_TEXT、GIMBAL_VOFA、CHASSIS_TEXT、CHASSIS_VOFA。
导航使用 NAVIGATION_TEXT 或 NAVIGATION_VOFA，选择底盘板；云台板使用 NAVIGATION_GIMBAL_TEXT/VOFA，业务控制待实现。底盘板启用 DBUS 与 IMU。
UI 使用 CHASSIS_UI_REFEREE 或 CHASSIS_UI_DEMO，默认其他配置关闭。
DEMO 是明确的模拟演示数据，不可当成真实机构状态。
Keil 缺失的 RVDS/ARM_CM4F/port.c 与 portmacro.h 已从本机 STM32Cube F4 V1.28.3 同版本固件包补齐，tasks.c 校验一致。当前未执行 Keil 编译器构建。
Keil 同步必须在所选预设配置成功之后执行，保持 RVDS FreeRTOS port；它会同步车组、板型、日志、UI 宏与 I2C3 源文件。

`project_cfg.h` 可覆盖 IMU_AHRS_ALGORITHM（DRV_IMU_ALGO_MAHONY / DRV_IMU_ALGO_EKF / DRV_IMU_ALGO_VQF）。
哨兵安装姿态为 (0,-90,0) 度，导航为 (0,0,0)；修改安装角必须同时核对实物与控制轴。
IMU 快照加速度为传感器系 m/s²，陀螺为校正后的机体系 rad/s，四元数为机体系到世界系。
欧拉角沿用哨兵 XYZ / 导航 ZYX 驱动约定；云台融合 pitch 关节后重建控制四元数。
磁力计为独立测量/健康数据（业务接口 T），没有自动融合成九轴 AHRS。

## 协议与波形

CAN1 每 5 ms 云台反馈 0x122（角速度 rad/s）、0x124（角度 rad）、0x233（控制四元数×30000）、0x139（yaw/pitch float 度，小端）。
0x139 使用参考代码实际的 CAN1，而不是参考提交说明中的 CAN2。
0x120 到 0x111 保留参考工程的云台到底盘转换：vx'=cos(yaw)vx+sin(yaw)vy，vy'=sin(yaw)vx-cos(yaw)vy。
不要将第二项擅自改成另一种常规旋转公式，它包含原坐标约定。
哨兵电流 ID 与急停保留；导航 CAN2 反馈 0x201/202/203/204 分别是 FR/RL/RR/FL，控制统一为 0x200 四槽。

VOFA 九通道，36B float 数据加 4B JustFloat 帧尾，总长 40B，UART1 115200，200 Hz。
云台通道依次为：拨弹标称角速度、反馈角速度、PID 积分输出 Ki×积分状态、反馈努力 raw、右摩擦目标、右摩擦反馈、左摩擦目标、左摩擦反馈、拨弹编码器 raw。
角速度为 rad/s；raw 未确认安培换算。底盘保留其八通道映射，第九通道为零。TEXT 模式保留文本内存/栈/堆日志。

## 外设配置

IOC 与生成源码已同步 I2C3（PA8 SCL、PC9 SDA，400 kHz）、PG6 磁力计复位输出、PG3 下降沿数据就绪 EXTI。
I2C3 EV/ER、EXTI3 优先级为 5；保留原 TIM6 HAL 时基与 RTOS 配置。
新 HAL I2C 源及 Core i2c.c 纳入 CMake/Keil。以后重新生成 CubeMX 时核对这些项，不能拿裸机 IOC 覆盖当前 FreeRTOS IOC。

## 原工程的完成程度与使用限制

- 重装 ARMORED 麦轮 PID 原值为零，关节姿态/高度控制未完成；独立集成预设保留算法与输出注入接口，整车控制任务尚未接入。HERO 已删除。
- 导航 game-controller 已接入独立任务的手柄端口；手柄、DBUS、CAN 与轮反馈均检查有效性和时效。世界系控制新增明确的 IMU 有效性/20 ms 新鲜度门控。
- REFEREE UI 使用真实机器人 ID、热量与弹量；当前底盘没有本地发射就绪与云台 pitch 数据，显示未知/FAULT 与 ---，不会伪造 READY。
- 原诊断占位框架不与现有 RTOS 健康任务重复运行；旧 TIM 中断控制调用、裸机 main 轮询和直接 CAN 发送由任务路径接管。
- UI 对接真实裁判客户端、磁力计实物收发、控制效果与实时性需要上板确认。编译和主机测试不能替代硬件验收。

## 验证方法

```powershell
tests/host/run_contracts.ps1 -Compiler gcc
python tests/port/run_tests.py --cc gcc
python tests/imu/run_tests.py
python tests/mag/run_tests.py
```

tests/port 覆盖遥控映射、控制融合角度与四元数、DBUS/CAN 速度变换、发射模式/前馈/抗饱和积分、CAN 布局、VOFA、裁判 CRC 分包、DM 命令及导航控制源/失效门控。
tests/imu 覆盖三算法姿态、安装、动态运动、偏置与异常样本。
tests/mag 验证解析、异步收发完成、错误不覆盖旧值、超时终止/恢复、失效与 tick 回绕。
tests/host/run_tests.ps1 是当前完整主机回归入口，包含架构、启动、车型隔离、控制迁移、IMU、磁力计和遥测测试。

静态栈帧检查中 EKF 修正最大单帧 576B，UI 热量构帧 352B；这不是调用链最大栈，也不包含全部库函数和异常现场。
IMU 栈仍为 1536B、裁判栈 1024B。上板检查各任务至少 20% 余量，记录 60 s 漏周期、采样年龄、队列丢弃、UART 超时和堆最低水位；尤其关注无优化 Debug 下的 VQF/EKF 执行耗时。
