# 哨兵 FreeRTOS 架构与开发指南

## 目录与依赖

```text
Self_core/
  App/
    Config/                 板型、周期、任务清单、容量和预算
    Contracts/              业务输入、输出和状态，无 HAL/RTOS/驱动实现类型
    Modules/
      gimbal/ chassis/ shoot/ 控制算法、任务和服务相邻
      can/ dbus/ referee/ imu/ 输入与通信模块
      health/ log/ telemetry/ 诊断服务
      navigation/ armored/ ui/ 导航内核、实验机构与可选裁判 UI
      control_support/      旧算法数值适配与参数
    Assembly/               静态硬件实例、回调连接、初始化
    Runtime/                任务表、启动门控、周期与运行统计
  Bsp/                      HAL 操作与外设事件路由
  Drv/                      设备协议、算法实例和硬件端口
  Lib/                      通用数学、PID、滤波
  Os/                       CMSIS/FreeRTOS 对象、内核、CPU 机制
```

业务通过同步函数计算，任务负责取输入、调度、提交输出。公共服务头不公开队列和互斥量句柄；`*_internal.h` 供本模块、装配和任务注册使用。`app_rtos` 保留既有接口命名，不代表函数都属于 Os。Os 不包含业务类型。

控制内核状态由所属控制任务独占。输入快照锁内只复制，读锁超时为零；读取失败保留上一完整副本并计数。数据过期与读取失败不同，控制保留原来的急停和断线判定。诊断快照失败不影响电机结果；遥测发布锁超时零，失败保留旧值并累计 `COUNT_TELEMETRY_DROP`。

## 调度与交付

哨兵云台、底盘 1ms，发射 5ms；导航底盘任务 1ms 调度、算法 5ms，健康 20ms，控制使用 `osDelayUntil()` 绝对 tick。等待只表示进入就绪，实际运行受抢占影响。IMU 仅 TIM14 触发，DMA 完成后任务解析与姿态计算。初始化完成发布共同起始 tick，失败发布持久失败状态。

| 关系 | 机制 | 内容与失败策略 |
|---|---|---|
| CAN1/2 ISR → CAN | 两个接收队列 | 原始帧值复制，满时丢弃计数 |
| 控制 → CAN | ctrl_to_can_queue | 当前机构结果，非阻塞；满时丢弃，下一周期再提交当前结果 |
| CAN → 主控制 | can_to_ctrl_queue | 指令事件，只有本板主控制消费；增量只在更新周期应用 |
| CAN 编码 → 发送 | 两个发送队列及待发槽 | 现有连续状态按 ID 合并，硬件邮箱忙保留待发帧 |
| DBUS/IMU → 控制 | 各自互斥快照 | 完整归一化输入或 SI 姿态，多读者复制 |
| 云台 → 发射 | dbus_to_shoot_mutex | 已裁决模式及就绪/急停，不再由发射独立裁决输入 |
| 裁判 ISR → 任务 | FIFO + 线程标志 | 连续有序字节，溢出/错误请求重同步 |
| 所有任务 → 日志 | 固定事件队列 | 满时丢新事件并计数，UART1 由日志任务独占 |

队列容量是存储上限；`*_BUDGET` 是每轮工作量，不应通过扩容同时扩大工作预算。新增逐条事件不得套用连续电机状态的覆盖策略。

## 单位与算法适配

业务采用 m、m/s、m/s²、rad、rad/s、s、V、A、W、J、B；温度按工程约定使用 °C。tick 保持整数内核时基，测时 `_us` 不改成浮点秒。四元数和归一化输入无量纲。

| 原表示 | 业务表示 | 转换位置 | 验证 |
|---|---|---|---|
| CAN vx/vy mm/s int16 | m/s float | CAN 解包/编码 | 65536 个原始值逐值回传一致 |
| CAN vz 0.001rad/s | rad/s | CAN 解包/编码 | 有符号范围往返 |
| 功率百分比×100 | 比例，1=100% | CAN 边界 | 0.0001/LSB 往返 |
| 电机编码器 0..8191 | rad | CAN 解包与旧算法适配 | 全编码器范围恢复 |
| 电机 RPM int16 | rad/s | CAN 与旧算法适配 | 全 int16 范围恢复 |
| 功率计 centiV/centiA | V/A/W | CAN 解包 | 原功率公式 V×A，不读取保留字为功率 |
| IMU 加速度 g | m/s² | IMU 发布 | 标准重力 9.80665 |
| IMU 欧拉角 degree | rad | IMU 发布与旧内核适配 | 旋转顺序、零位与正方向不变 |
| DJI 努力量 | raw int16 | 保留协议边界 | 未确认 A 换算，禁止当安培 |

`legacy_control_types.h` 只服务旧内核；旧 PID、几何和滤波尺度保留，输入适配恢复旧数值，输出适配发布 SI。不得在任务中再次乘 1000 或换度。`lib_physical.h` 集中换算和饱和量化。VOFA 显示映射读取具名遥测后产生，旧通道毫米/度显示只发生在输出端。

## 设备实例与 DMA

DBUS 和裁判端口每个实例持有自己的 DMA 缓冲、位置、回调和上下文；裁判解析器也必须单独分配 `drv_referee_context_t`。装配静态分配实例，回调使用 DATA/ERROR/SERVICE 判别，不靠 NULL 推断事件。

裁判 DMA 512B 循环写入。HT/TC/IDLE 读取当前写位置，与上次位置比较：未回绕交付一段，回绕先尾后头。回调将字节复制进 1024B FIFO；解析任务每次取有限片段，线性解析缓冲保留半帧，CRC 失败逐字节重新寻找 A5。完整多圈覆盖无法靠位置差恢复，必须及时响应中断。

IMU 实例持有 ACC/GYRO DMA 缓冲和状态。状态为 IDLE → ACC → GYRO_PENDING/GYRO → IDLE；错误进入 ABORTING，硬件确认停止后释放总线。SPI 每个硬件实例有独立收尾槽，同一 SPI 的完整采样只允许一个拥有者；忙时跳过并计数，不自动排队。DMA 缓冲必须普通 SRAM，不能放 CCM；异步启动成功只代表接受启动，完成前不可复用。

## 新增模块与团队协作

1. 在 `Modules/<功能>/` 放相邻的服务头、任务、算法与端口；私有状态设 static。
2. 先定义契约：所有者、调用者、上下文、复制/借用、单位、时间源、新鲜度和失败策略。生产者与消费者共同评审。
3. 模块 CMake 显式列出 algorithm/runtime/port 源；顶层只组合模块。
4. 新任务追加 `Config/app_task_list.def`，保持已有编号与入口名；配置板型能力、启动门控、栈和优先级。
5. 新通信对象在所属模块 init 创建，deinit 逆序删除；不向其他模块导出句柄。
6. 新设备静态分配端口与解析实例，装配连接明确事件回调；同一 UART 竞争接收者应初始化失败。
7. 新板型先定义能力，再配装配与协议；未实现车型不静默退回默认。

每个功能模块的维护者同时维护接口、实现、测试、CMake 和模块配置。平台维护者负责 Os、构建与规范；本仓库不虚构人员名单，由团队在评审分工时明确。

## 构建与验证

配置/构建使用 GIMBAL_TEXT、GIMBAL_VOFA、CHASSIS_TEXT、CHASSIS_VOFA 四个哨兵 CMake 预设；导航使用 NAVIGATION_TEXT/VOFA，UI 使用 CHASSIS_UI_REFEREE/DEMO。CMake 从实际目标生成 `build/<预设>/self_core_manifest.json`，Keil 同步读取此清单，先配置再运行 `python tools/sync_keil_project.py --preset GIMBAL_TEXT`。不要维护第二份 YAML 源码表。

主机边界回归：`tests/host/run_contracts.ps1 -Compiler <gcc路径>`。格式使用仓库 `.clang-format`。公共 API 的头文件说明是权威文档；私有函数在定义前说明。注释必须有 brief、逐参数方向、返回值与上下文，void 明确无返回值；单位、借用期限与失败语义由评审核对，自动扫描不能证明说明真实。

本机边界测试不能证明整套旧算法等价、真实 DMA 恢复或实时性。上板须对照急停/断线/全部发射模式、CAN 协议、60 秒漏周期、栈和堆水位。硬件验证结果单独记录，不把编译成功写成已验证控制效果。

## 姿态、导航与可选 UI

IMU 默认 VQF，也可配置 Mahony/EKF；安装补偿在驱动输出边界完成。磁力计异步 I2C 由 IMU 任务推进，仅提供独立数据和在线诊断。导航板启用 DBUS 与 IMU；英雄麦轮、关节仍为未完成内核，不自动接入任务。UI 与裁判 RX 共用 USART6 的不同方向，TX 缓冲独立，超时只终止 TX。云台 VOFA 为九通道 40B 帧。详细配置、协议单位、完成程度和测试入口见 [控制功能对应说明](BAREMETAL_PORT.md)。
