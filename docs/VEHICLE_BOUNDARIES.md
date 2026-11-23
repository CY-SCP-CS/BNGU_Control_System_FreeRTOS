# 车型、板型与任务归属

Sentry（哨兵）、Navigation（导航）、Armored（重装）是同级车型，每种车型均有 Chassis 与 Gimbal 两个板级目录。
HERO 已删除。车型选择 APP_ROBOT，板型选择 APP_BOARD；六种组合都可构建。

```text
App/
  Vehicles/
    Sentry/
      Chassis/       双舵轮算法、底盘任务、就绪提示端口
      Gimbal/        云台算法、云台任务；Shoot/ 发射算法与任务
    Navigation/
      Chassis/       四轮算法、导航任务、手柄端口
      Gimbal/        独立云台板边界，业务控制待实现
    Armored/
      Chassis/       麦轮和关节算法，整车任务待实现
      Gimbal/        独立云台板边界，业务控制待实现
  Modules/           公共通信、输入采样、诊断、日志、遥测、显示与命令快照
  Assembly/          选择车型和板型，装配任务与外设
  Runtime/           公共调度支持
  Contracts/         值类型契约
```

| 车型/板型 | 业务任务现状 | 输入与周期 |
|---|---|---|
| Sentry/Chassis | Start_ctrl_chas_Task | CAN 指令与轮反馈；1 ms 控制 |
| Sentry/Gimbal | Start_ctrl_gim_Task、Start_ctrl_shoot_Task | DBUS、IMU、CAN；云台 1 ms，发射 5 ms |
| Navigation/Chassis | Start_navigation_Task | DBUS、IMU、手柄、CAN；1 ms 接收，5 ms 轮控 |
| Navigation/Gimbal | 控制任务待实现 | 只有系统服务，不调用其他车型算法 |
| Armored/Chassis | 整车控制任务待实现 | 麦轮/关节算法独立保留，未装配电机输出 |
| Armored/Gimbal | 控制任务待实现 | 只有系统服务，不调用其他车型算法 |

任务不按车型复制整套 RTOS。每块板有独立固件和任务实例，公共 CAN 收发、健康、日志与初始化服务复用实现。
裁判任务当前由底盘板承担；DBUS/IMU 只在已有真实控制消费者的板上启用。未实现板级控制不会创建空转占位控制任务。
TASK_CHAS、TASK_GIM、TASK_SHOOT 是稳定的任务槽位，实际入口与启用条件由车型和板型共同决定；导航复用底盘槽位而不复用哨兵任务。
发射命令快照属于公共服务，发射控制任务和 PID 属于 Sentry/Gimbal/Shoot。

后续补全导航/重装云台时，云台轴控制归各自 Gimbal，发射机构也归各自云台板；运动与轮/关节控制归各自 Chassis。
板间通过值类型契约和 CAN 消息传递目标、状态与时效信息，不直接包含另一块板的任务私有头或访问控制状态。
每个控制环只由一个任务写入；只有更新周期或实时约束确有差异时再拆任务，不按每个电机增加线程。

重装原始麦轮 PID 为零、关节姿态/高度控制不完整；参考工程没有导航和重装的云台控制实现。
这些目录和固件入口已经建立，但不能把编译成功视为整车功能完成。当前这些未实现路径不发送电机控制帧，HERO 也没有保留为兼容别名。

原有 NAVIGATION_TEXT/VOFA 和 ARMORED_TEXT/VOFA 指向各自底盘板；
新增 NAVIGATION_GIMBAL_TEXT/VOFA、ARMORED_GIMBAL_TEXT/VOFA 指向各自云台板。
哨兵沿用 GIMBAL_TEXT/VOFA、CHASSIS_TEXT/VOFA。

验证：tests/host/run_tests.ps1 检查六种车型/板型的真实任务表、启动失败回滚和输入时效；
tests/architecture/test_vehicle_builds.py 检查编译清单未串用车型控制源码；tools/check_dependencies.py 检查目录迁移后的层级、私有边界和循环依赖。
编译及主机回归不替代实车控制调参与实时性验证。

车型参数统一归各车型的 Config/：

- Vehicles/Sentry/Config/：sentry_control_config.h、sentry_input_config.h、app_sentry_common.h。
- Vehicles/Navigation/Config/：navigation_control_config.h、navigation_input_config.h、Navigation_cfg.h。
- Vehicles/Armored/Config/：armored_control_config.h、armored_input_config.h、Armored_cfg.h。

系统资源容量、任务表与公共调度机制仍由公共配置管理。导航 RX 服务周期和轮控周期分别定义，算法 DT 与任务降频使用同一车型配置来源。

## 文件整合与接口归属（2026-11-22）

公共 App/Config 仅保留 app_vehicle_config.h（选择车型与公共能力查询）、app_task_list.def（统一线程装配清单）、app_rtos_config.h（系统级调度与诊断参数）。
车型控制、输入时效与能力参数仍在各车型 Config。CAN/DBUS/IMU/裁判资源容量及批处理预算放在各自 app_task_*.h；健康数据源清单归 Modules/health。

原 app_can_frame.h 与 app_can_data.h 已真正合为 Modules/can/app_can_data.h，统一声明原始帧、接收指令及发送消息。
该文件属于当前 CAN 服务的数据接口；公共命令定义引用 Modules/can/app_can_commands.h；车型控制模型引用 Vehicles/Sentry/Contracts，不再放在公共 Contracts 中冒充通用业务模型。
哨兵云台、发射、电机角色定义归 Sentry/Contracts；导航输入归 Navigation/Contracts；遥测汇总归 Modules/telemetry/app_ctrl_data.h。
公共 Contracts 保留运动指令、底盘遥测、反馈、遥控、IMU、裁判数据和时效工具。底盘速度、轮状态与遥测类型使用中性名称；遥控输入类型也不再以 gimbal 命名。
当前 CAN 服务仍承载已有哨兵协议和导航兼容速度指令，不能据此认为重装协议已实现。

装配调用链：Core/Src/freertos.c 的 MX_FREERTOS_Init 在启动调度前调用 app_rtos_create_tasks()。
该函数由 Assembly/app_rtos_setup.h 声明，app_rtos_setup.c 创建通信对象与线程，失败则终止已创建线程并回滚资源。
调度开始后，一次性初始化任务再推进板级初始化、端口启动、校准和采样等阶段。
原 app_devices.c 的外设实例选择、UART 启动和失败停止已并入 app_rtos_board.c，作为 static 函数使用。
因此 setup 负责线程/通信对象，board 负责 STM32 外设和算法的绑定。

三个哨兵 *_adapter.c 已并入各自算法文件，保留原 SI 转换、时效门控和计算顺序；公共入口集中在文件后部。
重装 app_armored_io.c 的输出绑定并入 app_Armored_joint.c。
任务、纯算法和涉及 DMA/HAL 的端口仍分开，避免把中断接收、硬件恢复与控制计算混为一个文件。

新版核心总线适配及未定项见 [CORE_CAN_ADAPTATION.md](CORE_CAN_ADAPTATION.md)。

## 函数可见性收口（2026-11-23，仅哨兵与公共框架）

跨模块服务在公共头声明；模块内部跨 .c 接口在私有头声明；仅本 .c 使用的函数在实现中定义为 static，不在头文件暴露。
三个哨兵 legacy_app_sentry_* 控制入口在 adapter 合并后已经只由本文件调用，现改为 static，并移除 app_chassis_legacy.h、app_gimbal_legacy.h、app_shoot_legacy.h。
保留公共初始化与 SI 控制入口。任务入口、跨文件 ISR 接口、宏展开调用的 UI 发送接口和明确预留的公共服务 API 不按单纯的文本调用次数改成 static。
本次没有修改重装与导航车型实现或接口。
通过 ARM 目标文件 nm 符号表核验：三个旧入口为本地 t，三个公共控制入口仍为全局 T。
