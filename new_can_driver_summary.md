# 新 CAN 驱动程序修改记录和说明

**目标:** 为新的 CAN 总线控制机器人创建并行的驱动程序，同时保持上层 ROS 接口不变。

**主要修改:**

1.  创建 `scripts/actual_robot_motor_controller.py`: 实现与新机器人 CAN 协议的通信，包括发送整车运动指令 (Vx, Vy, Vw) 和解析状态反馈。
2.  修改 `scripts/mecanum_controller_node.py`:
    *   添加 `driver_type` 参数，支持 'original' (原有驱动) 和 'actual' (新驱动) 两种模式。
    *   根据 `driver_type` 条件化地初始化 `MecanumController` 或 `ActualRobotMotorController`。
    *   为 'actual' 驱动添加了 `MecanumCommand` 到整车速度 (Vx, Vy, Vw) 的转换逻辑。
    *   为 'actual' 驱动添加了从新协议状态反馈到 `MecanumStatus` 消息的转换逻辑（包括轮速估算）。
3.  创建 `scripts/robot_self_check.py`: 创建独立的 Python 脚本，用于直接通过 `python-can` 库对机器人进行自检。

**原有 CAN 格式 (基于 CANopen):**

*   **控制目标:** 单个轮子电机。
*   **通信方式:** CANopen 协议，使用 SDO (Service Data Object) 消息进行配置和控制。
*   **CAN ID:**
    *   NMT 命令: `0x000`
    *   SDO 命令: `0x600 + node_id` (每个电机节点 ID 不同)
*   **数据格式:** 小端序 (Little-Endian)。
*   **控制流程:**
    1.  `MecanumController` 计算每个轮子的目标速度 (RPM)。
    2.  `MotorController` 将 RPM 转换为 CANopen SDO 消息，发送给对应的电机节点 ID。

**新的 CAN 格式 (自定义协议):**

*   **控制目标:** 整车运动。
*   **通信方式:** 自定义 CAN 协议，直接发送整车速度指令 (Vx, Vy, Vw) 并接收整车状态。
*   **CAN ID:**
    *   `0x75a`: 遥控器 -> 主控 (按键和脉冲)
    *   `0x7a5`: 遥控器 -> 主控 (运动速度指令 Vx, Vy, Vw, Vu)
    *   `0x65a`: 主控 -> 外部/遥控器 (基本状态 Soc, V, Type, UDSta)
    *   `0x65b`: 主控 -> 外部/遥控器 (故障、压力、安全触边 Error, Press, In)
    *   `0x65c`: 主控 -> 外部/遥控器 (车辆姿态和平台高度 Acx, Acy, H)
    *   `0x65d`: 主控 -> 外部/遥控器 (平台姿态和承重 Apx, Apy, W)
    *   `0x65e`: 主控 -> 外部/遥控器 (平台侧倾标定状态)
    *   `0x65f`: 主控 -> 外部/遥控器 (平台俯仰标定状态)
*   **数据格式:** 大端序 (Big-Endian)。
*   **控制流程:**
    1.  上层将运动指令转换为整车速度 (Vx, Vy, Vw)。
    2.  `ActualRobotMotorController` 将速度值打包成 CAN 数据帧，发送到 `0x7a5`。
    3.  `ActualRobotMotorController` 监听状态反馈 CAN ID，解析数据并提供给上层。

**总结:**

本次修改的关键在于创建了一个并行的驱动程序，通过 `driver_type` 参数进行切换，并使用转换层来适配新旧 CAN 协议的差异，从而实现了对新机器人的控制，同时保持了原有 ROS 接口的兼容性。