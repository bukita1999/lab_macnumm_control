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


**启动说明:**

为了方便使用新的 'actual' 驱动模式，已创建了专门的 launch 文件：

1.  **启动单独的 'actual' 驱动控制器节点:**

    ```bash
    roslaunch mecanum_control mecanum_controller_actual.launch
    ```
    这个 launch 文件 (`launch/mecanum_controller_actual.launch`) 会加载默认参数并启动 `mecanum_controller_node.py`，同时将 `driver_type` 设置为 `actual`。

2.  **启动包含 'actual' 驱动的完整系统 (控制器 + 命令接口):**

    ```bash
    roslaunch mecanum_control full_system_actual.launch
    ```
    这个 launch 文件 (`launch/full_system_actual.launch`) 会包含 `mecanum_controller_actual.launch` 和 `command_interface.launch`，启动控制器和命令接口节点。

**重要:**

*   请确保 `config/default_params.yaml` 文件中包含了 'actual' 驱动模式所需的运动学参数 (`kinematics/wheel_radius`, `kinematics/wheel_separation_width`, `kinematics/wheel_separation_length`) 以及正确的 CAN 配置 (`can_interface`, `can_channel`, `can_bitrate`)。
*   如果需要覆盖默认参数，可以直接修改 `mecanum_controller_actual.launch` 文件，或在启动时通过命令行参数传递，例如：
    ```bash
    roslaunch mecanum_control mecanum_controller_actual.launch can_channel:=can1 kinematics/wheel_radius:=0.06
