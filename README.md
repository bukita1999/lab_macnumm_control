# 麦克纳姆轮车辆控制系统 🚗

## 项目概述

这是一个基于Python开发的麦克纳姆轮车辆控制系统，通过ZLG USBCAN设备与电机驱动器通信，实现全方位移动控制。系统支持多种控制模式，包括命令行控制和键盘实时控制，并提供平滑的运动轨迹规划。项目深度集成ROS（Robot Operating System）框架，支持ROS节点通信和系统配置。

## 系统特性

- ✅ 全方位移动：前进、后退、左移、右移、旋转
- ✅ 平滑运动控制：使用梯形速度曲线实现平稳加减速
- ✅ 多种控制模式：ROS节点控制、命令行接口和键盘实时控制
- ✅ 精确速度控制：支持精确的电机RPM设置
- ✅ 错误处理机制：提供完整的异常处理和安全关闭功能
- ✅ 跨平台支持：提供Windows和Linux版本

## 系统架构

项目采用模块化设计，主要组件包括：

### 核心组件
1. **ZLGCANController** (`zlg_can_controller.py`)  
   - 封装ZLG USBCAN硬件接口，处理底层CAN通信
   - 支持消息发送和接收，错误处理和设备管理

2. **MotorController** (`motor_controller.py`)  
   - 管理单个电机的控制功能
   - 实现基于CiA 402协议的电机初始化、使能和速度控制

3. **TrajectoryPlanner** (`trajectory_planner.py`)  
   - 生成平滑的梯形速度曲线
   - 提供基于时间的运动规划

4. **MecanumController** (`mecanum_controller.py`)  
   - 协调四个麦克纳姆轮电机的控制
   - 将高级运动命令转换为各轮速度指令

### ROS集成组件
1. **ROS节点**  
   - **command_interface_node**：处理命令行输入并发布`MecanumCommand`消息  
   - **keyboard_control_node**：处理键盘输入并发布控制指令  
   - **mecanum_controller_node**：订阅`MecanumCommand`消息，生成电机控制指令  

2. **ROS消息接口**  
   - **MecanumCommand.msg**：定义运动指令格式（X/Y速度、角速度）  
   - **MecanumStatus.msg**：返回电机状态和系统信息  

### 用户接口
1. **CommandInterface** (`command_interface.py`)  
   - 提供基于命令行的用户交互界面  
   - 解析用户输入并发送给控制系统  

2. **KeyboardControlInterface** (`keyboard_control.py`)  
   - 提供实时键盘控制功能  
   - 支持同时按键的组合运动和渐进加速  

### 入口程序
- **run_keyboard_control_command.sh** - 在Ubuntu上运行按键控制代码  
- **main.py** - 基于命令行界面的主程序  
- **main_keyboard.py** - 基于键盘控制的主程序  

### 测试工具
- **test_can_usb_device.py** - 测试ZLG USBCAN设备连接  
- **can_device_scanner.py** - 扫描CAN总线上的设备  
- **test_simple_can.py** - 测试单个电机控制  

---

## 使用指南

### 安装要求
- **ROS环境**  
  - ROS Noetic或Melodic（根据系统版本选择）  
  - 安装ROS依赖：`rosdep install --from-paths . --ignore-src -r -y`  
  - 必要ROS包：`ros-<distro>-std-msgs`, `ros-<distro>-message-generation`  

- **硬件与软件**  
  - Python 3.6+  
  - ZLG USBCAN设备及驱动  
  - Windows或Linux系统  
  - 键盘控制模式依赖：`pip install keyboard`  

---

### ROS启动方式
1. **运行完整系统**  
   ```bash
   roslaunch lab_macnumm_control full_system.launch
   ```

2. **单独启动命令行接口**  
   ```bash
   roslaunch lab_macnumm_control command_interface.launch
   ```

3. **运行键盘控制模式**  
   ```bash
   roslaunch lab_macnumm_control keyboard_control.launch
   ```

---

### ROS消息接口说明
#### MecanumCommand
```proto
std_msgs/Header header
float64 x_velocity  # X轴速度 (m/s)
float64 y_velocity  # Y轴速度 (m/s)
float64 angular_velocity  # 角速度 (rad/s)
```

#### MecanumStatus
```proto
std_msgs/Header header
float64[] wheel_speeds  # 四个轮子的当前速度 (RPM)
bool is_enabled         # 控制器使能状态
string status_message   # 状态信息
```

---

### 快速开始
1. **硬件连接**  
   - ZLG USBCAN设备已连接到计算机  
   - 电机驱动器已连接到CAN总线  
   - 电源供应正常  

2. **ROS环境配置**  
   ```bash
   source /opt/ros/noetic/setup.bash  # 根据ROS版本调整路径
   cd lab_macnumm_control
   catkin_make  # 若需要重新编译
   ```

3. **启动系统**  
   ```bash
   roslaunch lab_macnumm_control full_system.launch
   ```

4. **使用键盘控制**  
   - 按 `W/A/S/D/Q/E` 控制移动方向  
   - 按 `X` 退出程序  
   - 使用 `+/=/-/_/[ ]/;/'` 调节速度和加速度  

---

## 硬件配置
- **CAN总线参数**  
  - 波特率: 500Kbps  
  - ZLG USBCAN设备索引: 0  
  - CAN通道: 0  
  - 轮子节点ID: 左前=1, 右前=2, 左后=12, 右后=4  

- **ROS参数配置**  
  - CAN设备参数可通过ROS参数服务器动态配置（参考`config/default_params.yaml`）
