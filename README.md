# 麦克纳姆轮车辆控制系统 🚗

## 项目概述

这是一个基于Python开发的麦克纳姆轮车辆控制系统，通过ZLG USBCAN设备与电机驱动器通信，实现全方位移动控制。系统支持多种控制模式，包括命令行控制和键盘实时控制，并提供平滑的运动轨迹规划。

## 系统特性

- ✅ 全方位移动：前进、后退、左移、右移、旋转
- ✅ 平滑运动控制：使用梯形速度曲线实现平稳加减速
- ✅ 多种控制模式：命令行接口和实时键盘控制
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

### 用户接口

1. **CommandInterface** (`command_interface.py`)
   - 提供基于命令行的用户交互界面
   - 解析用户输入并发送给控制系统

2. **KeyboardControlInterface** (`keyboard_control.py`)
   - 提供实时键盘控制功能
   - 支持同时按键的组合运动和渐进加速

### 入口程序

- **main.py** - 基于命令行界面的主程序
- **main_keyboard.py** - 基于键盘控制的主程序
- **main_linux.py** - Linux环境下的主程序版本

### 测试工具

- **test_can_usb_device.py** - 测试ZLG USBCAN设备连接
- **can_device_scanner.py** - 扫描CAN总线上的设备
- **test_simple_can.py** - 测试单个电机控制

## 使用指南

### 安装要求

- Python 3.6+
- ZLG USBCAN设备及驱动
- Windows或Linux系统
- 对于键盘控制模式，需要安装`keyboard`库: `pip install keyboard`

### 快速开始

1. 确保硬件连接正确：
   - ZLG USBCAN设备已连接到计算机
   - 电机驱动器已连接到CAN总线
   - 电源供应正常

2. 选择合适的程序入口：
   - 命令行控制: `python main.py`
   - 键盘实时控制: `python main_keyboard.py`
   - Linux环境: `python main_linux.py`

### 命令行控制模式

命令格式: `<运动模式> <加速时间> <匀速时间> <减速时间> <目标RPM>`

例如:
- `w 2 5 2 10` - 前进，加速2秒，匀速5秒，减速2秒，速度10 RPM
- `a 0.5 2 0.5 8` - 左移，加速0.5秒，匀速2秒，减速0.5秒，速度8 RPM

控制命令:
- `w` - 前进
- `s` - 后退
- `a` - 左移
- `d` - 右移
- `q` - 逆时针旋转
- `e` - 顺时针旋转
- `h` - 显示帮助
- `x` - 退出程序

### 键盘实时控制模式

控制键:
- `W` - 前进 (按住加速)
- `S` - 后退 (按住加速)
- `A` - 左移 (按住加速)
- `D` - 右移 (按住加速)
- `Q` - 逆时针旋转 (按住加速)
- `E` - 顺时针旋转 (按住加速)
- `X` - 停止并退出

参数调节:
- `+/=` - 增加最大速度
- `-/_` - 减少最大速度
- `[` - 减小加速度
- `]` - 增加加速度
- `;` - 减小减速度
- `'` - 增加减速度
- `V` - 切换速度显示

## 硬件配置

系统默认配置:
- CAN总线波特率: 500Kbps
- ZLG USBCAN设备索引: 0
- CAN通道: 0
- 轮子节点ID: 左前=1, 右前=2, 左后=12, 右后=4

## 开发指南

### 添加新的运动模式

1. 在`mecanum_controller.py`中的`MOTION_MODES`字典添加新模式
2. 在`get_wheel_speeds`方法中添加对应的速度计算逻辑
3. 更新命令接口中的帮助信息

### 自定义硬件配置

如需更改硬件配置:
1. 修改`main.py`中的`ZLGCANController`和`MecanumController`初始化参数
2. 根据实际硬件调整电机ID和波特率
3. 对于Linux版本，修改`main_linux.py`中的库路径为`libcontrolcan.so`

### 速度计算

电机速度从RPM转换为内部命令值的公式:
```
counts_per_rev = 10000  # 2500线 * 4倍频
counts_per_sec = rpm * counts_per_rev / 60
cmd_value = counts_per_sec / 0.1
```

## 故障排除

### 连接问题

- 确保`ControlCAN.dll`(Windows)或`libcontrolcan.so`(Linux)在当前目录下
- 检查USB连接和驱动程序安装状态
- 验证设备在设备管理器中正常显示

### 通信问题

- 确认波特率设置与总线一致(默认500Kbps)
- 检查CAN总线终端电阻
- 使用测试工具(`can_device_scanner.py`)扫描和确认设备在线

### 电机不响应

- 检查电源连接
- 确认节点ID设置正确
- 使用`test_simple_can.py`测试单个电机控制

## 项目扩展

本项目可进一步扩展:
- 集成传感器数据用于闭环控制
- 添加自动障碍物避免功能
- 开发基于ROS的接口
- 实现路径规划和自主导航
- 开发Web或移动端远程控制界面

## 注意事项

- 首次使用前，建议先运行测试工具确认设备连接正常
- 电机初始控制速度应设置为较低值，确保安全
- 确保运动区域内无障碍物和人员
- 使用前检查所有电气连接，确保系统安全

---

*开发者联系方式: 请发送电子邮件至...*