# 麦克纳姆轮车辆控制系统 🤖

## 📂 项目结构概览

项目包含两个主要目录：
- `mecanum_control` - 核心控制代码
- `tests` - CAN接口测试工具

## 🔧 核心组件

### ZLGCANController (`zlg_can_controller.py`)
- 封装ZLG USBCAN硬件接口，提供CAN通信功能 📡
- 使用Windows DLL (`ControlCAN.dll`) 与硬件通信
- 管理连接、低级消息格式化和收发

### MotorController (`motor_controller.py`)
- 通过CAN命令控制单个电机 ⚙️
- 提供初始化、使能/禁用和设置电机速度的方法
- 使用CiA 402协议（常用于伺服驱动器）

### TrajectoryPlanner (`trajectory_planner.py`)
- 生成梯形速度曲线实现平滑运动 📈
- 创建基于时间的速度曲线，包含加速、匀速和减速阶段

### MecanumController (`mecanum_controller.py`)
- 管理麦克纳姆轮系统的四个电机 🚗
- 将高级运动命令（前进、后退、左移、右移、旋转）映射到个别轮子速度
- 使用TrajectoryPlanner执行运动轨迹

### CommandInterface (`command_interface.py`)
- 提供基于文本的用户界面 💻
- 解析用户命令并发送到MecanumController
- 显示帮助信息并处理用户输入/输出

## 🚀 main.py使用指南

`main.py`是应用程序的入口点，以下是它的执行流程和使用方法：

```python
def main():
    # 创建CAN控制器
    can_controller = ZLGCANController(
        device_index=0,           # 第一个ZLG USBCAN设备
        channel=0,                # 设备上的第一个通道
        bitrate=500000,           # 500K波特率
        dll_path='./ControlCAN.dll'  # 控制库路径
    )
    
    # 创建麦克纳姆轮控制器 (轮子节点ID: 左前=1, 右前=2, 左后=12, 右后=4)
    mecanum = MecanumController(can_controller, wheel_ids=(1, 2, 12, 4))
    
    # 创建并运行命令接口
    cmd_interface = CommandInterface(mecanum)
    cmd_interface.run()
```

### 使用步骤 📝

1. **准备硬件** 🔌
   - 确保ZLG USBCAN设备已连接到计算机
   - 确保电机已连接到CAN总线，并且节点ID正确设置

2. **运行程序** ▶️
   - 在项目目录下运行 `python main.py`
   - 程序将显示帮助菜单，列出可用命令

3. **输入命令** ⌨️
   - 命令格式：`<运动模式> <加速时间> <匀速时间> <减速时间> <目标RPM>`
   - 例如：`w 2 5 2 10` 表示前进，加速2秒，匀速5秒，减速2秒，目标速度10 RPM

4. **可用运动命令** 🎮
   - `w` - 前进 ⬆️
   - `s` - 后退 ⬇️
   - `a` - 左移 ⬅️
   - `d` - 右移 ➡️
   - `q` - 逆时针旋转 🔄
   - `e` - 顺时针旋转 🔄
   - `h` - 显示帮助 ❓
   - `x` - 退出程序 🚪

5. **安全退出** 🛑
   - 输入 `x` 命令安全退出程序
   - 程序会自动停止所有电机并关闭CAN设备

### 命令实例 📋

- `w 2 5 2 10` - 前进，加速2秒，匀速5秒，减速2秒，目标10 RPM
- `s 1 3 1 15` - 后退，加速1秒，匀速3秒，减速1秒，目标15 RPM
- `a 0.5 2 0.5 8` - 左移，加速0.5秒，匀速2秒，减速0.5秒，目标8 RPM
- `q 1 4 1 5` - 逆时针旋转，加速1秒，匀速4秒，减速1秒，目标5 RPM

## 🔍 测试工具

- `test_can_usb_device.py` - 测试ZLG USBCAN设备连接 🧪
- `can_device_scanner.py` - 扫描CAN总线上的连接设备 🔎
- `test_simple_can.py` - 测试单个电机控制 ⚙️

## 💡 实现细节

1. 使用梯形速度曲线实现平滑运动控制 📈
2. 命令格式包含加速、匀速和减速时间，提供精确运动控制 ⏱️
3. 系统设计用于麦克纳姆轮，支持全方位移动 🚗
4. 实现了错误处理和清理机制，安全处理硬件故障 🛡️
5. main.py中的轮子ID设置为(1, 2, 12, 4)，这可能是特定于硬件设置的 🔢

## ⚠️ 注意事项

1. 确保`ControlCAN.dll`文件位于当前目录下
2. 确保CAN总线波特率设置正确（默认500Kbps）
3. 确保轮子节点ID正确对应实际硬件连接
4. 首次使用前可以运行测试工具验证硬件连接
