#!/usr/bin/env python3
# main_pythoncan_cmd.py - Linux环境下基于Python-CAN的命令行控制程序

from python_can_controller import PythonCANController
from mecanum_controller import MecanumController
from command_interface import CommandInterface

def main():
    """主函数"""
    print("麦克纳姆轮命令行控制系统 (Python-CAN版本)")
    print("===================")

    # 创建Python-CAN控制器
    # 注意：根据你的实际硬件调整接口类型和参数
    can_controller = PythonCANController(
        interface='socketcan',  # Linux下通常使用socketcan接口
        channel='can0',         # CAN接口名称，可能需要提前配置
        bitrate=500000          # 500K波特率
    )

    if not can_controller.connected:
        print("CAN设备连接失败，程序退出")
        return

    try:
        # 创建麦克纳姆轮控制器 (轮子节点ID: 左前=1, 右前=2, 左后=12, 右后=4)
        # 注意：根据实际电机ID配置调整
        mecanum = MecanumController(can_controller, wheel_ids=(1, 2, 12, 4))

        # 创建并运行命令接口
        cmd_interface = CommandInterface(mecanum)
        cmd_interface.run()

    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        # 关闭CAN设备
        can_controller.close()

if __name__ == "__main__":
    main()