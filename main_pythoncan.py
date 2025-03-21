from python_can_controller import PythonCANController
from mecanum_controller import MecanumController
from keyboard_control import KeyboardControlInterface

def main():
    """主函数"""
    print("麦克纳姆轮键盘控制系统 (Python-CAN版本)")
    print("===================")

    # 创建Python-CAN控制器 (根据实际接口调整)
    can_controller = PythonCANController(
        interface='canalystii',  # 接口类型: 可选'socketcan', 'canalystii'等
        channel=0,               # 通道
        bitrate=500000           # 500K波特率
    )

    if not can_controller.connected:
        print("CAN设备连接失败，程序退出")
        return

    try:
        # 创建麦克纳姆轮控制器 (轮子节点ID: 左前=1, 右前=2, 左后=12, 右后=4)
        mecanum = MecanumController(can_controller, wheel_ids=(1, 2, 12, 4))

        # 创建并运行键盘控制接口
        keyboard_interface = KeyboardControlInterface(mecanum)
        keyboard_interface.run()

    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        # 关闭CAN设备
        can_controller.close()

if __name__ == "__main__":
    main()