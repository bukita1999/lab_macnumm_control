#!/usr/bin/env python3
# zlg_motor_test_windows.py - 使用ZLG USBCAN进行电机速度控制测试 (Windows版)

from ctypes import *
import time
import os
import sys

# ZLG CAN设备类型定义
VCI_USBCAN2 = 4
STATUS_OK = 1

# 结构体定义
class VCI_INIT_CONFIG(Structure):  
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]  

class VCI_CAN_OBJ(Structure):  
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)
                ] 

class MotorSpeedController:
    """使用ZLG USBCAN控制电机速度的类"""
    
    def __init__(self, dll_path='./ControlCAN.dll'):
        """初始化控制器"""
        self.canDLL = None
        self.connected = False
        
        try:
            # 检查库文件是否存在
            if not os.path.exists(dll_path):
                print(f"错误: {dll_path} 文件不存在")
                return
                
            # 加载动态库 (Windows用windll)
            self.canDLL = windll.LoadLibrary(dll_path)
            print(f"成功加载 {dll_path}")
            
            # 打开设备
            ret = self.canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
            if ret != STATUS_OK:
                print("打开USBCAN设备失败")
                return
                
            # 配置CAN通道(500K波特率)
            vci_config = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x1C, 0)
            
            # 初始化并启动通道0
            ret = self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_config))
            if ret != STATUS_OK:
                print("初始化CAN通道0失败")
                self.canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
                return
                
            ret = self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
            if ret != STATUS_OK:
                print("启动CAN通道0失败")
                self.canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
                return
                
            self.connected = True
            print("USBCAN设备初始化成功")
            
        except Exception as e:
            print(f"初始化失败: {e}")
    
    def send_can_message(self, id, data):
        """发送CAN消息并打印十六进制表示"""
        if not self.connected:
            print("未连接到CAN设备")
            return False
            
        try:
            # 准备数据数组
            data_len = len(data)
            data_array = (c_ubyte * 8)(*data)
            reserved_array = (c_ubyte * 3)(0, 0, 0)
            
            # 创建消息对象
            msg = VCI_CAN_OBJ(
                ID=id,
                TimeStamp=0,
                TimeFlag=0,
                SendType=1,            # 单次发送
                RemoteFlag=0,          # 数据帧
                ExternFlag=0,          # 标准帧
                DataLen=data_len,
                Data=data_array,
                Reserved=reserved_array
            )
            
            # 打印十六进制消息内容
            hex_data = ' '.join([f"{b:02X}" for b in data])
            print(f"发送CAN消息: ID=0x{id:X}, 数据=[{hex_data}]")
            
            # 发送消息
            ret = self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(msg), 1)
            if ret == STATUS_OK:
                print("✓ 发送成功")
            else:
                print("✗ 发送失败")
            return ret == STATUS_OK
            
        except Exception as e:
            print(f"发送消息失败: {e}")
            return False
    
    def set_operation_mode(self, node_id):
        """设置节点为操作模式"""
        print(f"设置节点 {node_id} 为操作模式...")
        return self.send_can_message(0x000, [0x01, node_id])
    
    def set_speed_mode(self, node_id):
        """设置为速度控制模式"""
        print(f"设置节点 {node_id} 为速度控制模式...")
        return self.send_can_message(0x600 + node_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
    
    def enable_drive(self, node_id):
        """使能驱动器"""
        print(f"使能节点 {node_id} 的驱动器...")
        return self.send_can_message(0x600 + node_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
    
    def set_speed(self, node_id, rpm):
        """设置电机转速"""
        # 将RPM转换为命令值
        counts_per_rev = 10000  # 2500线 * 4倍频
        counts_per_sec = rpm * counts_per_rev / 60
        cmd_value = int(counts_per_sec / 0.1)
        
        # 转换为字节数组（小端序）
        cmd_bytes = list(cmd_value.to_bytes(4, byteorder='little', signed=True))
        
        # 构造速度命令
        data = [0x23, 0xFF, 0x60, 0x00] + cmd_bytes
        
        print(f"设置节点 {node_id} 的速度为 {rpm} RPM (命令值: {cmd_value})...")
        return self.send_can_message(0x600 + node_id, data)
    
    def stop_drive(self, node_id):
        """停止驱动器"""
        print(f"停止节点 {node_id} 的驱动器...")
        return self.send_can_message(0x600 + node_id, [0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    def close(self):
        """关闭设备"""
        if self.connected and self.canDLL:
            self.canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
            self.connected = False
            print("CAN设备已关闭")

def test_motor_speed(node_id=1, rpm=10, duration=5):
    """测试电机速度控制"""
    controller = MotorSpeedController()
    
    if not controller.connected:
        print("CAN设备初始化失败，无法进行测试")
        return
    
    try:
        print(f"\n开始对节点 {node_id} 进行速度控制测试:")
        print(f"目标速度: {rpm} RPM, 持续时间: {duration} 秒")
        print("-" * 60)
        
        # 初始化电机
        controller.set_operation_mode(node_id)
        time.sleep(0.1)
        
        controller.set_speed_mode(node_id)
        time.sleep(0.1)
        
        controller.enable_drive(node_id)
        time.sleep(0.1)
        
        # 设置速度并运行
        print(f"\n开始运行电机 {rpm} RPM...")
        if not controller.set_speed(node_id, rpm):
            print("设置速度失败")
            return
        
        # 倒计时
        for i in range(duration, 0, -1):
            print(f"运行中... 剩余 {i} 秒", end="\r")
            time.sleep(1)
        
        # 停止电机
        print("\n停止电机...")
        controller.stop_drive(node_id)
        
        print("\n速度控制测试完成!")
        
    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
        controller.stop_drive(node_id)
    except Exception as e:
        print(f"\n测试出错: {e}")
        controller.stop_drive(node_id)
    finally:
        controller.close()

if __name__ == "__main__":
    # 获取命令行参数
    if len(sys.argv) > 1:
        node_id = int(sys.argv[1])
    else:
        node_id = 12  # 默认节点ID
        
    if len(sys.argv) > 2:
        rpm = float(sys.argv[2])
    else:
        rpm = 1000.0  # 默认速度
        
    if len(sys.argv) > 3:
        duration = int(sys.argv[3])
    else:
        duration = 5  # 默认持续时间
    
    print("=" * 60)
    print("ZLG USBCAN 电机速度控制测试 (Windows版)")
    print("=" * 60)
    print(f"节点ID: {node_id}, 目标转速: {rpm} RPM, 持续时间: {duration}秒")
    print("-" * 60)
    
    test_motor_speed(node_id, rpm, duration)