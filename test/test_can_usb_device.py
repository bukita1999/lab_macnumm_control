#!/usr/bin/env python3
# zlg_can_test.py - 测试ZLG USBCAN设备连接

from ctypes import *
import time
import os

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

# 结构体数组类
class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [('SIZE', c_uint16), ('STRUCT_ARRAY', POINTER(VCI_CAN_OBJ))]

    def __init__(self, num_of_structs):
        self.STRUCT_ARRAY = cast((VCI_CAN_OBJ * num_of_structs)(), POINTER(VCI_CAN_OBJ))
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]

def test_zlg_can_connection():
    """测试ZLG USBCAN设备连接"""
    print("ZLG USBCAN设备连接测试")
    print("=======================")
    
    # 加载动态库
    try:
        # 尝试加载库文件
        dll_path = './ControlCAN.dll'
        if not os.path.exists(dll_path):
            print(f"错误: {dll_path} 文件不存在")
            print("请确保libcontrolcan.so文件放在当前目录下")
            return False
            
        canDLL = cdll.LoadLibrary(dll_path)
        print(f"成功加载 {dll_path}")
    except Exception as e:
        print(f"加载动态库失败: {e}")
        return False
    
    # 打开设备
    ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
    if ret == STATUS_OK:
        print('✅ 成功打开USBCAN设备')
    else:
        print('❌ 打开USBCAN设备失败')
        return False
    
    # 配置波特率为250Kbps (根据您的需求可修改timing0和timing1值)
    # 波特率对照表(timing0, timing1):
    # 1000K: 0x00, 0x14
    # 800K:  0x00, 0x16
    # 500K:  0x00, 0x1C
    # 250K:  0x01, 0x1C
    # 125K:  0x03, 0x1C
    # 100K:  0x04, 0x1C
    # 50K:   0x09, 0x1C
    vci_config = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x01, 0x1C, 0)  # 250K波特率
    
    try:
        # 初始化通道0
        ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_config))
        if ret == STATUS_OK:
            print('✅ 成功初始化CAN通道0')
        else:
            print('❌ 初始化CAN通道0失败')
            canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
            return False
        
        # 启动通道0
        ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('✅ 成功启动CAN通道0')
        else:
            print('❌ 启动CAN通道0失败')
            canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
            return False
        
        # 初始化通道1
        ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 1, byref(vci_config))
        if ret == STATUS_OK:
            print('✅ 成功初始化CAN通道1')
        else:
            print('❌ 初始化CAN通道1失败')
        
        # 启动通道1
        ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 1)
        if ret == STATUS_OK:
            print('✅ 成功启动CAN通道1')
        else:
            print('❌ 启动CAN通道1失败')
        
        # 准备发送测试消息
        print("\n发送测试消息...")
        data_array = (c_ubyte * 8)(0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)
        reserved_array = (c_ubyte * 3)(0, 0, 0)
        
        # 创建消息对象
        test_msg = VCI_CAN_OBJ(
            ID=0x100,              # 测试消息ID
            TimeStamp=0,
            TimeFlag=0,
            SendType=1,            # 单次发送
            RemoteFlag=0,          # 数据帧
            ExternFlag=0,          # 标准帧
            DataLen=8,             # 数据长度
            Data=data_array,
            Reserved=reserved_array
        )
        
        # 发送测试消息
        ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(test_msg), 1)
        if ret == STATUS_OK:
            print('✅ 成功发送测试消息')
        else:
            print('❌ 发送测试消息失败')
        
        # 准备接收消息
        print("\n等待接收消息(5秒)...")
        rx_buffer = VCI_CAN_OBJ_ARRAY(50)  # 最多接收50条消息
        
        # 接收循环
        start_time = time.time()
        rx_count = 0
        
        while time.time() - start_time < 5:  # 5秒超时
            # 查询通道0接收
            ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_buffer.ADDR), 50, 100)
            if ret > 0:
                for i in range(ret):
                    rx_count += 1
                    print(f"通道0接收: ID=0x{rx_buffer.STRUCT_ARRAY[i].ID:X}, "
                          f"数据={list(rx_buffer.STRUCT_ARRAY[i].Data[:rx_buffer.STRUCT_ARRAY[i].DataLen])}")
            
            # 查询通道1接收
            ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 1, byref(rx_buffer.ADDR), 50, 100)
            if ret > 0:
                for i in range(ret):
                    rx_count += 1
                    print(f"通道1接收: ID=0x{rx_buffer.STRUCT_ARRAY[i].ID:X}, "
                          f"数据={list(rx_buffer.STRUCT_ARRAY[i].Data[:rx_buffer.STRUCT_ARRAY[i].DataLen])}")
            
            time.sleep(0.1)  # 短暂休眠，减少CPU占用
        
        if rx_count > 0:
            print(f"\n✅ 共接收到 {rx_count} 条消息")
        else:
            print("\n⚠️ 未接收到任何消息。如果CAN总线上有其他设备，请检查连接和终端电阻")
        
    finally:
        # 关闭设备
        canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
        print("\n设备已关闭")
    
    return True

if __name__ == "__main__":
    test_zlg_can_connection()