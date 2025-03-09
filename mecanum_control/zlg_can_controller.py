from ctypes import *
import os
import time

class ZLGCANController:
    """ZLG USBCAN设备通信控制类"""
    
    # 设备类型常量
    VCI_USBCAN2 = 4
    STATUS_OK = 1
    
    # 波特率映射表
    BITRATE_TIMING = {
        1000000: (0x00, 0x14),  # 1000K
        800000: (0x00, 0x16),   # 800K
        500000: (0x00, 0x1C),   # 500K
        250000: (0x01, 0x1C),   # 250K
        125000: (0x03, 0x1C),   # 125K
        100000: (0x04, 0x1C),   # 100K
        50000: (0x09, 0x1C),    # 50K
        20000: (0x18, 0x1C)     # 20K
    }
    
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
    
    def __init__(self, device_index=0, channel=0, bitrate=500000, dll_path='./ControlCAN.dll'):
        """初始化ZLG CAN控制器"""
        self.device_index = device_index
        self.channel = channel
        self.bitrate = bitrate
        self.dll_path = dll_path
        self.can_dll = None
        self.connected = False
        
        # 连接设备
        self.connect()
    
    def connect(self):
        """连接到ZLG CAN设备"""
        try:
            # 检查DLL文件是否存在
            if not os.path.exists(self.dll_path):
                print(f"错误: {self.dll_path} 文件不存在")
                return False
            
            # 加载DLL
            self.can_dll = windll.LoadLibrary(self.dll_path)
            print(f"成功加载 {self.dll_path}")
            
            # 打开设备
            ret = self.can_dll.VCI_OpenDevice(self.VCI_USBCAN2, self.device_index, 0)
            if ret != self.STATUS_OK:
                print(f"打开ZLG USBCAN设备失败，错误码: {ret}")
                return False
                
            # 获取波特率对应的timing值
            timing0, timing1 = self.BITRATE_TIMING.get(self.bitrate, (0x00, 0x1C))
            print(f"配置CAN总线波特率: {self.bitrate/1000}K (Timing0=0x{timing0:02X}, Timing1=0x{timing1:02X})")
            
            # 初始化CAN通道
            vci_config = self.VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, timing0, timing1, 0)
            ret = self.can_dll.VCI_InitCAN(self.VCI_USBCAN2, self.device_index, self.channel, byref(vci_config))
            if ret != self.STATUS_OK:
                print(f"初始化CAN通道{self.channel}失败，错误码: {ret}")
                self.can_dll.VCI_CloseDevice(self.VCI_USBCAN2, self.device_index)
                return False
            
            # 启动CAN通道
            ret = self.can_dll.VCI_StartCAN(self.VCI_USBCAN2, self.device_index, self.channel)
            if ret != self.STATUS_OK:
                print(f"启动CAN通道{self.channel}失败，错误码: {ret}")
                self.can_dll.VCI_CloseDevice(self.VCI_USBCAN2, self.device_index)
                return False
            
            self.connected = True
            print(f"CAN设备已连接，通道: {self.channel}")
            return True
            
        except Exception as e:
            print(f"连接CAN设备失败: {e}")
            return False
            
    def send_message(self, id, data, is_extended=False, is_remote=False):
        """发送CAN消息"""
        if not self.connected:
            print("CAN设备未连接")
            return False
            
        try:
            # 准备数据数组
            data_len = len(data)
            data_array = (c_ubyte * 8)(*data)
            reserved_array = (c_ubyte * 3)(0, 0, 0)
            
            # 创建消息对象
            msg = self.VCI_CAN_OBJ(
                ID=id,
                TimeStamp=0,
                TimeFlag=0,
                SendType=1,                # 单次发送
                RemoteFlag=1 if is_remote else 0,  # 远程帧/数据帧
                ExternFlag=1 if is_extended else 0,  # 扩展帧/标准帧
                DataLen=data_len,
                Data=data_array,
                Reserved=reserved_array
            )
            
            # 打印十六进制消息内容
            hex_data = ' '.join([f"{b:02X}" for b in data])
            print(f"发送CAN消息: ID=0x{id:X}, 数据=[{hex_data}]")
            
            # 发送消息
            ret = self.can_dll.VCI_Transmit(self.VCI_USBCAN2, self.device_index, self.channel, byref(msg), 1)
            if ret != self.STATUS_OK:
                print(f"发送CAN消息失败，错误码: {ret}")
                return False
                
            return True
            
        except Exception as e:
            print(f"发送CAN消息异常: {e}")
            return False
    
    def receive_message(self, wait_time=100):
        """接收CAN消息，返回消息列表，wait_time为等待时间(毫秒)"""
        if not self.connected:
            print("CAN设备未连接")
            return []
            
        try:
            # 创建接收缓冲区
            rx_buffer = (self.VCI_CAN_OBJ * 50)()  # 最多接收50条消息
            
            # 接收消息
            ret = self.can_dll.VCI_Receive(self.VCI_USBCAN2, self.device_index, self.channel, byref(rx_buffer), 50, wait_time)
            
            # 处理接收到的消息
            messages = []
            if ret > 0:
                for i in range(ret):
                    msg = rx_buffer[i]
                    # 提取数据
                    data = [msg.Data[j] for j in range(msg.DataLen)]
                    messages.append({
                        'id': msg.ID,
                        'data': data,
                        'is_remote': bool(msg.RemoteFlag),
                        'is_extended': bool(msg.ExternFlag),
                        'timestamp': msg.TimeStamp
                    })
            
            return messages
            
        except Exception as e:
            print(f"接收CAN消息异常: {e}")
            return []
    
    def close(self):
        """关闭CAN设备连接"""
        if self.connected and self.can_dll:
            self.can_dll.VCI_CloseDevice(self.VCI_USBCAN2, self.device_index)
            self.connected = False
            print("CAN设备已关闭")