#!/usr/bin/env python3
# can_device_scanner.py - CAN设备扫描工具

from ctypes import *
import os
import time
import sys
import argparse

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
            return True
        return False

def scan_can_devices_passive(can_controller, scan_time=5):
    """
    被动扫描CAN总线上的设备
    通过监听CAN总线上的消息来识别活跃设备
    
    参数:
    can_controller: ZLG CAN控制器实例
    scan_time: 扫描时间(秒)
    
    返回:
    发现的设备ID列表
    """
    if not can_controller.connected:
        print("CAN设备未连接")
        return []
    
    print(f"开始被动扫描CAN总线设备 (监听{scan_time}秒)...")
    print("监听中，按Ctrl+C可提前结束...")
    
    active_ids = set()
    start_time = time.time()
    
    try:
        while time.time() - start_time < scan_time:
            # 接收所有消息，每次等待100毫秒
            messages = can_controller.receive_message(wait_time=100)
            
            for msg in messages:
                if msg['id'] not in active_ids:
                    active_ids.add(msg['id'])
                    data_hex = ' '.join([f"{b:02X}" for b in msg['data']])
                    print(f"检测到新设备! ID: 0x{msg['id']:X}, 数据: [{data_hex}]")
            
            # 简单的进度显示
            elapsed = time.time() - start_time
            progress = min(100, int(elapsed / scan_time * 100))
            print(f"扫描进度: {progress}% (已发现 {len(active_ids)} 个设备)", end="\r")
            
            # 短暂休眠，减少CPU占用
            time.sleep(0.05)
        
        print("\n扫描完成!")
        
        if not active_ids:
            print("未检测到任何设备活动")
        else:
            print(f"检测到 {len(active_ids)} 个活跃设备:")
            for device_id in sorted(active_ids):
                # 尝试解释一些常见ID的用途
                id_description = ""
                if 0x000 <= device_id <= 0x07F:
                    id_description = "[NMT控制]"
                elif 0x080 <= device_id <= 0x0FF:
                    id_description = "[同步/紧急消息]"
                elif 0x180 <= device_id <= 0x1FF:
                    id_description = "[TPD0 (节点状态)]"
                elif 0x200 <= device_id <= 0x27F:
                    id_description = "[RPD0]" 
                elif 0x280 <= device_id <= 0x2FF:
                    id_description = "[TPD1]"
                elif 0x300 <= device_id <= 0x37F:
                    id_description = "[RPD1]"
                elif 0x380 <= device_id <= 0x3FF:
                    id_description = "[TPD2]"
                elif 0x400 <= device_id <= 0x47F:
                    id_description = "[RPD2]"
                elif 0x480 <= device_id <= 0x4FF:
                    id_description = "[TPD3]"
                elif 0x500 <= device_id <= 0x57F:
                    id_description = "[RPD3]"
                elif 0x580 <= device_id <= 0x5FF:
                    id_description = "[TSDO (节点应答)]"
                elif 0x600 <= device_id <= 0x67F:
                    id_description = "[RSDO (节点命令)]"
                elif 0x700 <= device_id <= 0x77F:
                    id_description = "[Node Guarding]"
                
                node_id = None
                if 0x580 <= device_id <= 0x5FF:
                    node_id = device_id - 0x580
                elif 0x600 <= device_id <= 0x67F:
                    node_id = device_id - 0x600
                elif 0x700 <= device_id <= 0x77F:
                    node_id = device_id - 0x700
                
                if node_id is not None:
                    print(f"- ID: 0x{device_id:X} {id_description} (可能是节点 {node_id})")
                else:
                    print(f"- ID: 0x{device_id:X} {id_description}")
        
        return sorted(list(active_ids))
        
    except KeyboardInterrupt:
        print("\n扫描被用户中断")
        if active_ids:
            print(f"已检测到 {len(active_ids)} 个活跃设备:")
            for device_id in sorted(active_ids):
                print(f"- 设备ID: 0x{device_id:X}")
        return sorted(list(active_ids))

def scan_can_devices_active(can_controller, start_id=1, end_id=127, timeout=0.1):
    """
    主动扫描CAN总线上的设备
    向每个可能的节点ID发送Node Guarding请求，检查是否有回应
    
    参数:
    can_controller: ZLG CAN控制器实例
    start_id: 起始节点ID
    end_id: 结束节点ID
    timeout: 每个节点的等待超时时间(秒)
    
    返回:
    发现的设备ID列表
    """
    if not can_controller.connected:
        print("CAN设备未连接")
        return []
    
    print(f"开始主动扫描CAN总线设备 (节点 {start_id}-{end_id})...")
    print("扫描中，按Ctrl+C可提前结束...")
    
    found_devices = []
    total_nodes = end_id - start_id + 1
    
    try:
        for node_id in range(start_id, end_id + 1):
            # 计算进度
            progress = int((node_id - start_id) / total_nodes * 100)
            print(f"扫描节点 {node_id}/{end_id} ({progress}%)  ", end="\r")
            
            # 发送Node Guarding请求 (CANopen协议)
            request_id = 0x700 + node_id
            can_controller.send_message(request_id, [], is_remote=True)
            
            # 等待回应
            start_wait = time.time()
            response_received = False
            
            while time.time() - start_wait < timeout:
                messages = can_controller.receive_message(wait_time=10)
                
                for msg in messages:
                    # 检查是否是对应节点的回应
                    if msg['id'] == request_id:
                        found_devices.append(node_id)
                        data_hex = ' '.join([f"{b:02X}" for b in msg['data']])
                        print(f"\n发现设备! 节点ID: {node_id}, 回应: [{data_hex}]")
                        response_received = True
                        break
                
                if response_received:
                    break
                
                time.sleep(0.01)
        
        print("\n扫描完成!")
        
        if not found_devices:
            print("未发现任何支持CANopen的设备")
        else:
            print(f"发现 {len(found_devices)} 个设备:")
            for device_id in found_devices:
                print(f"- 节点ID: {device_id} (通信ID: 0x{0x700 + device_id:X})")
        
        return found_devices
        
    except KeyboardInterrupt:
        print("\n扫描被用户中断")
        if found_devices:
            print(f"已发现 {len(found_devices)} 个设备:")
            for device_id in found_devices:
                print(f"- 节点ID: {device_id} (通信ID: 0x{0x700 + device_id:X})")
        return found_devices

def scan_motor_drives(can_controller, start_id=1, end_id=127, protocol='cia402'):
    """
    扫描支持特定协议的电机驱动器
    
    参数:
    can_controller: ZLG CAN控制器实例
    start_id: 起始节点ID
    end_id: 结束节点ID
    protocol: 电机协议类型 ('cia402' - CiA 402伺服驱动器)
    
    返回:
    发现的设备ID列表
    """
    if not can_controller.connected:
        print("CAN设备未连接")
        return []
    
    print(f"开始扫描电机驱动器 (节点 {start_id}-{end_id}, 协议: {protocol})...")
    print("扫描中，按Ctrl+C可提前结束...")
    
    found_drives = []
    total_nodes = end_id - start_id + 1
    
    try:
        for node_id in range(start_id, end_id + 1):
            # 计算进度
            progress = int((node_id - start_id) / total_nodes * 100)
            print(f"扫描节点 {node_id}/{end_id} ({progress}%)  ", end="\r")
            
            if protocol.lower() == 'cia402':
                # 发送SDO读取命令，读取设备类型(0x1000)
                can_controller.send_message(
                    0x600 + node_id,
                    [0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00]
                )
                
                # 等待回应
                time.sleep(0.1)  
                messages = can_controller.receive_message(wait_time=100)
                
                response_received = False
                for msg in messages:
                    if msg['id'] == 0x580 + node_id:
                        found_drives.append(node_id)
                        data_hex = ' '.join([f"{b:02X}" for b in msg['data']])
                        print(f"\n发现电机驱动器! 节点ID: {node_id}, 回应: [{data_hex}]")
                        response_received = True
                        break
                
                if not response_received:
                    # 尝试读取状态字(0x6041)
                    can_controller.send_message(
                        0x600 + node_id,
                        [0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
                    )
                    
                    time.sleep(0.1)
                    messages = can_controller.receive_message(wait_time=100)
                    
                    for msg in messages:
                        if msg['id'] == 0x580 + node_id:
                            found_drives.append(node_id)
                            data_hex = ' '.join([f"{b:02X}" for b in msg['data']])
                            print(f"\n发现电机驱动器! 节点ID: {node_id}, 回应: [{data_hex}]")
                            break
            else:
                print(f"\n不支持的协议: {protocol}")
                return []
                
        print("\n扫描完成!")
        
        if not found_drives:
            print(f"未发现任何支持{protocol}协议的电机驱动器")
        else:
            print(f"发现 {len(found_drives)} 个电机驱动器:")
            for device_id in found_drives:
                print(f"- 节点ID: {device_id}")
                print(f"  命令ID: 0x{0x600 + device_id:X}")
                print(f"  状态ID: 0x{0x580 + device_id:X}")
        
        return found_drives
        
    except KeyboardInterrupt:
        print("\n扫描被用户中断")
        if found_drives:
            print(f"已发现 {len(found_drives)} 个电机驱动器:")
            for device_id in found_drives:
                print(f"- 节点ID: {device_id}")
        return found_drives

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='CAN总线设备扫描工具')
    
    parser.add_argument('--mode', '-m', choices=['passive', 'active', 'motor'], default='passive',
                        help='扫描模式: passive(被动监听), active(主动查询), motor(电机驱动器)')
    
    parser.add_argument('--bitrate', '-b', type=int, default=500000,
                        help='CAN总线波特率(bps) (默认: 500000)')
    
    parser.add_argument('--device', '-d', type=int, default=0,
                        help='ZLG CAN设备索引 (默认: 0)')
    
    parser.add_argument('--channel', '-c', type=int, default=0,
                        help='CAN通道索引 (默认: 0)')
    
    parser.add_argument('--dll', type=str, default='./ControlCAN.dll',
                        help='ZLG CAN驱动库路径 (默认: ./ControlCAN.dll)')
    
    parser.add_argument('--time', '-t', type=int, default=10,
                        help='被动扫描时间(秒) (默认: 10)')
    
    parser.add_argument('--start', '-s', type=int, default=1,
                        help='主动扫描起始节点ID (默认: 1)')
    
    parser.add_argument('--end', '-e', type=int, default=16,
                        help='主动扫描结束节点ID (默认: 16)')
    
    parser.add_argument('--protocol', '-p', type=str, default='cia402',
                        help='电机协议类型 (默认: cia402)')
    
    args = parser.parse_args()
    
    print("CAN总线设备扫描工具")
    print("==================")
    print(f"扫描模式: {args.mode}")
    print(f"波特率: {args.bitrate} bps")
    print(f"设备索引: {args.device}")
    print(f"通道: {args.channel}")
    print(f"驱动库: {args.dll}")
    
    if args.mode == 'passive':
        print(f"扫描时间: {args.time} 秒")
    else:
        print(f"扫描范围: 节点 {args.start} 到 {args.end}")
    
    print("==================")
    
    # 创建CAN控制器
    can_controller = ZLGCANController(
        device_index=args.device,
        channel=args.channel,
        bitrate=args.bitrate,
        dll_path=args.dll
    )
    
    if not can_controller.connected:
        print("CAN设备连接失败，程序退出")
        return
    
    try:
        # 根据模式执行不同的扫描
        if args.mode == 'passive':
            # 被动扫描模式
            scan_can_devices_passive(can_controller, scan_time=args.time)
        elif args.mode == 'active':
            # 主动扫描模式
            scan_can_devices_active(can_controller, start_id=args.start, end_id=args.end)
        elif args.mode == 'motor':
            # 电机驱动器扫描模式
            scan_motor_drives(can_controller, start_id=args.start, end_id=args.end, protocol=args.protocol)
        else:
            print(f"不支持的扫描模式: {args.mode}")
    
    except Exception as e:
        print(f"扫描过程发生错误: {e}")
    finally:
        # 关闭CAN设备
        can_controller.close()

if __name__ == "__main__":
    main()