import time

class MotorController:
    """单个电机控制类"""
    
    def __init__(self, can_controller, node_id):
        """初始化电机控制器"""
        self.can = can_controller
        self.node_id = node_id
        self.is_enabled = False
    
    def set_operation_mode(self):
        """设置节点为操作模式"""
        print(f"设置节点 {self.node_id} 为操作模式...")
        return self.can.send_message(0x000, [0x01, self.node_id])
    
    def set_speed_mode(self):
        """设置为速度控制模式"""
        print(f"设置节点 {self.node_id} 为速度控制模式...")
        return self.can.send_message(0x600 + self.node_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
    
    def enable(self):
        """使能驱动器"""
        print(f"使能节点 {self.node_id} 的驱动器...")
        result = self.can.send_message(0x600 + self.node_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        if result:
            self.is_enabled = True
        return result
    
    def disable(self):
        """禁用驱动器"""
        print(f"禁用节点 {self.node_id} 的驱动器...")
        result = self.can.send_message(0x600 + self.node_id, [0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        if result:
            self.is_enabled = False
        return result
    
    def set_speed(self, rpm):
        """设置电机转速"""
        if not self.is_enabled:
            print(f"警告: 节点 {self.node_id} 的驱动器未使能")
            return False
            
        # 将RPM转换为命令值
        counts_per_rev = 10000  # 2500线 * 4倍频
        counts_per_sec = rpm * counts_per_rev / 60
        cmd_value = int(counts_per_sec / 0.1)
        
        # 转换为字节数组（小端序）
        cmd_bytes = list(cmd_value.to_bytes(4, byteorder='little', signed=True))
        
        # 构造速度命令
        data = [0x23, 0xFF, 0x60, 0x00] + cmd_bytes
        
        print(f"设置节点 {self.node_id} 的速度为 {rpm} RPM (命令值: {cmd_value})...")
        return self.can.send_message(0x600 + self.node_id, data)
    
    def initialize(self):
        """初始化电机控制器"""
        print(f"初始化电机 {self.node_id}...")
        
        # 设置操作模式
        if not self.set_operation_mode():
            print(f"设置节点 {self.node_id} 的操作模式失败")
            return False
        time.sleep(0.1)
        
        # 设置速度控制模式
        if not self.set_speed_mode():
            print(f"设置节点 {self.node_id} 的速度控制模式失败")
            return False
        time.sleep(0.1)
        
        # 使能驱动器
        if not self.enable():
            print(f"使能节点 {self.node_id} 的驱动器失败")
            return False
        time.sleep(0.1)
        
        print(f"电机 {self.node_id} 初始化成功")
        return True