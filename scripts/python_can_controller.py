import can
import time
import rospy

class PythonCANController:
    """Python-CAN based controller to replace ZLGCANController"""
    
    def __init__(self, interface='canalystii', channel='0', bitrate=500000):
        """Initialize Python-CAN controller

        Args:
            interface: CAN interface type ('socketcan', 'canalystii', etc)
            channel: CAN channel index
            bitrate: CAN bus bitrate in bps
        """
        # 从 ROS 参数服务器获取参数
        if rospy.has_param('~can_interface'):
            interface = rospy.get_param('~can_interface')
        if rospy.has_param('~can_channel'):
            channel = rospy.get_param('~can_channel')
        if rospy.has_param('~can_bitrate'):
            bitrate = rospy.get_param('~can_bitrate')

        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.connected = False

        # Try to connect
        self.connect()

    def connect(self):
        """Connect to CAN bus using python-can"""
        try:
            rospy.loginfo(
                f"正在连接到CAN总线: 接口={self.interface}, 通道={self.channel}, 波特率={self.bitrate}...")
            self.bus = can.Bus(
                interface=self.interface,
                channel=self.channel,
                bitrate=self.bitrate
            )
            self.connected = True
            rospy.loginfo(f"CAN总线连接成功: 通道 {self.channel}")
            return True
        except can.CanError as e:
            rospy.logerr(f"连接CAN总线失败: {e}")
            return False

    def send_message(self, id, data, is_extended=False, is_remote=False):
        """Send CAN message
        
        Args:
            id: CAN message ID
            data: Message data (list of bytes)
            is_extended: Whether to use extended ID
            is_remote: Whether this is a remote frame
            
        Returns:
            bool: Success status
        """
        if not self.connected:
            rospy.logerr("CAN设备未连接")
            return False
        
        try:
            # Create CAN message
            msg = can.Message(
                arbitration_id=id,
                data=bytes(data),
                is_extended_id=is_extended,
                is_remote_frame=is_remote
            )
            
            # Print hex data for debugging
            hex_data = ' '.join([f"{b:02X}" for b in data])
            # rospy.loginfo(f"发送CAN消息: ID=0x{id:X}, 数据=[{hex_data}]")
            
            # Send message
            self.bus.send(msg)
            return True
        except Exception as e:
            rospy.logerr(f"发送CAN消息异常: {e}")
            return False
    
    def receive_message(self, wait_time=0.1):
        """Receive CAN messages
        
        Args:
            wait_time: Wait time in seconds (converted from ms)
            
        Returns:
            list: List of received messages as dictionaries
        """
        if not self.connected:
            rospy.logerr("CAN设备未连接")
            return []
        
        try:
            messages = []
            timeout = time.time() + wait_time
            
            # Receive messages until timeout
            while time.time() < timeout:
                msg = self.bus.recv(timeout=timeout - time.time())
                if msg:
                    # Convert to format expected by existing code
                    messages.append({
                        'id': msg.arbitration_id,
                        'data': list(msg.data),
                        'is_remote': msg.is_remote_frame,
                        'is_extended': msg.is_extended_id,
                        'timestamp': msg.timestamp
                    })
            
            return messages
        except Exception as e:
            rospy.logerr(f"接收CAN消息异常: {e}")
            return []
    
    def close(self):
        """Close CAN bus connection"""
        if self.connected and self.bus:
            self.bus.shutdown()
            self.connected = False
            rospy.loginfo("CAN设备已关闭")
