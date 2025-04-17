#!/usr/bin/env python3

import rospy
from mecanum_control.msg import MecanumCommand
import sys
import tty
import termios

class KeyboardControlNode:
    def __init__(self):
        rospy.init_node('keyboard_control_node')
        self.cmd_pub = rospy.Publisher('/mecanum_cmd', MecanumCommand, queue_size=10)
        
        # 运动模式映射
        self.motion_modes = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'q': 'turn_ccw',
            'e': 'turn_cw'
        }
        
        # 获取终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        rospy.loginfo("键盘控制节点已启动")
        self.run()
    
    def get_key(self):
        """读取键盘输入"""
        if rospy.is_shutdown():
            return None
        return sys.stdin.read(1)
    
    def run(self):
        """主循环"""
        print("\n麦克纳姆轮键盘控制")
        print("===================")
        print("w/s: 前进/后退")
        print("a/d: 左移/右移")
        print("q/e: 逆时针/顺时针旋转")
        print("Ctrl+C 退出")
        
        while not rospy.is_shutdown():
            key = self.get_key()
            if key is None:
                break
            
            motion_type = self.motion_modes.get(key)
            if motion_type:
                command = MecanumCommand()
                command.motion_type = motion_type
                command.accel_time = 1
                command.cruise_time = 0
                command.decel_time = 1
                command.target_rpm = 100.0
                self.cmd_pub.publish(command)
                print(f"发送命令: {motion_type}")
            elif ord(key) == 3:  # Ctrl+C
                break
            else:
                print("无效按键")
    
    def shutdown(self):
        """关闭节点时的清理操作"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        rospy.loginfo("键盘控制节点关闭")

if __name__ == '__main__':
    keyboard_node = KeyboardControlNode()
    rospy.on_shutdown(keyboard_node.shutdown)
