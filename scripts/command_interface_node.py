#!/usr/bin/env python3

import rospy
from mecanum_control.msg import MecanumCommand
import sys
import threading

class CommandInterfaceNode:
    def __init__(self):
        rospy.init_node('command_interface_node')
        self.cmd_pub = rospy.Publisher('/mecanum_cmd', MecanumCommand, queue_size=10)
        self.running = True
        
        # 运动模式映射
        self.motion_modes = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'q': 'turn_ccw',
            'e': 'turn_cw'
        }
        
        rospy.loginfo("命令行接口节点已启动")
        self.display_help()
        
        # 启动命令输入线程
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
    
    def display_help(self):
        """显示帮助信息"""
        print("\n麦克纳姆轮控制程序 (ROS 版)")
        print("=======================")
        # 帮助信息...
        
    def parse_command(self, command):
        """解析命令"""
        parts = command.strip().split()
        # 命令解析逻辑...
        
    def input_loop(self):
        """命令输入循环"""
        while self.running and not rospy.is_shutdown():
            try:
                command = input("\n请输入命令 > ")
                self.parse_command(command)
            except (KeyboardInterrupt, EOFError):
                self.running = False
                rospy.signal_shutdown("用户退出")
            except Exception as e:
                print(f"发生错误: {e}")
                
    def shutdown(self):
        """关闭节点时的清理操作"""
        self.running = False
        rospy.loginfo("命令行接口节点关闭")

if __name__ == '__main__':
    cmd_node = CommandInterfaceNode()
    rospy.on_shutdown(cmd_node.shutdown)
    rospy.spin()
