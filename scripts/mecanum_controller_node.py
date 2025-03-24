#!/usr/bin/env python3

import rospy
from mecanum_control.msg import MecanumCommand, MecanumStatus
from python_can_controller import PythonCANController
from mecanum_controller import MecanumController

class MecanumControllerNode:
    def __init__(self):
        rospy.init_node('mecanum_controller_node')

        # 从参数服务器获取配置
        self.wheel_ids = rospy.get_param('~wheel_ids', [1, 2, 12, 4])

        # 初始化 CAN 控制器和麦克纳姆轮控制器
        self.can_controller = PythonCANController()
        if not self.can_controller.connected:
            rospy.logerr("CAN 设备连接失败，节点无法启动")
            return

        self.mecanum = MecanumController(self.can_controller,
                                         wheel_ids=tuple(self.wheel_ids))
        if not self.mecanum.initialize():
            rospy.logerr("麦克纳姆轮控制器初始化失败")
            return

        # 创建订阅者和发布者
        self.cmd_sub = rospy.Subscriber('/mecanum_cmd', MecanumCommand,
                                        self.command_callback, queue_size=10)
        self.status_pub = rospy.Publisher('/mecanum_status', MecanumStatus,
                                         queue_size=10)

        # 启动状态发布定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_status)

        rospy.loginfo("麦克纳姆轮控制器节点已启动")

    def command_callback(self, msg):
        """处理接收到的命令消息"""
        motion_mapping = {
            'forward': 'w',
            'backward': 's',
            'left': 'a',
            'right': 'd',
            'turn_ccw': 'q',
            'turn_cw': 'e'
        }

        motion_key = motion_mapping.get(msg.motion_type)
        if motion_key:
            self.mecanum.execute_motion(
                motion_key,
                msg.accel_time,
                msg.cruise_time,
                msg.decel_time,
                msg.target_rpm
            )
        else:
            rospy.logwarn(f"未知运动类型: {msg.motion_type}")

    def publish_status(self, event):
        """发布控制器状态"""
        status = MecanumStatus()
        status.header.stamp = rospy.Time.now()

        # 填充状态信息
        # ...

        self.status_pub.publish(status)

    def shutdown(self):
        """关闭节点时的清理操作"""
        rospy.loginfo("关闭麦克纳姆轮控制器节点")
        if self.mecanum:
            self.mecanum.close()
        if self.can_controller:
            self.can_controller.close()

if __name__ == '__main__':
    controller_node = MecanumControllerNode()
    rospy.on_shutdown(controller_node.shutdown)
    rospy.spin()
