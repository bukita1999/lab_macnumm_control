#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from mecanum_control.msg import MecanumCommand, MecanumStatus
from python_can_controller import PythonCANController
from mecanum_controller import MecanumController
from actual_robot_motor_controller import ActualRobotMotorController # 新增导入
from string_command_controller import StringCommandController # <-- 新增导入
from trajectory_planner import TrajectoryPlanner # <-- 新增导入

class MecanumControllerNode:
    # --- Constants ---
    TRAJECTORY_TIME_STEP = 0.1 # Seconds, resolution for trajectory execution

    def __init__(self):
        rospy.init_node('mecanum_controller_node')

        # 从参数服务器获取配置
        # --- 获取通用参数 ---
        self.driver_type = rospy.get_param('~driver_type', 'original') # 'original', 'actual', or 'string_cmd'
        self.publish_rate = rospy.get_param('~publish_rate', 10.0) # Hz

        # Trajectory execution state (for string_cmd driver)
        self.trajectory_timer = None
        self.current_speed_profile = None
        self.current_motion_type = None
        self.trajectory_index = 0
        self.string_cmd_speed_mode = 'A' # Default, will be overwritten by param if driver_type is string_cmd

        # --- 初始化 CAN 控制器 ---
        # TODO: 从参数读取 CAN 配置 (interface, channel, bitrate)
        can_interface = rospy.get_param('~can_interface', 'socketcan')
        can_channel = rospy.get_param('~can_channel', 'can0')
        can_bitrate = rospy.get_param('~can_bitrate', 500000)
        self.can_controller = PythonCANController(interface=can_interface, channel=can_channel, bitrate=can_bitrate)
        if not self.can_controller.connected:
            rospy.logerr(f"CAN 设备 {can_interface}:{can_channel} 连接失败，节点无法启动")
            self.controller = None # 标记未成功初始化
            return

        # --- 根据驱动类型初始化特定控制器 ---
        self.controller = None
        if self.driver_type == 'original':
            rospy.loginfo("使用 'original' 驱动模式 (MotorController)")
            wheel_ids = rospy.get_param('~wheel_ids', [1, 2, 12, 4]) # 原驱动需要节点ID
            self.controller = MecanumController(self.can_controller, wheel_ids=tuple(wheel_ids))
            if not self.controller.initialize():
                rospy.logerr("原始麦克纳姆轮控制器 (MecanumController) 初始化失败")
                self.controller = None
                return
        elif self.driver_type == 'actual':
            rospy.loginfo("使用 'actual' 驱动模式 (ActualRobotMotorController)")
            # 获取运动学参数
            self.wheel_radius = rospy.get_param('~kinematics/wheel_radius', 0.05) # m
            self.wheel_separation_width = rospy.get_param('~kinematics/wheel_separation_width', 0.3) # m (左右轮距)
            self.wheel_separation_length = rospy.get_param('~kinematics/wheel_separation_length', 0.4) # m (前后轴距)
            # 获取速度指令缩放比例 (可选，可以硬编码在 ActualRobotMotorController 中)
            # velocity_scale = rospy.get_param('~actual_robot_velocity_scale', 32767.0 / 1.0)

            self.controller = ActualRobotMotorController(self.can_controller)
            # **重要假设**: 假设 can_controller 有 add_listener 方法
            # 如果没有，需要修改 PythonCANController 或在此处实现接收逻辑
            try:
                self.can_controller.add_listener(self.controller._can_message_callback)
                rospy.loginfo("CAN 消息监听器已添加")
            except AttributeError:
                 rospy.logwarn("CAN 控制器没有 add_listener 方法，状态反馈可能无法工作。请检查 PythonCANController 实现。")
            except Exception as e:
                 rospy.logwarn(f"添加 CAN 监听器时出错: {e}")

            if not self.controller.initialize():
                rospy.logerr("实际机器人控制器 (ActualRobotMotorController) 初始化失败")
                self.controller = None
                return
        elif self.driver_type == 'string_cmd':
            rospy.loginfo("使用 'string_cmd' 驱动模式 (StringCommandController)")
            self.string_cmd_speed_mode = rospy.get_param('~string_cmd_speed_mode', 'A') # Get speed mode from config
            self.controller = StringCommandController(self.can_controller, speed_mode_char=self.string_cmd_speed_mode)
            if not self.controller.initialize():
                rospy.logerr("字符串命令控制器 (StringCommandController) 初始化失败")
                self.controller = None
                return
        else:
            rospy.logerr(f"未知的 driver_type: {self.driver_type}")
            return

        # 检查控制器是否成功初始化
        if self.controller is None:
             rospy.logerr("控制器未能成功初始化，节点退出。")
             return
        # 创建订阅者和发布者
        self.cmd_sub = rospy.Subscriber('/mecanum_cmd', MecanumCommand,
                                        self.command_callback, queue_size=10)
        self.status_pub = rospy.Publisher('/mecanum_status', MecanumStatus,
                                         queue_size=10)

        # 启动状态发布定时器
        if self.publish_rate > 0:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_status)
        else:
            rospy.logwarn("状态发布频率 <= 0，状态将不会被发布。")
        rospy.loginfo("麦克纳姆轮控制器节点已启动")

    def command_callback(self, msg: MecanumCommand):
        """处理接收到的命令消息"""
        if self.controller is None:
            rospy.logerr("控制器未初始化，无法处理命令。")
            return

        if self.driver_type == 'original':
            # --- 原始驱动逻辑 ---
            motion_mapping = {
                'forward': 'w', 'backward': 's', 'left': 'a',
                'right': 'd', 'turn_ccw': 'q', 'turn_cw': 'e'
            }
            motion_key = motion_mapping.get(msg.motion_type)
            if motion_key:
                # 注意：原始 MecanumController 是阻塞执行的
                # TODO: 考虑将其改为非阻塞或在单独线程中执行
                rospy.loginfo(f"Original Driver: Executing {msg.motion_type} with RPM {msg.target_rpm}")
                self.controller.execute_motion(
                    motion_key,
                    msg.accel_time,
                    msg.cruise_time,
                    msg.decel_time,
                    msg.target_rpm
                )
            else:
                rospy.logwarn(f"未知运动类型: {msg.motion_type}")

        elif self.driver_type == 'actual':
            # --- 新驱动逻辑 ---
            # 1. 将 motion_type 和 target_rpm 转换为 Vx, Vy, Vw
            target_vx, target_vy, target_vw = 0.0, 0.0, 0.0
            try:
                # 将 RPM 转换为轮子线速度 m/s
                target_wheel_linear_speed_mps = msg.target_rpm * (2 * math.pi / 60.0) * self.wheel_radius

                # 近似计算角速度 rad/s (基于轮子线速度和机器人几何中心到轮子的平均距离)
                # 这个转换可能需要根据实际机器人标定调整
                avg_dist_to_center = (self.wheel_separation_width + self.wheel_separation_length) / 4.0
                if avg_dist_to_center > 0.01: # 避免除零
                     target_angular_speed_radps = target_wheel_linear_speed_mps / avg_dist_to_center
                else:
                     target_angular_speed_radps = 0.0


                if msg.motion_type == 'forward':
                    target_vy = target_wheel_linear_speed_mps # 假设Y轴向前
                elif msg.motion_type == 'backward':
                    target_vy = -target_wheel_linear_speed_mps
                elif msg.motion_type == 'left':
                    target_vx = -target_wheel_linear_speed_mps # 假设X轴向左
                elif msg.motion_type == 'right':
                    target_vx = target_wheel_linear_speed_mps
                elif msg.motion_type == 'turn_ccw':
                    target_vw = target_angular_speed_radps # 假设W正为逆时针
                elif msg.motion_type == 'turn_cw':
                    target_vw = -target_angular_speed_radps
                elif msg.motion_type == 'stop': # 添加停止指令
                    pass # 速度保持为0
                else:
                    rospy.logwarn(f"未知运动类型: {msg.motion_type}")
                    return # 不发送指令

                # TODO: 实现梯形加减速逻辑，此处暂时直接发送目标速度
                rospy.logdebug(f"Actual Driver: Sending command Vx={target_vx:.2f}, Vy={target_vy:.2f}, Vw={target_vw:.2f}")
                self.controller.send_motion_command(target_vx, target_vy, target_vw)

            except AttributeError as e:
                 rospy.logerr(f"计算目标速度时出错，可能缺少运动学参数？ Error: {e}")
            except Exception as e:
                 rospy.logerr(f"处理命令时发生未知错误: {e}")

        elif self.driver_type == 'string_cmd':
            # --- String Command Driver Logic ---
            rospy.logdebug(f"String Driver: Received command {msg.motion_type} with RPM {msg.target_rpm}")

            # --- Stop Command Handling ---
            if msg.motion_type == 'stop':
                rospy.loginfo("String Driver: Received stop command.")
                # Stop any ongoing trajectory
                if self.trajectory_timer:
                    self.trajectory_timer.shutdown()
                    self.trajectory_timer = None
                    rospy.loginfo("String Driver: Trajectory timer stopped.")
                self.current_speed_profile = None # Clear profile
                # Send immediate stop command (Speed 0)
                # Use 'F' and configured speed mode, speed 0
                stop_cmd_str = f"$SPD,F,{self.string_cmd_speed_mode},0"
                if self.controller:
                    self.controller.send_string_command(stop_cmd_str)
                    rospy.loginfo(f"String Driver: Sent immediate stop command: {stop_cmd_str}")
                return # Stop processing further

            # --- Trajectory Generation ---
            try:
                # Use parameters from the message
                time_points, speed_profile_rpm = TrajectoryPlanner.generate_trapezoidal_profile(
                    target_speed=msg.target_rpm,
                    accel_time=msg.accel_time,
                    cruise_time=msg.cruise_time,
                    decel_time=msg.decel_time,
                    time_step=self.TRAJECTORY_TIME_STEP
                )

                if not speed_profile_rpm:
                    rospy.logwarn("String Driver: Generated empty speed profile. No action taken.")
                    return

                rospy.loginfo(f"String Driver: Generated trajectory with {len(speed_profile_rpm)} points.")

                # --- Start Trajectory Execution ---
                self.current_speed_profile = speed_profile_rpm
                self.current_motion_type = msg.motion_type
                self.trajectory_index = 0

                # Stop previous timer if running
                if self.trajectory_timer:
                    self.trajectory_timer.shutdown()
                    rospy.logdebug("String Driver: Shutting down previous trajectory timer.")

                # Start new timer
                self.trajectory_timer = rospy.Timer(
                    rospy.Duration(self.TRAJECTORY_TIME_STEP),
                    self.trajectory_step_callback,
                    oneshot=False # Keep running until stopped or finished
                )
                rospy.loginfo("String Driver: Trajectory execution started.")

            except Exception as e:
                rospy.logerr(f"String Driver: Error generating or starting trajectory: {e}")
                # Ensure timer is stopped on error
                if self.trajectory_timer:
                    self.trajectory_timer.shutdown()
                    self.trajectory_timer = None
                self.current_speed_profile = None


    def trajectory_step_callback(self, event):
        """Called periodically by the timer to send the next trajectory point as string command."""
        if self.controller is None or not self.controller.initialized:
            rospy.logerr("String Driver Callback: Controller not ready. Stopping timer.")
            if self.trajectory_timer: self.trajectory_timer.shutdown()
            self.trajectory_timer = None
            self.current_speed_profile = None
            return

        if self.current_speed_profile is None or self.trajectory_index >= len(self.current_speed_profile):
            rospy.loginfo("String Driver Callback: Trajectory finished or no profile. Stopping timer.")
            if self.trajectory_timer: self.trajectory_timer.shutdown()
            self.trajectory_timer = None
            self.current_speed_profile = None
            # Optionally send a final stop command here if needed, though profile should end at 0
            # stop_cmd_str = f"$SPD,F,{self.string_cmd_speed_mode},0"
            # self.controller.send_string_command(stop_cmd_str)
            return

        # Get current speed setpoint
        current_rpm = self.current_speed_profile[self.trajectory_index]
        speed_value = int(current_rpm) # Convert RPM to integer for the command

        # Map motion type and speed value to command parameters
        move_char = 'F' # Default
        motion = self.current_motion_type

        if motion == 'forward':
            move_char = 'F'
            # speed_value remains positive
        elif motion == 'backward':
            move_char = 'F'
            speed_value = -speed_value # Negative speed for backward
        elif motion == 'left':
            move_char = 'L'
            # speed_value remains positive (assuming L+ is left)
        elif motion == 'right':
            move_char = 'L'
            speed_value = -speed_value # Negative speed for right (assuming L- is right)
        elif motion == 'turn_ccw':
            move_char = 'R'
            # speed_value remains positive (assuming R+ is CCW)
        elif motion == 'turn_cw':
            move_char = 'R'
            speed_value = -speed_value # Negative speed for CW (assuming R- is CW)
        else:
            rospy.logwarn_throttle(5, f"String Driver Callback: Unknown motion type '{motion}'. Sending stop.")
            speed_value = 0
            move_char = 'F' # Send stop as forward 0 speed

        # Build command string
        # Speed mode is fixed based on the parameter read during init
        cmd_str = f"$SPD,{move_char},{self.string_cmd_speed_mode},{speed_value}"

        # Send command
        rospy.logdebug(f"String Driver Callback: Sending step {self.trajectory_index}: {cmd_str}")
        if not self.controller.send_string_command(cmd_str):
            rospy.logwarn(f"String Driver Callback: Failed to send command: {cmd_str}. Stopping timer.")
            if self.trajectory_timer: self.trajectory_timer.shutdown()
            self.trajectory_timer = None
            self.current_speed_profile = None
            return

        # Increment index for next step
        self.trajectory_index += 1


    def publish_status(self, event):
        """定期发布控制器状态"""
        if self.controller is None:
            # rospy.logwarn_throttle(5, "控制器未初始化，无法发布状态。")
            return

        status = MecanumStatus()
        status.header.stamp = rospy.Time.now()

        if self.driver_type == 'original':
            # --- 原始驱动状态获取 ---
            # TODO: 实现从 MecanumController/MotorController 获取状态
            # 目前 MecanumController 没有提供获取状态的方法
            status.current_state = "Unknown (Original Driver)"
            status.wheel_speeds = [0.0] * 4
            status.wheel_enabled = [self.controller.initialized] * 4 # 简单用初始化状态代替
            status.battery_voltage = 0.0 # 无法获取
            status.error_code = 0 # 无法获取

        elif self.driver_type == 'actual':
            # --- 新驱动状态获取与转换 ---
            try:
                parsed_status = self.controller.get_parsed_status()

                status.error_code = parsed_status.get('error_code', 0)
                # 假设 SOC 可以直接用作电压的替代或需要转换
                status.battery_voltage = parsed_status.get('soc', 0.0)

                # 状态判断
                if status.error_code != 0:
                    status.current_state = "Error"
                elif parsed_status.get('vehicle_speed', 0.0) > 0.01: # 使用反馈的速度判断
                    status.current_state = "Moving"
                else:
                    status.current_state = "Idle"

                # 估算轮速 (使用逆运动学)
                # 注意：这里使用反馈的整车速度 V (标量) 可能不准确，因为它没有方向信息
                # 更好的方法是使用最后发送的指令速度 (vx, vy, vw) 或解析更丰富的状态信息
                # 暂时使用反馈的 V 作为 vy，其他为0来估算
                vx_feedback = 0.0
                vy_feedback = parsed_status.get('vehicle_speed', 0.0) # 使用反馈的V作为Vy
                vw_feedback = 0.0 # 无法直接从 V 获取角速度

                lx = self.wheel_separation_length / 2.0
                ly = self.wheel_separation_width / 2.0
                r = self.wheel_radius
                if r > 0.001: # 避免除零
                    inv_r = 1.0 / r
                    # Mecanum inverse kinematics
                    w_lf = inv_r * (vx_feedback - vy_feedback - (lx + ly) * vw_feedback)
                    w_rf = inv_r * (vx_feedback + vy_feedback + (lx + ly) * vw_feedback)
                    w_lb = inv_r * (vx_feedback + vy_feedback - (lx + ly) * vw_feedback)
                    w_rb = inv_r * (vx_feedback - vy_feedback + (lx + ly) * vw_feedback)

                    # Convert rad/s to RPM
                    rpm_lf = w_lf * 60.0 / (2 * math.pi)
                    rpm_rf = w_rf * 60.0 / (2 * math.pi)
                    rpm_lb = w_lb * 60.0 / (2 * math.pi)
                    rpm_rb = w_rb * 60.0 / (2 * math.pi)
                    status.wheel_speeds = [rpm_lf, rpm_rf, rpm_lb, rpm_rb]
                else:
                    status.wheel_speeds = [0.0] * 4

                # 假设轮子使能状态，除非有特定错误
                # TODO: 根据 error_code 映射更精确的使能状态
                status.wheel_enabled = [status.error_code == 0] * 4

            except AttributeError as e:
                 rospy.logerr_throttle(5, f"获取或转换状态时出错，可能缺少运动学参数？ Error: {e}")
                 status.current_state = "Error (Status Conversion)"
                 status.wheel_speeds = [0.0] * 4
                 status.wheel_enabled = [False] * 4
            except Exception as e:
                 rospy.logerr_throttle(5, f"发布状态时发生未知错误: {e}")
                 status.current_state = "Error (Unknown)"
                 status.wheel_speeds = [0.0] * 4
                 status.wheel_enabled = [False] * 4


        elif self.driver_type == 'string_cmd':
            # --- String Command Driver Status ---
            # TODO: Implement status fetching from StringCommandController if needed
            status.current_state = "Executing (String Cmd)" if self.trajectory_timer and self.current_speed_profile else "Idle (String Cmd)"
            status.wheel_speeds = [0.0] * 4 # Placeholder
            status.wheel_enabled = [self.controller.initialized] * 4 # Placeholder
            status.battery_voltage = 0.0 # Placeholder
            status.error_code = 0 # Placeholder


        self.status_pub.publish(status)

    def shutdown(self):
        """关闭节点时的清理操作"""
        rospy.loginfo("正在关闭麦克纳姆轮控制器节点...")
        # Stop trajectory timer if running
        if self.trajectory_timer:
            self.trajectory_timer.shutdown()
            rospy.loginfo("Trajectory timer stopped during shutdown.")
        if hasattr(self, 'timer') and self.timer:
            self.timer.shutdown() # 停止定时器
        # Close the specific controller if it exists and has a close method
        if hasattr(self, 'controller') and self.controller and hasattr(self.controller, 'close'):
            self.controller.close()
        # Close the main CAN controller
        if hasattr(self, 'can_controller') and self.can_controller:
            self.can_controller.close()

if __name__ == '__main__':
    controller_node = MecanumControllerNode()
    rospy.on_shutdown(controller_node.shutdown)
    rospy.spin()
