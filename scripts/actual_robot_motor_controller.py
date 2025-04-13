#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import time
from threading import Lock

# ==============================================================================
# 新协议 CAN ID 定义 (建议后续移至配置文件或ROS参数)
# ==============================================================================
# 遥控器 -> 主控
REMOTE_KEY_PULSE_ID = 0x75a  # 按键和脉冲帧
REMOTE_MOTION_CMD_ID = 0x7a5 # 运动速度指令帧 (Vx, Vy, Vw, Vu)
# REMOTE_PLATFORM_CMD_ID = 0xXXX # 平台控制指令帧 (Vh, Vp, Vr) - 文档未指定ID，暂不实现

# 主控 -> 外部/遥控器
MASTER_STATUS_BASIC_ID = 0x65a  # 基本状态 (Soc, V, Type, UDSta)
MASTER_STATUS_FAULT_ID = 0x65b  # 故障、压力、安全触边 (Error, Press, In)
MASTER_STATUS_POSE_H_ID = 0x65c # 车辆姿态和平台高度 (Acx, Acy, H)
MASTER_STATUS_PLAT_W_ID = 0x65d # 平台姿态和承重 (Apx, Apy, W)
MASTER_STATUS_CALIB_ROLL_ID = 0x65e # 平台侧倾标定状态
MASTER_STATUS_CALIB_PITCH_ID = 0x65f # 平台俯仰标定状态

# ==============================================================================
# 数据转换常量
# ==============================================================================
# 速度指令转换 (假设遥控器发送的速度值范围对应实际速度，需要标定)
# 例如，如果协议中的 S16 值 32767 对应 1.0 m/s
VELOCITY_CMD_SCALE = 32767.0 / 1.0  # S16_Value / (m/s or rad/s) - 需要根据实际情况调整

# 状态反馈转换
SOC_SCALE = 0.1  # 单位: %
PRESSURE_SCALE = 0.1 # 单位: Mpa
ANGLE_SCALE = 0.1 # 单位: 度
HEIGHT_SCALE = 1.0 # 单位: mm
WEIGHT_SCALE = 1.0 # 单位: Kg

class ActualRobotMotorController:
    """
    用于控制遵循自定义CAN协议的实际机器人的类。
    直接发送整车运动指令 (Vx, Vy, Vw) 并接收整车状态。
    """
    def __init__(self, can_controller):
        """
        初始化控制器。
        :param can_controller: 用于发送和接收CAN消息的底层控制器实例。
        """
        self.can = can_controller
        self.status_lock = Lock()
        self.last_status = {
            'soc': 0.0,
            'vehicle_speed': 0.0,
            'control_type': 0,
            'platform_lifting': False,
            'error_code': 0,
            'pressure': 0.0,
            'safety_edge': 0,
            'vehicle_roll': 0.0,
            'vehicle_pitch': 0.0,
            'platform_height': 0,
            'platform_roll': 0.0,
            'platform_pitch': 0.0,
            'platform_weight': 0,
            'calib_roll_flag': 0,
            'calib_pitch_flag': 0,
            'last_update_time': 0.0
        }
        # TODO: 启动一个线程或使用回调来处理接收到的CAN消息
        # self.can.add_listener(self._can_message_callback)

    def _can_message_callback(self, msg):
        """
        处理接收到的CAN消息的回调函数。
        需要根据底层 can_controller 的实现来适配。
        """
        timestamp = time.time()
        can_id = msg.arbitration_id
        data = msg.data

        with self.status_lock:
            self.last_status['last_update_time'] = timestamp
            try:
                if can_id == MASTER_STATUS_BASIC_ID and len(data) == 8:
                    # 解析 0x65a: Soc(U16), V(f32), Type(U8), UDSta(U8)
                    # 注意：struct 使用 '>' 表示大端序
                    soc_raw, v_raw = struct.unpack('>Hf', data[0:6]) # U16, f32
                    type_raw, udsta_raw = struct.unpack('>BB', data[6:8]) # U8, U8
                    self.last_status['soc'] = soc_raw * SOC_SCALE
                    self.last_status['vehicle_speed'] = v_raw # 直接使用float值
                    self.last_status['control_type'] = type_raw
                    self.last_status['platform_lifting'] = (udsta_raw == 1)
                    # rospy.logdebug(f"Parsed 0x65a: Soc={self.last_status['soc']}, V={self.last_status['vehicle_speed']}")

                elif can_id == MASTER_STATUS_FAULT_ID and len(data) == 8:
                    # 解析 0x65b: Error(U32), Press(U16), In(U16)
                    error_raw, press_raw, in_raw = struct.unpack('>IHH', data) # U32, U16, U16
                    self.last_status['error_code'] = error_raw
                    self.last_status['pressure'] = press_raw * PRESSURE_SCALE
                    self.last_status['safety_edge'] = in_raw
                    if error_raw != 0:
                        rospy.logwarn(f"Received Error Code: {error_raw:#010x}")
                    # rospy.logdebug(f"Parsed 0x65b: Error={self.last_status['error_code']:#x}, Press={self.last_status['pressure']}, In={self.last_status['safety_edge']}")

                elif can_id == MASTER_STATUS_POSE_H_ID and len(data) >= 6: # 允许数据长度大于6，忽略后面未用字节
                    # 解析 0x65c: Acx(S16), Acy(S16), H(U16)
                    acx_raw, acy_raw, h_raw = struct.unpack('>hhH', data[0:6]) # S16, S16, U16
                    self.last_status['vehicle_roll'] = acx_raw * ANGLE_SCALE
                    self.last_status['vehicle_pitch'] = acy_raw * ANGLE_SCALE
                    self.last_status['platform_height'] = h_raw * HEIGHT_SCALE
                    # rospy.logdebug(f"Parsed 0x65c: Acx={self.last_status['vehicle_roll']}, Acy={self.last_status['vehicle_pitch']}, H={self.last_status['platform_height']}")

                elif can_id == MASTER_STATUS_PLAT_W_ID and len(data) >= 6:
                    # 解析 0x65d: Apx(S16), Apy(S16), W(U16)
                    apx_raw, apy_raw, w_raw = struct.unpack('>hhH', data[0:6]) # S16, S16, U16
                    self.last_status['platform_roll'] = apx_raw * ANGLE_SCALE
                    self.last_status['platform_pitch'] = apy_raw * ANGLE_SCALE
                    self.last_status['platform_weight'] = w_raw * WEIGHT_SCALE
                    # rospy.logdebug(f"Parsed 0x65d: Apx={self.last_status['platform_roll']}, Apy={self.last_status['platform_pitch']}, W={self.last_status['platform_weight']}")

                elif can_id == MASTER_STATUS_CALIB_ROLL_ID and len(data) >= 1:
                    # 解析 0x65e: Flag(U8)
                    flag_raw, = struct.unpack('>B', data[0:1])
                    self.last_status['calib_roll_flag'] = flag_raw
                    # rospy.logdebug(f"Parsed 0x65e: Flag={flag_raw}")

                elif can_id == MASTER_STATUS_CALIB_PITCH_ID and len(data) >= 1:
                    # 解析 0x65f: Flag(U8)
                    flag_raw, = struct.unpack('>B', data[0:1])
                    self.last_status['calib_pitch_flag'] = flag_raw
                    # rospy.logdebug(f"Parsed 0x65f: Flag={flag_raw}")

            except struct.error as e:
                rospy.logerr(f"Error unpacking CAN message ID {can_id:#x}: {e}, Data: {data.hex()}")
            except Exception as e:
                 rospy.logerr(f"Error processing CAN message ID {can_id:#x}: {e}")


    def send_motion_command(self, vx, vy, vw, vu=0.0):
        """
        发送整车运动指令 (对应 CAN ID 0x7a5)。
        :param vx: X方向速度 (m/s)
        :param vy: Y方向速度 (m/s)
        :param vw: W方向角速度 (rad/s) - 需要确认单位，文档是Vw，假设是角速度
        :param vu: U方向速度 (m/s, 备用)
        :return: True 如果发送成功, False otherwise.
        """
        try:
            # 将浮点速度转换为 S16 整数
            # 注意：需要根据实际标定调整 VELOCITY_CMD_SCALE
            # 注意：需要限制速度在 S16 范围内 (-32768 到 32767)
            vx_cmd = max(-32768, min(32767, int(vx * VELOCITY_CMD_SCALE)))
            vy_cmd = max(-32768, min(32767, int(vy * VELOCITY_CMD_SCALE)))
            vw_cmd = max(-32768, min(32767, int(vw * VELOCITY_CMD_SCALE))) # 假设 Vw 对应角速度
            vu_cmd = max(-32768, min(32767, int(vu * VELOCITY_CMD_SCALE)))

            # 使用 '>' 表示大端序打包 S16 (signed short)
            data = struct.pack('>hhhh', vx_cmd, vy_cmd, vw_cmd, vu_cmd)

            # rospy.logdebug(f"Sending motion command: Vx={vx:.2f}({vx_cmd}), Vy={vy:.2f}({vy_cmd}), Vw={vw:.2f}({vw_cmd}), Vu={vu:.2f}({vu_cmd})")
            return self.can.send_message(REMOTE_MOTION_CMD_ID, list(data))

        except Exception as e:
            rospy.logerr(f"Error sending motion command: {e}")
            return False

    def send_key_pulse_command(self, key_state, pulse_value):
        """
        发送按键和手轮脉冲指令 (对应 CAN ID 0x75a)。
        :param key_state: 32位按键状态 (U32)
        :param pulse_value: 32位手轮脉冲值 (U32)
        :return: True 如果发送成功, False otherwise.
        """
        try:
            # 使用 '>' 表示大端序打包 U32 (unsigned int)
            data = struct.pack('>II', key_state, pulse_value)
            # rospy.logdebug(f"Sending key/pulse command: Key={key_state:#010x}, Pulse={pulse_value}")
            return self.can.send_message(REMOTE_KEY_PULSE_ID, list(data))
        except Exception as e:
            rospy.logerr(f"Error sending key/pulse command: {e}")
            return False

    def get_parsed_status(self):
        """
        获取最新解析的状态信息字典。
        """
        with self.status_lock:
            # 返回状态字典的深拷贝，防止外部修改
            return self.last_status.copy()

    def initialize(self):
        """
        初始化控制器（如果需要特定的初始化序列）。
        对于这个协议，可能不需要像 CANopen 那样复杂的初始化。
        但可以预留，例如用于检查通信或发送初始状态。
        """
        rospy.loginfo("Initializing ActualRobotMotorController...")
        # 可以在这里添加检查CAN总线或发送特定启动消息的逻辑
        # 例如，尝试发送一个零速指令
        if not self.send_motion_command(0.0, 0.0, 0.0, 0.0):
             rospy.logwarn("Initial zero speed command send failed.")
             # 根据需要决定是否认为初始化失败
        rospy.loginfo("ActualRobotMotorController initialized (basic check).")
        return True # 假设基本初始化成功

    def close(self):
        """
        关闭控制器，执行必要的清理操作。
        """
        rospy.loginfo("Closing ActualRobotMotorController...")
        # 发送停止指令
        self.send_motion_command(0.0, 0.0, 0.0, 0.0)
        # TODO: 停止监听CAN消息
        # self.can.remove_listener(self._can_message_callback)
        rospy.loginfo("ActualRobotMotorController closed.")

# Example usage (for testing purposes, assuming a mock can_controller)
if __name__ == '__main__':
    class MockCanController:
        def send_message(self, can_id, data):
            print(f"Mock CAN Send: ID={can_id:#x}, Data={bytes(data).hex()}")
            return True
        # Add mock listener methods if needed

    rospy.init_node('actual_robot_test', anonymous=True, log_level=rospy.DEBUG)
    mock_can = MockCanController()
    controller = ActualRobotMotorController(mock_can)

    if controller.initialize():
        print("Initialization successful.")
        controller.send_motion_command(vx=0.5, vy=0.0, vw=0.1)
        time.sleep(1)
        controller.send_key_pulse_command(key_state=0x100, pulse_value=12345) # Example key state
        time.sleep(1)
        controller.send_motion_command(vx=0.0, vy=0.0, vw=0.0)

        # Mock receiving messages
        class MockCanMessage:
            def __init__(self, can_id, data):
                self.arbitration_id = can_id
                self.data = bytes(data)

        # Mock 0x65a message (Soc=85.5%, V=0.2 m/s, Type=1, UDSta=0)
        # Soc = 855 (0x0357), V = 0.2 (0x3e4ccccd), Type=1, UDSta=0
        msg_65a = MockCanMessage(MASTER_STATUS_BASIC_ID, [0x03, 0x57, 0x3e, 0x4c, 0xcc, 0xcd, 0x01, 0x00])
        controller._can_message_callback(msg_65a)

        # Mock 0x65b message (Error=0x20, Press=10.0 Mpa, In=5)
        # Error=0x20, Press=100 (0x0064), In=5 (0x0005)
        msg_65b = MockCanMessage(MASTER_STATUS_FAULT_ID, [0x00, 0x00, 0x00, 0x20, 0x00, 0x64, 0x00, 0x05])
        controller._can_message_callback(msg_65b)

        status = controller.get_parsed_status()
        print("\nLatest Status:")
        import json
        print(json.dumps(status, indent=2))

        controller.close()
