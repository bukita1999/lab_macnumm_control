#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import can
import time
import struct
import threading
import logging
import argparse

# ==============================================================================
# CAN ID 定义 (从 actual_robot_motor_controller.py 复制)
# ==============================================================================
REMOTE_KEY_PULSE_ID = 0x75a
REMOTE_MOTION_CMD_ID = 0x7a5
MASTER_STATUS_BASIC_ID = 0x65a
MASTER_STATUS_FAULT_ID = 0x65b
MASTER_STATUS_POSE_H_ID = 0x65c
MASTER_STATUS_PLAT_W_ID = 0x65d
MASTER_STATUS_CALIB_ROLL_ID = 0x65e
MASTER_STATUS_CALIB_PITCH_ID = 0x65f

# ==============================================================================
# 数据转换常量 (从 actual_robot_motor_controller.py 复制)
# ==============================================================================
VELOCITY_CMD_SCALE = 32767.0 / 1.0 # S16_Value / (m/s or rad/s) - 需要标定
SOC_SCALE = 0.1
PRESSURE_SCALE = 0.1
ANGLE_SCALE = 0.1
HEIGHT_SCALE = 1.0
WEIGHT_SCALE = 1.0

# ==============================================================================
# 日志配置
# ==============================================================================
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# ==============================================================================
# 全局变量存储最新状态
# ==============================================================================
latest_status = {}
status_lock = threading.Lock()
stop_event = threading.Event()

# ==============================================================================
# CAN 消息处理
# ==============================================================================
def parse_can_message(msg):
    """解析接收到的CAN消息并更新全局状态"""
    global latest_status
    can_id = msg.arbitration_id
    data = msg.data
    timestamp = msg.timestamp # 使用消息自带的时间戳

    with status_lock:
        latest_status['last_update_time'] = timestamp
        try:
            if can_id == MASTER_STATUS_BASIC_ID and len(data) == 8:
                soc_raw, v_raw = struct.unpack('>Hf', data[0:6])
                type_raw, udsta_raw = struct.unpack('>BB', data[6:8])
                latest_status['soc'] = soc_raw * SOC_SCALE
                latest_status['vehicle_speed'] = v_raw
                latest_status['control_type'] = type_raw
                latest_status['platform_lifting'] = (udsta_raw == 1)
                logging.debug(f"Parsed 0x65a: Soc={latest_status['soc']:.1f}%, V={latest_status['vehicle_speed']:.2f} m/s")

            elif can_id == MASTER_STATUS_FAULT_ID and len(data) == 8:
                error_raw, press_raw, in_raw = struct.unpack('>IHH', data)
                latest_status['error_code'] = error_raw
                latest_status['pressure'] = press_raw * PRESSURE_SCALE
                latest_status['safety_edge'] = in_raw
                if error_raw != 0:
                    logging.warning(f"Received Error Code: {error_raw:#010x}")
                logging.debug(f"Parsed 0x65b: Error={latest_status['error_code']:#x}, Press={latest_status['pressure']:.1f} Mpa")

            # 可以根据需要添加其他状态帧的解析
            # elif can_id == MASTER_STATUS_POSE_H_ID and len(data) >= 6: ...
            # elif can_id == MASTER_STATUS_PLAT_W_ID and len(data) >= 6: ...

        except struct.error as e:
            logging.error(f"Error unpacking CAN message ID {can_id:#x}: {e}, Data: {data.hex()}")
        except Exception as e:
             logging.error(f"Error processing CAN message ID {can_id:#x}: {e}")

def can_receiver_thread(bus):
    """持续接收CAN消息的线程"""
    logging.info("CAN receiver thread started.")
    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.5) # 设置超时以允许线程退出检查
            if msg:
                parse_can_message(msg)
        except can.CanError as e:
            logging.error(f"CAN receive error: {e}")
            # 可以在这里添加重新连接逻辑或直接退出
            break
        except Exception as e:
            logging.error(f"Unexpected error in receiver thread: {e}")
    logging.info("CAN receiver thread stopped.")

def send_motion_command(bus, vx, vy, vw, vu=0.0):
    """发送整车运动指令"""
    try:
        vx_cmd = max(-32768, min(32767, int(vx * VELOCITY_CMD_SCALE)))
        vy_cmd = max(-32768, min(32767, int(vy * VELOCITY_CMD_SCALE)))
        vw_cmd = max(-32768, min(32767, int(vw * VELOCITY_CMD_SCALE)))
        vu_cmd = max(-32768, min(32767, int(vu * VELOCITY_CMD_SCALE)))
        data = struct.pack('>hhhh', vx_cmd, vy_cmd, vw_cmd, vu_cmd)
        message = can.Message(arbitration_id=REMOTE_MOTION_CMD_ID, data=data, is_extended_id=False)
        bus.send(message)
        logging.debug(f"Sent motion command: Vx={vx:.2f}, Vy={vy:.2f}, Vw={vw:.2f}")
        return True
    except Exception as e:
        logging.error(f"Error sending motion command: {e}")
        return False

def get_latest_status():
    """获取最新状态的副本"""
    with status_lock:
        return latest_status.copy()

# ==============================================================================
# 自检主逻辑
# ==============================================================================
def run_self_check(bus):
    """执行自检序列"""
    results = {"initial_check": False, "motion_test": False}
    logging.info("--- Starting Robot Self-Check ---")

    # 1. 发送零速指令并检查初始状态
    logging.info("Step 1: Sending zero velocity and checking initial status...")
    if not send_motion_command(bus, 0.0, 0.0, 0.0):
        logging.error("Failed to send initial zero velocity command.")
        return results
    time.sleep(1.0) # 等待状态更新

    status = get_latest_status()
    if not status:
        logging.warning("No status received after initial command.")
    else:
        logging.info(f"Initial Status: SOC={status.get('soc', 'N/A'):.1f}%, Error={status.get('error_code', 'N/A'):#x}, Speed={status.get('vehicle_speed', 'N/A'):.2f} m/s")
        initial_error = status.get('error_code', -1)
        if initial_error == 0:
            logging.info("Initial error check: OK")
            results["initial_check"] = True
        else:
            logging.error(f"Initial error check: FAILED (Error code: {initial_error:#x})")

    if not results["initial_check"]:
         logging.warning("Skipping motion test due to initial check failure.")
         return results

    # 2. 执行短暂的向前运动测试
    logging.info("Step 2: Performing short forward motion test...")
    test_speed = 0.1 # m/s
    test_duration = 0.5 # seconds

    logging.info(f"Sending forward command (Vy={test_speed} m/s) for {test_duration}s...")
    if not send_motion_command(bus, 0.0, test_speed, 0.0):
         logging.error("Failed to send forward motion command.")
         return results

    time.sleep(test_duration)

    logging.info("Sending zero velocity command...")
    if not send_motion_command(bus, 0.0, 0.0, 0.0):
        logging.error("Failed to send stop command after motion test.")
        return results

    time.sleep(1.0) # 等待速度稳定和状态更新

    status = get_latest_status()
    if not status:
        logging.warning("No status received after motion test.")
    else:
        logging.info(f"Status after motion test: SOC={status.get('soc', 'N/A'):.1f}%, Error={status.get('error_code', 'N/A'):#x}, Speed={status.get('vehicle_speed', 'N/A'):.2f} m/s")
        final_error = status.get('error_code', -1)
        final_speed = status.get('vehicle_speed', 999)

        if final_error == 0 and abs(final_speed) < 0.05: # 检查错误码和速度是否归零（允许小误差）
            logging.info("Motion test: OK (No errors, speed returned to near zero)")
            results["motion_test"] = True
        else:
            logging.error(f"Motion test: FAILED (Error: {final_error:#x}, Final Speed: {final_speed:.2f} m/s)")

    logging.info("--- Robot Self-Check Finished ---")
    return results

# ==============================================================================
# 主程序入口
# ==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Robot CAN Self-Check Script')
    parser.add_argument('--interface', type=str, default='socketcan', help='CAN interface type (e.g., socketcan, pcan, vector)')
    parser.add_argument('--channel', type=str, default='can0', help='CAN channel name (e.g., can0, PCAN_USBBUS1)')
    parser.add_argument('--bitrate', type=int, default=500000, help='CAN bitrate')
    args = parser.parse_args()

    bus = None
    receiver = None
    try:
        logging.info(f"Attempting to connect to CAN bus: interface={args.interface}, channel={args.channel}, bitrate={args.bitrate}")
        bus = can.interface.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
        logging.info("CAN bus connected successfully.")

        # 启动接收线程
        receiver = threading.Thread(target=can_receiver_thread, args=(bus,), daemon=True)
        receiver.start()

        # 运行自检
        check_results = run_self_check(bus)
        logging.info(f"Self-Check Results: {check_results}")

    except can.CanError as e:
        logging.error(f"Failed to connect or communicate via CAN: {e}")
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
    finally:
        logging.info("Shutting down...")
        stop_event.set() # 通知接收线程停止
        if receiver:
            receiver.join(timeout=1.0) # 等待接收线程结束
        if bus:
            bus.shutdown()
            logging.info("CAN bus shutdown.")
        logging.info("Script finished.")
