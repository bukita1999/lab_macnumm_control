import can
import time
import logging
import platform
from enum import Enum, auto

# --- Configuration ---
CAN_INTERFACE = 'canalystii'
CAN_CHANNEL = 0
CAN_DEVICE = 0
CAN_BITRATE = 250000
SEND_INTERVAL_S = 0.015

class MoveType(Enum):
    FORWARD_BACKWARD = auto()
    LEFT_RIGHT = auto()
    ROTATE = auto()

    @staticmethod
    def from_char(c):
        c = c.strip().lower()
        if c == 'f':
            return MoveType.FORWARD_BACKWARD
        elif c == 'l':
            return MoveType.LEFT_RIGHT
        elif c == 'r':
            return MoveType.ROTATE
        else:
            raise ValueError(f"Unknown move type: {c}")

class SpeedMode(Enum):
    ONE_TENTH = auto()
    HALF = auto()
    FULL = auto()

    @staticmethod
    def from_char(c):
        c = c.strip().lower()
        if c == 'a':
            return SpeedMode.ONE_TENTH
        elif c == 'b':
            return SpeedMode.HALF
        elif c == 'c':
            return SpeedMode.FULL
        else:
            raise ValueError(f"Unknown speed mode: {c}")

class SpeedCommand:
    def __init__(self, move_type, speed_mode, speed_value):
        self.move_type = move_type
        self.speed_mode = speed_mode
        self.speed_value = int(speed_value)

    @staticmethod
    def from_string(cmd_str):
        # $SPD,<F/L/R>,<A/B/C>,<speed>
        parts = cmd_str.strip().split(',')
        if len(parts) != 4 or parts[0].strip().upper() != '$SPD':
            raise ValueError("Format: $SPD,<F/L/R>,<A/B/C>,<speed>")
        move_type = MoveType.from_char(parts[1])
        speed_mode = SpeedMode.from_char(parts[2])
        speed_value = int(parts[3])
        return SpeedCommand(move_type, speed_mode, speed_value)

    def to_string(self):
        # 生成标准字符串
        move_map = {MoveType.FORWARD_BACKWARD: 'F', MoveType.LEFT_RIGHT: 'L', MoveType.ROTATE: 'R'}
        speed_map = {SpeedMode.ONE_TENTH: 'A', SpeedMode.HALF: 'B', SpeedMode.FULL: 'C'}
        return f"$SPD,{move_map[self.move_type]},{speed_map[self.speed_mode]},{self.speed_value}"

    def build_key(self):
        key = 0
        if self.move_type == MoveType.FORWARD_BACKWARD:
            key |= 0x1
        elif self.move_type == MoveType.LEFT_RIGHT:
            key |= 0x2
        elif self.move_type == MoveType.ROTATE:
            key |= 0x4
        key |= 0x1000  # 左侧使能
        if self.speed_mode == SpeedMode.ONE_TENTH:
            key |= 0x10000
        elif self.speed_mode == SpeedMode.HALF:
            key |= 0x20000
        elif self.speed_mode == SpeedMode.FULL:
            key |= 0x40000
        # 速度模式
        key |= 0x100000
        return key

    def build_speed_data(self, speed_val=None):
        # 速度帧格式 0x7a5，2字节Vx，2字节Vy，2字节Vw，2字节Vu，均为S16，小端
        vx = vy = vw = vu = 0
        val = self.speed_value if speed_val is None else int(speed_val)
        if self.move_type == MoveType.FORWARD_BACKWARD:
            vx = val
        elif self.move_type == MoveType.LEFT_RIGHT:
            vy = val
        elif self.move_type == MoveType.ROTATE:
            vw = val
        def encode(v):
            if v >= 0:
                return v.to_bytes(2, byteorder='little', signed=True)
            else:
                vv = (~(abs(v) - 1)) & 0xFFFF
                return vv.to_bytes(2, byteorder='little', signed=False)
        vx_bytes = encode(vx)
        vy_bytes = encode(vy)
        vw_bytes = encode(vw)
        vu_bytes = encode(vu)
        return list(vx_bytes + vy_bytes + vw_bytes + vu_bytes)

    def build_spd_setup_data(self):
        key = self.build_key()
        key_bytes = key.to_bytes(4, byteorder='big', signed=False)
        pulse_bytes = (0).to_bytes(4, byteorder='little', signed=False)
        return list(key_bytes + pulse_bytes)

    def print_info(self, speed_val=None):
        print("Command to be sent:")
        print(f"  Move Type:   {self.move_type.name} (F=Forward/Backward, L=Left/Right, R=Rotate)")
        print(f"  Speed Mode:  {self.speed_mode.name} (A=1/10, B=Half, C=Full)")
        print(f"  Speed:       {self.speed_value if speed_val is None else speed_val}")
        print(f"  0x75A Data:  {' '.join(f'{b:02X}' for b in self.build_spd_setup_data())}")
        print(f"  0x7A5 Data:  {' '.join(f'{b:02X}' for b in self.build_speed_data(self.speed_value if speed_val is None else speed_val))}")

    def send(self, bus, duration=1.0):
        # 发送速度模式命令
        setup_data = self.build_spd_setup_data()
        speed_data = self.build_speed_data()
        start_time = time.time()
        while time.time() - start_time < duration:
            send_can_message(bus, 0x75A, setup_data)
            time.sleep(SEND_INTERVAL_S)
            send_can_message(bus, 0x7A5, speed_data)
            time.sleep(SEND_INTERVAL_S)
        # 发送停止命令
        stop_speed_data = self.build_speed_data(0)
        self.print_info(speed_val=0)
        send_can_message(bus, 0x75A, setup_data)
        time.sleep(SEND_INTERVAL_S)
        send_can_message(bus, 0x7A5, stop_speed_data)
        print("Stop command sent.")

def send_can_message(bus, arbitration_id, data, is_extended=False):
    message = can.Message(
        arbitration_id=arbitration_id,
        data=bytearray(data),
        is_extended_id=is_extended
    )
    try:
        bus.send(message)
        return True
    except can.CanError as e:
        logging.error(f"Error sending message (ID: {arbitration_id:#05x}): {e}")
        return False
    except Exception as e:
        logging.error(f"Unexpected error during send (ID: {arbitration_id:#05x}): {e}")
        return False

class SpeedCommandBuilder:
    @staticmethod
    def ask_and_build():
        print("=== Build Speed Command ===")
        print("Select move type: [F]orward/Backward, [L]eft/Right, [R]otate")
        while True:
            move = input("Move type (F/L/R): ").strip().upper()
            if move in ['F', 'L', 'R']:
                break
            print("Invalid input. Please enter F, L, or R.")
        print("Select speed mode: [A]=1/10, [B]=Half, [C]=Full")
        while True:
            speed_mode = input("Speed mode (A/B/C): ").strip().upper()
            if speed_mode in ['A', 'B', 'C']:
                break
            print("Invalid input. Please enter A, B, or C.")
        while True:
            try:
                speed_val = int(input("Speed value (integer, e.g. 30): ").strip())
                break
            except ValueError:
                print("Invalid input. Please enter an integer.")
        cmd = SpeedCommand(
            MoveType.from_char(move),
            SpeedMode.from_char(speed_mode),
            speed_val
        )
        print("Generated command string:", cmd.to_string())
        return cmd

def print_usage():
    print("=== CAN Speed Control Command Line Usage ===")
    print("Format: $SPD,<F/L/R>,<A/B/C>,<speed>")
    print("  <F/L/R>: F=Forward/Backward, L=Left/Right, R=Rotate")
    print("  <A/B/C>: A=1/10 speed, B=Half speed, C=Full speed")
    print("  <speed>: Run speed (integer, e.g. 30)")
    print("Example: $SPD,F,A,30")
    print("===============================================")

if __name__ == "__main__":
    logging.info("--- Python-CAN CANalyst-II Speed Control (OOP, Interactive) ---")
    print_usage()
    check_canalystii_setup()
    cmd = SpeedCommandBuilder.ask_and_build()
    cmd.print_info()
    ans = input("Type y to confirm, any other key to cancel: ").strip().lower()
    if ans != 'y':
        print("Cancelled.")
        exit(0)
    with can.Bus(interface=CAN_INTERFACE,
                 channel=CAN_CHANNEL,
                 device=CAN_DEVICE,
                 bitrate=CAN_BITRATE) as bus:
        logging.info(f"Successfully connected to CAN bus: {bus.channel_info}")
        cmd.send(bus, duration=1.0)