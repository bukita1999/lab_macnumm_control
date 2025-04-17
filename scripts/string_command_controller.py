#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import can
import time
import logging
import math
from enum import Enum, auto

# --- Constants ---
SEND_INTERVAL_S = 0.015 # 两次 CAN 帧之间的最小间隔

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
            raise ValueError(f"Unknown move type char: {c}")

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
            raise ValueError(f"Unknown speed mode char: {c}")

class SpeedCommand:
    """Helper class to parse string command and build CAN data."""
    def __init__(self, move_type, speed_mode, speed_value):
        self.move_type = move_type
        self.speed_mode = speed_mode
        self.speed_value = int(speed_value) # Ensure integer

    @staticmethod
    def from_string(cmd_str):
        # $SPD,<F/L/R>,<A/B/C>,<speed>
        parts = cmd_str.strip().split(',')
        if len(parts) != 4 or parts[0].strip().upper() != '$SPD':
            raise ValueError("Format: $SPD,<F/L/R>,<A/B/C>,<speed>")
        move_type = MoveType.from_char(parts[1])
        speed_mode = SpeedMode.from_char(parts[2])
        speed_value = int(parts[3]) # Ensure integer
        return SpeedCommand(move_type, speed_mode, speed_value)

    def build_key(self):
        key = 0
        if self.move_type == MoveType.FORWARD_BACKWARD:
            key |= 0x1
        elif self.move_type == MoveType.LEFT_RIGHT:
            key |= 0x2
        elif self.move_type == MoveType.ROTATE:
            key |= 0x4
        key |= 0x1000  # 左侧使能 (Assuming this is always needed)

        if self.speed_mode == SpeedMode.ONE_TENTH:
            key |= 0x10000
        elif self.speed_mode == SpeedMode.HALF:
            key |= 0x20000
        elif self.speed_mode == SpeedMode.FULL:
            key |= 0x40000

        key |= 0x100000 # 速度模式 (Assuming this is always needed)
        return key

    def build_speed_data(self):
        # 速度帧格式 0x7a5，2字节Vx，2字节Vy，2字节Vw，2字节Vu，均为S16，小端
        vx = vy = vw = vu = 0
        val = self.speed_value

        # Map F/L/R to Vx/Vy/Vw based on the command structure
        if self.move_type == MoveType.FORWARD_BACKWARD:
            # Assuming 'F' maps to Y-axis speed in the robot's frame
            vy = val
        elif self.move_type == MoveType.LEFT_RIGHT:
            # Assuming 'L' maps to X-axis speed in the robot's frame
            vx = val # Note: Positive value for 'L' might mean move left, adjust sign if needed
        elif self.move_type == MoveType.ROTATE:
            # Assuming 'R' maps to rotational speed W
            vw = val

        def encode(v):
            # Handles negative numbers correctly for signed 16-bit little-endian
            return v.to_bytes(2, byteorder='little', signed=True)

        vx_bytes = encode(vx)
        vy_bytes = encode(vy)
        vw_bytes = encode(vw)
        vu_bytes = encode(vu) # vu is always 0 based on reference code
        return list(vx_bytes + vy_bytes + vw_bytes + vu_bytes)

    def build_spd_setup_data(self):
        key = self.build_key()
        key_bytes = key.to_bytes(4, byteorder='big', signed=False)
        pulse_bytes = (0).to_bytes(4, byteorder='little', signed=False) # Pulse is always 0
        return list(key_bytes + pulse_bytes)

def send_can_message(bus, arbitration_id, data, is_extended=False):
    """Sends a single CAN message."""
    if bus is None:
        logging.error("CAN bus is not initialized.")
        return False
    message = can.Message(
        arbitration_id=arbitration_id,
        data=bytearray(data),
        is_extended_id=is_extended
    )
    try:
        bus.send(message)
        # logging.debug(f"Sent CAN ID: {arbitration_id:#05x} Data: {' '.join(f'{b:02X}' for b in data)}")
        return True
    except can.CanError as e:
        logging.error(f"Error sending message (ID: {arbitration_id:#05x}): {e}")
        return False
    except Exception as e:
        logging.error(f"Unexpected error during send (ID: {arbitration_id:#05x}): {e}")
        return False

class StringCommandController:
    """
    Controls the robot using $SPD string commands over CAN.
    Handles the conversion from string to CAN frames and sending.
    """
    def __init__(self, can_controller, speed_mode_char='A'):
        """
        Initializes the controller.
        :param can_controller: An instance of PythonCANController (or similar with a 'bus' attribute).
        :param speed_mode_char: The default speed mode ('A', 'B', or 'C').
        """
        self.can_controller = can_controller
        self.bus = getattr(can_controller, 'bus', None) # Get the underlying python-can bus object
        self.initialized = False
        try:
            self.default_speed_mode = SpeedMode.from_char(speed_mode_char)
            logging.info(f"StringCommandController initialized with speed mode: {self.default_speed_mode.name}")
        except ValueError as e:
            logging.error(f"Invalid speed_mode_char '{speed_mode_char}': {e}. Defaulting to ONE_TENTH ('A').")
            self.default_speed_mode = SpeedMode.ONE_TENTH

    def initialize(self):
        """Checks if the CAN bus is ready."""
        if self.bus and self.can_controller.connected:
            logging.info("StringCommandController initialized successfully (CAN bus connected).")
            self.initialized = True
            return True
        else:
            logging.error("StringCommandController initialization failed: CAN bus not connected or not found.")
            self.initialized = False
            return False

    def send_string_command(self, cmd_str):
        """
        Parses a $SPD string command, builds CAN frames, and sends them once. Non-blocking.
        :param cmd_str: The command string, e.g., "$SPD,F,A,100".
        :return: True if both messages were sent successfully, False otherwise.
        """
        if not self.initialized:
            logging.error("Controller not initialized. Cannot send command.")
            return False

        try:
            cmd = SpeedCommand.from_string(cmd_str)
            # Override speed mode if necessary (or keep it fixed based on init?)
            # For now, let the command string dictate the speed mode.
            # If you want to force the mode from config, modify cmd.speed_mode here.
            # cmd.speed_mode = self.default_speed_mode

            setup_data = cmd.build_spd_setup_data()
            speed_data = cmd.build_speed_data()

            logging.debug(f"Sending String Command: {cmd_str}")
            logging.debug(f"  0x75A Data: {' '.join(f'{b:02X}' for b in setup_data)}")
            logging.debug(f"  0x7A5 Data: {' '.join(f'{b:02X}' for b in speed_data)}")

            # Send setup frame
            success1 = send_can_message(self.bus, 0x75A, setup_data)
            time.sleep(SEND_INTERVAL_S) # Small delay between frames might be necessary
            # Send speed frame
            success2 = send_can_message(self.bus, 0x7A5, speed_data)

            return success1 and success2

        except ValueError as e:
            logging.error(f"Invalid command string '{cmd_str}': {e}")
            return False
        except Exception as e:
            logging.error(f"Error processing or sending command '{cmd_str}': {e}")
            return False

    def close(self):
        """Cleanup resources (although CAN bus closing is handled by can_controller)."""
        logging.info("StringCommandController closing.")
        # No specific resources to close here as CAN bus is managed externally
        pass

    # --- Optional: Status Handling ---
    # def _can_message_callback(self, msg):
    #     """Placeholder for receiving CAN messages if status feedback is needed."""
    #     # Parse messages from the robot (e.g., actual speed, errors)
    #     pass

    # def get_status(self):
    #     """Placeholder for returning parsed status information."""
    #     # Return a dictionary or object with current status
    #     return {"state": "unknown"}