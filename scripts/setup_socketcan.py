#!/usr/bin/env python3
# setup_socketcan.py - Configure and set up SocketCAN interface

import os
import subprocess
import argparse

def setup_socketcan(interface="can0", bitrate=500000):
    """Set up SocketCAN interface
    
    Args:
        interface: Interface name (can0, can1, etc)
        bitrate: Bitrate in bps
    """
    print(f"设置SocketCAN接口: {interface}, 波特率: {bitrate}")
    
    # Down the interface first if it exists
    try:
        subprocess.run(["sudo", "ip", "link", "set", "down", interface])
        print(f"接口 {interface} 已关闭")
    except:
        print(f"接口 {interface} 不存在或无法关闭")
    
    # Set up the interface
    try:
        # Set interface type to can
        subprocess.run(["sudo", "ip", "link", "set", interface, "type", "can"])
        
        # Set bitrate
        subprocess.run(["sudo", "ip", "link", "set", interface, "up", "type", "can", "bitrate", str(bitrate)])
        
        print(f"接口 {interface} 已设置为 {bitrate} bps 并启动")
        return True
    except Exception as e:
        print(f"设置接口 {interface} 失败: {e}")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="设置SocketCAN接口")
    parser.add_argument("--interface", "-i", default="can0", help="接口名称 (默认: can0)")
    parser.add_argument("--bitrate", "-b", type=int, default=500000, help="波特率 (默认: 500000)")
    
    args = parser.parse_args()
    setup_socketcan(args.interface, args.bitrate)