import keyboard
import threading
import time
import os
from command_interface import CommandInterface

class KeyboardControlInterface(CommandInterface):
    """基于键盘的麦克纳姆轮实时控制接口"""

    def __init__(self, mecanum_controller):
        """初始化键盘控制接口"""
        super().__init__(mecanum_controller)
        self.keys_pressed = set()  # 当前按下的键
        self.key_mapping = {
            'w': 'forward',    # 前进
            's': 'backward',   # 后退
            'a': 'left',       # 左移
            'd': 'right',      # 右移
            'q': 'turn_ccw',   # 逆时针旋转
            'e': 'turn_cw'     # 顺时针旋转
        }
        
        # 速度控制参数
        self.target_speeds = {key: 0 for key in self.key_mapping}
        self.current_speeds = {key: 0 for key in self.key_mapping}
        self.max_speed = 100.0  # 最大RPM
        self.acceleration = 20.0  # 每秒增加的RPM
        self.deceleration = 50.0  # 每秒减少的RPM
        
        # 显示控制
        self.display_speeds = True
        self.last_display_time = 0
        self.display_interval = 0.2  # 显示更新间隔(秒)
        
        # 控制线程
        self.control_thread = None
        self.update_interval = 0.05  # 控制更新间隔(秒)
        self.running = True  # 运行状态
        
    def display_help(self):
        """显示帮助信息"""
        print("\n麦克纳姆轮键盘控制")
        print("=======================")
        print("控制键:")
        print("  W - 前进")
        print("  S - 后退")
        print("  A - 左移")
        print("  D - 右移")
        print("  Q - 逆时针旋转")
        print("  E - 顺时针旋转")
        print("  X - 停止并退出")
        print("  H - 显示帮助")
        print("\n参数调节:")
        print("  + / = - 增加最大速度")
        print("  - / _ - 减少最大速度")
        print("  [ - 减小加速度")
        print("  ] - 增加加速度")
        print("  ; - 减小减速度")
        print("  ' - 增加减速度")
        print("  V - 切换速度显示")
        print("=======================")
        print("同时按下多个键可以实现组合运动")
        print("按键越久，速度越快(渐进加速)")
        print("松开按键自动减速停止")
        print("=======================")
        print(f"当前参数: 最大速度={self.max_speed:.1f} RPM, 加速度={self.acceleration:.1f} RPM/s, 减速度={self.deceleration:.1f} RPM/s")
        print("=======================")
        
    def on_key_event(self, e):
        """处理键盘事件"""
        key = e.name.lower()
        
        if e.event_type == keyboard.KEY_DOWN:
            if key in self.key_mapping:
                self.keys_pressed.add(key)
                self.target_speeds[key] = self.max_speed
            elif key in ['=', '+']:
                self.max_speed = min(self.max_speed + 1.0, 50.0)
                print(f"\n最大速度增加至 {self.max_speed:.1f} RPM")
            elif key in ['-', '_']:
                self.max_speed = max(self.max_speed - 1.0, 1.0)
                print(f"\n最大速度减小至 {self.max_speed:.1f} RPM")
            elif key == '[':
                self.acceleration = max(self.acceleration - 0.5, 0.5)
                print(f"\n加速度减小至 {self.acceleration:.1f} RPM/s")
            elif key == ']':
                self.acceleration = min(self.acceleration + 0.5, 20.0)
                print(f"\n加速度增加至 {self.acceleration:.1f} RPM/s")
            elif key == ';':
                self.deceleration = max(self.deceleration - 0.5, 1.0)
                print(f"\n减速度减小至 {self.deceleration:.1f} RPM/s")
            elif key == "'":
                self.deceleration = min(self.deceleration + 0.5, 30.0)
                print(f"\n减速度增加至 {self.deceleration:.1f} RPM/s")
            elif key == 'v':
                self.display_speeds = not self.display_speeds
                if self.display_speeds:
                    print("\n速度显示已开启")
                else:
                    print("\n速度显示已关闭")
                    # 清除当前行
                    print(" " * 80, end="\r")
            elif key == 'x':
                self.running = False
            elif key == 'h':
                self.display_help()
        elif e.event_type == keyboard.KEY_UP:
            if key in self.key_mapping:
                self.keys_pressed.discard(key)
                self.target_speeds[key] = 0
                
    def update_speeds(self):
        """根据按键状态更新速度"""
        while self.running:
            for key, motion in self.key_mapping.items():
                target = self.target_speeds[key]
                current = self.current_speeds[key]
                
                # 应用加速或减速
                if current < target:
                    self.current_speeds[key] = min(current + self.acceleration * self.update_interval, target)
                elif current > target:
                    self.current_speeds[key] = max(current - self.deceleration * self.update_interval, 0)
            
            # 计算最终的运动速度向量
            forward_speed = self.current_speeds['w'] - self.current_speeds['s']
            right_speed = self.current_speeds['d'] - self.current_speeds['a']
            rotation_speed = self.current_speeds['e'] - self.current_speeds['q']
            
            # 应用到电机
            wheel_speeds = self.apply_motion(forward_speed, right_speed, rotation_speed)
            
            # 更新速度显示
            current_time = time.time()
            if self.display_speeds and current_time - self.last_display_time > self.display_interval:
                self.display_current_speeds(wheel_speeds, forward_speed, right_speed, rotation_speed)
                self.last_display_time = current_time
            
            time.sleep(self.update_interval)
    
    def display_current_speeds(self, wheel_speeds, forward, right, rotation):
        """显示当前速度信息"""
        # 获取终端宽度
        try:
            terminal_width = os.get_terminal_size().columns
        except:
            terminal_width = 80
            
        if wheel_speeds:
            # 格式化各轮速度
            speed_info = f"LF:{wheel_speeds.get('lf', 0):6.1f} RF:{wheel_speeds.get('rf', 0):6.1f} "
            speed_info += f"LB:{wheel_speeds.get('lb', 0):6.1f} RB:{wheel_speeds.get('rb', 0):6.1f} | "
            speed_info += f"前进:{forward:5.1f} 右移:{right:5.1f} 旋转:{rotation:5.1f} | "
            speed_info += f"最大速度:{self.max_speed:4.1f}"
            
            # 截断到终端宽度
            if len(speed_info) > terminal_width - 5:
                speed_info = speed_info[:terminal_width - 5] + "..."
                
            print(speed_info, end="\r")
            
    def apply_motion(self, forward, right, rotation):
        """应用运动到电机控制器，返回计算后的各轮速度"""
        # 计算各轮速度
        wheel_speeds = {}
        
        # 前进/后退分量
        if forward != 0:
            move_type = 'forward' if forward > 0 else 'backward'
            speeds = self.controller.get_wheel_speeds(move_type, abs(forward))
            for wheel, speed in speeds.items():
                wheel_speeds[wheel] = wheel_speeds.get(wheel, 0) + speed
                
        # 左/右分量
        if right != 0:
            move_type = 'right' if right > 0 else 'left'
            speeds = self.controller.get_wheel_speeds(move_type, abs(right))
            for wheel, speed in speeds.items():
                wheel_speeds[wheel] = wheel_speeds.get(wheel, 0) + speed
                
        # 旋转分量
        if rotation != 0:
            move_type = 'turn_cw' if rotation > 0 else 'turn_ccw'
            speeds = self.controller.get_wheel_speeds(move_type, abs(rotation))
            for wheel, speed in speeds.items():
                wheel_speeds[wheel] = wheel_speeds.get(wheel, 0) + speed
        
        # 如果没有任何输入，停止所有电机
        if forward == 0 and right == 0 and rotation == 0:
            for wheel in self.controller.motors:
                self.controller.motors[wheel].set_speed(0)
            return {}
            
        # 设置各轮速度
        for wheel, speed in wheel_speeds.items():
            self.controller.motors[wheel].set_speed(speed)
            
        return wheel_speeds
            
    def run(self):
        """运行键盘控制接口"""
        print("\n初始化麦克纳姆轮控制器...")
        if not self.controller.initialize():
            print("初始化失败，程序退出")
            return
            
        self.display_help()
            
        # 注册键盘钩子
        keyboard.hook(self.on_key_event)
        
        # 启动速度控制线程
        self.control_thread = threading.Thread(target=self.update_speeds)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        try:
            # 保持主线程运行
            while self.running:
                time.sleep(0.1)  # 减少CPU使用率
                
        except KeyboardInterrupt:
            print("\n收到中断信号，退出程序")
            self.running = False
        finally:
            # 清除速度显示行
            if self.display_speeds:
                print(" " * 80, end="\r")
                print()  # 换行
                
            # 取消键盘钩子
            keyboard.unhook_all()
            # 关闭控制器
            self.controller.close()
            print("麦克纳姆轮控制系统已关闭")
