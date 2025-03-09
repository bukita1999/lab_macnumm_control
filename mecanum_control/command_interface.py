class CommandInterface:
    """用户命令接口类"""
    
    def __init__(self, mecanum_controller):
        """初始化命令接口"""
        self.controller = mecanum_controller
        self.running = True
    
    def display_help(self):
        """显示帮助信息"""
        print("\n麦克纳姆轮控制程序")
        print("=======================")
        print("命令格式: <运动模式> <加速时间> <匀速时间> <减速时间> <目标RPM>")
        print("示例: w 2 5 2 10")
        print("\n运动模式:")
        print("  w - 前进")
        print("  s - 后退")
        print("  a - 左移")
        print("  d - 右移")
        print("  q - 逆时针旋转")
        print("  e - 顺时针旋转")
        print("  h - 显示帮助")
        print("  x - 退出程序")
        print("=======================")
    
    def parse_command(self, command):
        """解析命令"""
        parts = command.strip().split()
        
        if not parts:
            print("请输入命令")
            return
        
        motion_key = parts[0].lower()
        
        if motion_key == 'h':
            self.display_help()
            return
        
        if motion_key == 'x':
            self.running = False
            print("退出程序")
            return
        
        if motion_key not in self.controller.MOTION_MODES:
            print(f"未知运动模式: {motion_key}")
            print("输入 'h' 查看帮助")
            return
        
        if len(parts) != 5:
            print("参数不足，格式：<运动模式> <加速时间> <匀速时间> <减速时间> <目标RPM>")
            return
        
        try:
            accel_time = float(parts[1])
            cruise_time = float(parts[2])
            decel_time = float(parts[3])
            target_rpm = float(parts[4])
            
            self.controller.execute_motion(motion_key, accel_time, cruise_time, decel_time, target_rpm)
            
        except ValueError:
            print("参数格式错误，需要数字")
    
    def run(self):
        """运行命令接口"""
        print("\n初始化麦克纳姆轮控制器...")
        if not self.controller.initialize():
            print("初始化失败，程序退出")
            return
        
        self.display_help()
        
        while self.running:
            try:
                command = input("\n请输入命令 > ")
                self.parse_command(command)
            except KeyboardInterrupt:
                print("\n收到中断信号，退出程序")
                self.running = False
            except Exception as e:
                print(f"发生错误: {e}")
        
        # 关闭控制器
        self.controller.close()