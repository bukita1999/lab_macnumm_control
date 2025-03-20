import time
from motor_controller import MotorController
from trajectory_planner import TrajectoryPlanner

class MecanumController:
    """麦克纳姆轮控制类"""
    
    # 运动模式映射
    MOTION_MODES = {
        'w': 'forward',    # 前进
        's': 'backward',   # 后退
        'a': 'left',       # 左移
        'd': 'right',      # 右移
        'q': 'turn_ccw',   # 逆时针旋转
        'e': 'turn_cw'     # 顺时针旋转
    }
    
    def __init__(self, can_controller, wheel_ids=(1, 2, 3, 4)):
        """初始化麦克纳姆轮控制器
        wheel_ids: 四个轮子的节点ID(左前, 右前, 左后, 右后)
        """
        self.can = can_controller
        
        # 创建四个电机控制器
        self.motors = {
            'lf': MotorController(can_controller, wheel_ids[0]),  # 左前
            'rf': MotorController(can_controller, wheel_ids[1]),  # 右前
            'lb': MotorController(can_controller, wheel_ids[2]),  # 左后
            'rb': MotorController(can_controller, wheel_ids[3])   # 右后
        }
        
        # 创建轨迹规划器
        self.trajectory_planner = TrajectoryPlanner()
        
        # 初始化标志
        self.initialized = False
    
    def initialize(self):
        """初始化所有电机"""
        print("初始化麦克纳姆轮控制器...")
        success = True
        
        for name, motor in self.motors.items():
            print(f"初始化 {name} 电机...")
            if not motor.initialize():
                print(f"初始化 {name} 电机失败")
                success = False
        
        self.initialized = success
        return success
    
    def get_wheel_speeds(self, motion_type, speed):
        """根据运动类型计算每个轮子的速度"""
        speeds = {}
        
        if motion_type == 'forward':
            # 前进: 所有轮子正转
            speeds = {
                'lf': speed,
                'rf': -speed,
                'lb': speed,
                'rb': -speed
            }
        elif motion_type == 'backward':
            # 后退: 所有轮子反转
            speeds = {
                'lf': -speed,
                'rf': speed,
                'lb': -speed,
                'rb': speed
            }
        elif motion_type == 'left':
            # 左移: 对角轮子方向相反
            speeds = {
                'lf': -speed,
                'rf': -speed,
                'lb': speed,
                'rb': speed
            }
        elif motion_type == 'right':
            # 右移: 对角轮子方向相反
            speeds = {
                'lf': speed,
                'rf': speed,
                'lb': -speed,
                'rb': -speed
            }
        elif motion_type == 'turn_ccw':
            # 逆时针旋转: 左侧轮子反转，右侧轮子正转
            speeds = {
                'lf': -speed,
                'rf': -speed,
                'lb': -speed,
                'rb': -speed
            }
        elif motion_type == 'turn_cw':
            # 顺时针旋转: 左侧轮子正转，右侧轮子反转
            speeds = {
                'lf': speed,
                'rf': speed,
                'lb': speed,
                'rb': speed
            }
        
        return speeds
    
    def execute_motion(self, motion_key, accel_time, cruise_time, decel_time, target_rpm):
        """执行运动命令"""
        if not self.initialized:
            print("控制器未初始化")
            return False
            
        if motion_key not in self.MOTION_MODES:
            print(f"未知运动模式: {motion_key}")
            return False
            
        motion_type = self.MOTION_MODES[motion_key]
        print(f"执行 {motion_type} 运动, 目标RPM: {target_rpm}")
        
        try:
            # 生成梯形速度曲线
            time_points, speed_profile = self.trajectory_planner.generate_trapezoidal_profile(
                target_rpm, accel_time, cruise_time, decel_time
            )
            
            # 获取各轮速度方向
            wheel_directions = self.get_wheel_speeds(motion_type, 1.0)
            
            # 创建适配各轮的速度曲线
            wheel_profiles = {}
            for wheel, direction in wheel_directions.items():
                wheel_profiles[wheel] = [s * direction for s in speed_profile]
            
            # 执行轨迹
            print("开始执行轨迹...")
            start_time = time.time()
            last_index = 0
            
            while last_index < len(time_points):
                current_time = time.time() - start_time
                
                # 找到当前时间对应的速度点
                while last_index < len(time_points) and time_points[last_index] <= current_time:
                    last_index += 1
                
                if last_index > 0:
                    progress = min(100, int(current_time / time_points[-1] * 100))
                    remaining = max(0, time_points[-1] - current_time)
                    print(f"执行中: {progress}%, 剩余 {remaining:.1f} 秒       ", end="\r")
                    
                    # 设置各轮速度
                    for wheel, profile in wheel_profiles.items():
                        current_speed = profile[last_index - 1]
                        self.motors[wheel].set_speed(current_speed)
                
                # 休眠短暂时间，减少CPU占用
                time.sleep(0.05)
                
                # 检查是否完成
                if current_time >= time_points[-1]:
                    break
            
            # 停止所有电机
            for motor in self.motors.values():
                motor.set_speed(0)
            
            print("\n运动执行完成!")
            return True
            
        except KeyboardInterrupt:
            print("\n运动被用户中断")
            # 停止所有电机
            for motor in self.motors.values():
                motor.set_speed(0)
            return False
        except Exception as e:
            print(f"\n运动执行异常: {e}")
            # 停止所有电机
            for motor in self.motors.values():
                motor.set_speed(0)
            return False
    
    # 新增方法
    def set_wheel_speeds_directly(self, wheel_speeds):
        """直接设置各轮速度
        wheel_speeds: 包含各轮速度的字典 {'lf': speed, 'rf': speed, 'lb': speed, 'rb': speed}
        """
        if not self.initialized:
            print("控制器未初始化")
            return False
            
        try:
            for wheel, speed in wheel_speeds.items():
                self.motors[wheel].set_speed(speed)
            return True
        except Exception as e:
            print(f"设置轮速度异常: {e}")
            return False

    def stop_all(self):
        """停止所有电机"""
        for name, motor in self.motors.items():
            print(f"停止 {name} 电机...")
            motor.set_speed(0)
            time.sleep(0.05)
    
    def close(self):
        """关闭控制器"""
        self.stop_all()
        print("麦克纳姆轮控制器已关闭")
