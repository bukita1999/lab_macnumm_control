import numpy as np
import time

class TrajectoryPlanner:
    """轨迹规划类，用于生成梯形速度曲线"""
    
    @staticmethod
    def generate_trapezoidal_profile(target_speed, accel_time, cruise_time, decel_time, time_step=0.1):
        """生成梯形速度曲线，返回时间点和对应的速度值"""
        # 计算总时间和采样点数
        total_time = accel_time + cruise_time + decel_time
        num_samples = int(total_time / time_step) + 1
        
        # 生成时间序列
        time_points = np.linspace(0, total_time, num_samples)
        speed_profile = np.zeros_like(time_points)
        
        # 计算加速度
        acceleration = target_speed / accel_time if accel_time > 0 else 0
        deceleration = target_speed / decel_time if decel_time > 0 else 0
        
        # 生成速度曲线
        for i, t in enumerate(time_points):
            if t <= accel_time:
                # 加速阶段
                speed_profile[i] = t * acceleration
            elif t <= accel_time + cruise_time:
                # 匀速阶段
                speed_profile[i] = target_speed
            else:
                # 减速阶段
                decel_t = t - (accel_time + cruise_time)
                speed_profile[i] = max(0, target_speed - decel_t * deceleration)
        
        return time_points.tolist(), speed_profile.tolist()
    
    def execute_speed_profile(self, motor, time_points, speed_profile, speed_callback=None):
        """执行速度曲线控制"""
        if len(time_points) != len(speed_profile):
            print("错误: 时间点和速度值数量不匹配")
            return False
        
        start_time = time.time()
        last_index = 0
        
        try:
            while last_index < len(time_points):
                current_time = time.time() - start_time
                
                # 找到当前时间对应的速度点
                while last_index < len(time_points) and time_points[last_index] <= current_time:
                    last_index += 1
                
                if last_index > 0:
                    current_speed = speed_profile[last_index - 1]
                    
                    # 执行速度命令
                    if not motor.set_speed(current_speed):
                        print("设置速度失败")
                        return False
                    
                    # 回调通知
                    if speed_callback:
                        speed_callback(current_time, current_speed)
                
                # 休眠短暂时间，减少CPU占用
                time.sleep(0.05)
                
                # 检查是否完成
                if current_time >= time_points[-1]:
                    break
            
            return True
            
        except KeyboardInterrupt:
            print("执行被用户中断")
            return False
        except Exception as e:
            print(f"执行速度曲线异常: {e}")
            return False