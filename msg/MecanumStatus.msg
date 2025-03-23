# 麦克纳姆轮控制器状态
Header header          # 标准头（时间戳）
string[] wheel_names   # 轮子名称 ["lf", "rf", "lb", "rb"]
float32[] wheel_speeds # 各轮当前速度(RPM)
string motion_state    # 当前运动状态
float32 progress       # 执行进度 (0-1)
string current_command # 当前命令
bool is_error          # 是否有错误
string error_message   # 错误信息
