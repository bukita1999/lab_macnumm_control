Header header         # 时间戳和帧ID
string current_state  # 当前状态
float32[] wheel_speeds # 各轮当前速度
bool[] wheel_enabled  # 各轮使能状态
float32 battery_voltage # 电池电压（可选）
int32 error_code      # 错误代码（如果有）
