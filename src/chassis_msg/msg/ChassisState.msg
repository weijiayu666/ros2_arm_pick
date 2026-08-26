# 底盘实时状态消息
# 由 chassis_driver_node 以 10Hz 发布

# 电机速度反馈 (来自 BDC STREAM 实时数据, 单位: 转/秒)
float64 motor1_speed
float64 motor2_speed

# 电机电流 (A)
float64 motor1_current
float64 motor2_current

# 输入电压 (V)
float64 bus_voltage

# 芯片温度 (°C)
float64 temperature

# 电机是否运动中 (任一电机非零占空比)
bool is_moving

# 保护标志 (bit0=电压, bit1=温度, bit2=功率)
int32 protection_flags