# 底盘运动指令 (twist 风格)
# 由 chassis_arm_bridge 或遥控节点发布到 /chassis_cmd

# 线速度 (m/s), 正=前进, 负=后退
float64 linear_x

# 角速度 (rad/s), 正=逆时针(左转), 负=顺时针(右转)
float64 angular_z