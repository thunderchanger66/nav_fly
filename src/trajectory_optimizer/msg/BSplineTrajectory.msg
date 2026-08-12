std_msgs/Header header

# 当前固定为三次B样条
uint8 degree

# 相邻节点时间间隔
float64 dt

# 整条轨迹持续时间
float64 duration

# 优化后的B样条控制点
geometry_msgs/Point[] control_points