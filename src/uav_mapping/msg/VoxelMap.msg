# 三维体素占据地图

std_msgs/Header header


# ==============================
# 体素状态
# ==============================

int8 UNKNOWN=-1
int8 FREE=0
int8 OCCUPIED=100


# ==============================
# 地图参数
# ==============================

# 一个体素的边长，单位 m
float32 resolution


# voxel(0,0,0) 对应的地图最小角点
geometry_msgs/Point origin


# 三个方向体素数量
uint32 size_x
uint32 size_y
uint32 size_z


# ==============================
# 地图数据
# ==============================

# 一维数组保存三维地图
#
# index =
# x + size_x * (y + size_y * z)
#
# data[index]:
#
# -1   UNKNOWN
#  0   FREE
# 100  OCCUPIED

int8[] data