#!/bin/bash

# ============================================================
# PX4 UAV Navigation 一键启动脚本
#
# 启动：
# 1. PX4 + Gazebo
# 2. MicroXRCEAgent
# 3. Gazebo clock bridge
# 4. 3D LiDAR bridge
# 5. ROS2 navigation launch
# ============================================================


# 出错变量直接报错
set -u


# ============================================================
# 路径配置
# ============================================================

PX4_DIR="$HOME/PX4-Autopilot"

WORLD_NAME="uav_mapping"

PX4_MODEL="gz_x500_lidar_3d"


echo "========================================"
echo " PX4 UAV Navigation"
echo "========================================"


# ============================================================
# 清理函数
#
# Ctrl+C退出脚本时，把我们启动的后台进程一起关闭
# ============================================================

cleanup()
{
    echo
    echo "[INFO] 正在关闭仿真系统..."

    jobs -pr | xargs -r kill 2>/dev/null

    sleep 1

    echo "[INFO] 已退出"
}


trap cleanup EXIT INT TERM


# ============================================================
# 1. Source ROS
# ============================================================

# ============================================================
# 2. 启动 MicroXRCEAgent
# ============================================================

echo "[2/5] 启动 MicroXRCEAgent..."

MicroXRCEAgent udp4 -p 8888 &

AGENT_PID=$!

sleep 1


# ============================================================
# 3. 启动 Gazebo -> ROS2 Bridge
#
# 同一个bridge处理：
# /clock
# /lidar_3d/points
# ============================================================

echo "[3/5] 启动 Gazebo ROS2 Bridge..."

ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
    '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
    &

BRIDGE_PID=$!


# ============================================================
# 4. 启动 PX4 + Gazebo
# ============================================================

echo "[4/5] 启动 PX4 + Gazebo..."

cd "$PX4_DIR" || exit 1


PX4_GZ_WORLD="$WORLD_NAME" \
make px4_sitl "$PX4_MODEL" &

PX4_PID=$!


# Gazebo/PX4需要一点启动时间
sleep 5


echo
echo "========================================"
echo " 系统已启动"
echo
echo " PX4 model : $PX4_MODEL"
echo " World     : $WORLD_NAME"
echo
echo " Ctrl+C 退出全部进程"
echo "========================================"


# 等待所有后台任务
wait