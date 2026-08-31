#!/bin/bash

# ============================================================
# PX4 UAV Navigation 一键启动脚本
#
# 该脚本启动：
# 1. MicroXRCEAgent
# 2. Gazebo clock + 3D LiDAR bridge
# 3. PX4 + Gazebo
#
# ROS2 导航节点由 hint.md 中的第二条命令单独启动：
# ros2 launch bringup navigation.launch.py
# ============================================================


# 出错变量直接报错
set -u


# ============================================================
# 路径配置
# ============================================================

PX4_DIR="$HOME/PX4-Autopilot"

WORLD_NAME="uav_mapping"

PX4_MODEL="gz_x500_lidar_3d"

CLEANUP_DONE=0


echo "========================================"
echo " PX4 UAV Navigation"
echo "========================================"

# Make the script usable from a fresh terminal. The navigation launch is
# started separately, so source both ROS and this workspace when available.
ROS_SETUP=""
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
fi

if [ -n "$ROS_SETUP" ]; then
    # ROS setup scripts may reference optional variables while set -u is on.
    set +u
    source "$ROS_SETUP"
    set -u
fi

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    set +u
    source "$WORKSPACE_DIR/install/setup.bash"
    set -u
fi


# ============================================================
# 清理函数
#
# Ctrl+C退出脚本时，把我们启动的后台进程一起关闭
# ============================================================

cleanup()
{
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return
    fi
    CLEANUP_DONE=1

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

echo "[1/3] 启动 MicroXRCEAgent..."

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

echo "[2/3] 启动 Gazebo ROS2 Bridge..."

ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
    '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
    &

BRIDGE_PID=$!


# ============================================================
# 4. 启动 PX4 + Gazebo
# ============================================================

echo "[3/3] 启动 PX4 + Gazebo..."

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
