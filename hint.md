# 启动3d激光雷达无人机
PX4_GZ_WORLD=uav_mapping make px4_sitl gz_x500_lidar_3d
# px4话题
MicroXRCEAgent udp4 -p 8888
# 启动桥接
ros2 run ros_gz_bridge parameter_bridge '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
# 雷达话题
/lidar_3d/points
# 点云坐标系
/link
# 雷达相对位姿
$$t^{body}_{lidar}=\begin{bmatrix}0.12\\0\\0.26\end{bmatrix}$$

# 使用别人仓库
git submodule add https://github.com/xxxx/别人的仓库.git 本地文件夹名
# 如何clone
git clone --recursive 你的仓库地址