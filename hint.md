# 启动3d激光雷达无人机
make px4_sitl gz_x500_lidar_3d
# 启动桥接
ros2 run ros_gz_bridge parameter_bridge '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
# 雷达话题
/lidar_3d/points