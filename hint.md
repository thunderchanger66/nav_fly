# 启动3d激光雷达无人机
PX4_GZ_WORLD=uav_mapping make px4_sitl gz_x500_lidar_3d
# px4话题
MicroXRCEAgent udp4 -p 8888
# 启动桥接
ros2 run ros_gz_bridge parameter_bridge '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
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

# PX4与uXRCE-DDS时间同步关闭
param show UXRCE_DDS_SYNCT
param set UXRCE_DDS_SYNCT 0

# 安装第三方库
git submodule add https://github.com/osqp/osqp.git third_party/osqp
git submodule add https://github.com/gbionics/osqp-eigen.git third_party/osqp-eigen
cd third_party
## osqp
cd osqp
mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/uav_navigation_ws/third_party/install
cmake --build . -j$(nproc)
cmake --install .
## osqp-eigen
cd osqp-eigen
mkdir build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/study/nav_fly/third_party/install \
  -DCMAKE_PREFIX_PATH=$HOME/study/nav_fly/third_party/install
cmake --build . -j$(nproc)
cmake --install .

# 编译时需加上这个
export CMAKE_PREFIX_PATH=$HOME/study/nav_fly/third_party/install:$CMAKE_PREFIX_PATH
colcon build --packages-up-to bringup --symlink-install

# 总启动
./scripts/start_sim.sh
ros2 launch bringup navigation.launch.py

# 单独的启动
# 建图启动
ros2 launch uav_mapping mapping.launch.py
# 轨迹优化启动
ros2 launch trajectory_optimizer trajectory_optimizer.launch.py
# Astar规划服务
ros2 service call /astar/plan astar_planner/srv/PlanPath "{start: {x: 0.0, y: 0.0, z: 2.5}, goal: {x: 12.0, y: 0.0, z: 2.5}}"