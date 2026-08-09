#include "uav_mapping/voxel_map.hpp"

#include <algorithm>
#include <cmath>

namespace uav_mapping
{

VoxelMap::VoxelMap(
    double resolution,
    const Eigen::Vector3d &map_min,
    const Eigen::Vector3d &map_max,
    double inflation_radius)
    : resolution_(resolution),
      inflation_radius_(inflation_radius),
      map_min_(map_min),
      map_max_(map_max)
{
    size_x_ = static_cast<int>(
        std::ceil(
            (map_max_.x() - map_min_.x()) /
            resolution_));

    size_y_ = static_cast<int>(
        std::ceil(
            (map_max_.y() - map_min_.y()) /
            resolution_));

    size_z_ = static_cast<int>(
        std::ceil(
            (map_max_.z() - map_min_.z()) /
            resolution_));


    const std::size_t total_size =
        static_cast<std::size_t>(size_x_) *
        static_cast<std::size_t>(size_y_) *
        static_cast<std::size_t>(size_z_);


    // 初始全部未知
    state_.assign(
        total_size,
        UNKNOWN);

    inflated_.assign(
        total_size,
        0);
}


// ============================================================
// 判断世界坐标是否在地图范围内
// ============================================================

bool VoxelMap::insideMap(
    const Eigen::Vector3d &p) const
{
    return
        p.x() >= map_min_.x() &&
        p.y() >= map_min_.y() &&
        p.z() >= map_min_.z() &&

        p.x() < map_max_.x() &&
        p.y() < map_max_.y() &&
        p.z() < map_max_.z();
}


// ============================================================
// 判断体素索引是否合法
// ============================================================

bool VoxelMap::validGrid(
    int x,
    int y,
    int z) const
{
    return
        x >= 0 && x < size_x_ &&
        y >= 0 && y < size_y_ &&
        z >= 0 && z < size_z_;
}


// ============================================================
// 世界坐标 -> 体素索引
// ============================================================

bool VoxelMap::worldToGrid(
    const Eigen::Vector3d &p,
    int &x,
    int &y,
    int &z) const
{
    if (!insideMap(p))
        return false;


    x = static_cast<int>(
        std::floor(
            (p.x() - map_min_.x()) /
            resolution_));

    y = static_cast<int>(
        std::floor(
            (p.y() - map_min_.y()) /
            resolution_));

    z = static_cast<int>(
        std::floor(
            (p.z() - map_min_.z()) /
            resolution_));


    return validGrid(x, y, z);
}


// ============================================================
// 体素索引 -> 世界坐标
//
// 返回体素中心，而不是体素角点
// ============================================================

Eigen::Vector3d VoxelMap::gridToWorld(
    int x,
    int y,
    int z) const
{
    return map_min_ +
           resolution_ *
           Eigen::Vector3d(
               x + 0.5,
               y + 0.5,
               z + 0.5);
}


// ============================================================
// 三维索引 -> 一维数组索引
//
// index = x + Nx * (y + Ny * z)
// ============================================================

int VoxelMap::toIndex(
    int x,
    int y,
    int z) const
{
    return
        x +
        size_x_ *
        (y + size_y_ * z);
}


// ============================================================
// 获取体素状态
// ============================================================

int8_t VoxelMap::get(
    int x,
    int y,
    int z) const
{
    if (!validGrid(x, y, z))
        return OCCUPIED;

    return state_[
        toIndex(x, y, z)];
}


// ============================================================
// 设置FREE
//
// 当前任务是静态环境。
// 如果该位置已经确认是障碍物，不再被FREE覆盖。
// ============================================================

void VoxelMap::setFree(
    int x,
    int y,
    int z)
{
    if (!validGrid(x, y, z))
        return;

    const int index =
        toIndex(x, y, z);

    if (state_[index] == UNKNOWN)
    {
        state_[index] = FREE;
    }
}


// ============================================================
// 设置OCCUPIED
// ============================================================

void VoxelMap::setOccupied(
    int x,
    int y,
    int z)
{
    if (!validGrid(x, y, z))
        return;

    const int index =
        toIndex(x, y, z);


    // 已经是障碍物，不需要重复膨胀
    if (state_[index] == OCCUPIED)
        return;


    state_[index] =
        OCCUPIED;


    inflateObstacle(
        x,
        y,
        z);
}


// ============================================================
// 障碍物膨胀
//
// 将无人机近似成一个点，
// 反过来扩大障碍物。
// ============================================================

void VoxelMap::inflateObstacle(
    int cx,
    int cy,
    int cz)
{
    const int radius_grid =
        static_cast<int>(
            std::ceil(
                inflation_radius_ /
                resolution_));


    const double radius2 =
        inflation_radius_ *
        inflation_radius_;


    for (int dx = -radius_grid;
         dx <= radius_grid;
         ++dx)
    {
        for (int dy = -radius_grid;
             dy <= radius_grid;
             ++dy)
        {
            for (int dz = -radius_grid;
                 dz <= radius_grid;
                 ++dz)
            {
                // 真实欧氏距离
                const double rx =
                    dx * resolution_;

                const double ry =
                    dy * resolution_;

                const double rz =
                    dz * resolution_;


                if (rx * rx +
                    ry * ry +
                    rz * rz >
                    radius2)
                {
                    continue;
                }


                const int x =
                    cx + dx;

                const int y =
                    cy + dy;

                const int z =
                    cz + dz;


                if (!validGrid(x, y, z))
                    continue;


                inflated_[
                    toIndex(
                        x,
                        y,
                        z)] = 1;
            }
        }
    }
}


// ============================================================
// 激光Raycasting
//
// start：LiDAR位置
// end：激光击中的障碍物位置
// ============================================================

void VoxelMap::integrateRay(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &end)
{
    if (!insideMap(start) ||
        !insideMap(end))
    {
        return;
    }


    const Eigen::Vector3d direction =
        end - start;

    const double length =
        direction.norm();


    if (length < 1e-6)
        return;


    // 每半个voxel采样一次，
    // 简单且对于当前仿真任务足够。
    const double step =
        resolution_ * 0.5;


    const int step_num =
        std::max(
            1,
            static_cast<int>(
                std::ceil(
                    length / step)));


    int last_index = -1;


    // 注意 i < step_num：
    // 不把真正终点当FREE处理
    for (int i = 0;
         i < step_num;
         ++i)
    {
        const double ratio =
            static_cast<double>(i) /
            static_cast<double>(step_num);


        const Eigen::Vector3d p =
            start +
            ratio * direction;


        int x, y, z;

        if (!worldToGrid(
                p,
                x,
                y,
                z))
        {
            continue;
        }


        const int index =
            toIndex(x, y, z);


        // 防止一条射线在同一个voxel
        // 重复操作很多次
        if (index == last_index)
            continue;


        last_index = index;


        setFree(
            x,
            y,
            z);
    }


    // 最后的激光击中位置是障碍物
    int end_x;
    int end_y;
    int end_z;

    if (worldToGrid(
            end,
            end_x,
            end_y,
            end_z))
    {
        setOccupied(
            end_x,
            end_y,
            end_z);
    }
}


// ============================================================
// 生成给A*使用的地图
//
// 原始障碍物：100
// 膨胀区域：100
// FREE：0
// UNKNOWN：-1
// ============================================================

void VoxelMap::getPlanningData(
    std::vector<int8_t> &data) const
{
    data.resize(
        state_.size());


    for (std::size_t i = 0;
         i < state_.size();
         ++i)
    {
        if (state_[i] == OCCUPIED ||
            inflated_[i])
        {
            data[i] =
                OCCUPIED;
        }
        else
        {
            data[i] =
                state_[i];
        }
    }
}


// ============================================================
// 生成RViz显示点云
// ============================================================

std::vector<Eigen::Vector3d>
VoxelMap::getOccupiedPoints(
    bool include_inflation) const
{
    std::vector<Eigen::Vector3d>
        points;


    for (int z = 0;
         z < size_z_;
         ++z)
    {
        for (int y = 0;
             y < size_y_;
             ++y)
        {
            for (int x = 0;
                 x < size_x_;
                 ++x)
            {
                const int index =
                    toIndex(x, y, z);


                bool occupied =
                    state_[index] ==
                    OCCUPIED;


                if (include_inflation)
                {
                    occupied =
                        occupied ||
                        inflated_[index];
                }


                if (!occupied)
                    continue;


                points.push_back(
                    gridToWorld(
                        x,
                        y,
                        z));
            }
        }
    }


    return points;
}

}  // namespace uav_mapping