#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace uav_mapping
{

class VoxelMap
{
public:

    static constexpr int8_t UNKNOWN  = -1;
    static constexpr int8_t FREE     = 0;
    static constexpr int8_t OCCUPIED = 100;

    VoxelMap(
        double resolution,
        const Eigen::Vector3d &map_min,
        const Eigen::Vector3d &map_max,
        double inflation_radius);


    // ==========================
    // 基本地图信息
    // ==========================

    double resolution() const
    {
        return resolution_;
    }

    int sizeX() const
    {
        return size_x_;
    }

    int sizeY() const
    {
        return size_y_;
    }

    int sizeZ() const
    {
        return size_z_;
    }

    const Eigen::Vector3d &mapMin() const
    {
        return map_min_;
    }


    // ==========================
    // 坐标转换
    // ==========================

    bool insideMap(
        const Eigen::Vector3d &p) const;

    bool validGrid(
        int x,
        int y,
        int z) const;

    bool worldToGrid(
        const Eigen::Vector3d &p,
        int &x,
        int &y,
        int &z) const;

    Eigen::Vector3d gridToWorld(
        int x,
        int y,
        int z) const;


    // ==========================
    // 地图访问
    // ==========================

    int toIndex(
        int x,
        int y,
        int z) const;

    int8_t get(
        int x,
        int y,
        int z) const;

    void setFree(
        int x,
        int y,
        int z);

    void setOccupied(
        int x,
        int y,
        int z);


    // ==========================
    // 激光射线更新
    // ==========================

    void integrateRay(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &end);


    // ==========================
    // 发布数据
    // ==========================

    void getPlanningData(
        std::vector<int8_t> &data) const;

    std::vector<Eigen::Vector3d>
    getOccupiedPoints(
        bool include_inflation) const;


private:

    // 障碍物膨胀
    void inflateObstacle(
        int x,
        int y,
        int z);


private:

    double resolution_;
    double inflation_radius_;

    Eigen::Vector3d map_min_;
    Eigen::Vector3d map_max_;

    int size_x_;
    int size_y_;
    int size_z_;

    // 原始占据地图
    std::vector<int8_t> state_;

    // 膨胀区域
    std::vector<uint8_t> inflated_;
};

}  // namespace uav_mapping