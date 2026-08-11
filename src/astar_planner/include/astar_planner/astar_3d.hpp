#pragma once

#include <uav_mapping/msg/voxel_map.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>


namespace astar_planner
{

class AStar3D
{
public:

    AStar3D() = default;


    // 设置最新VoxelMap
    void setMap(
        const uav_mapping::msg::VoxelMap &map);


    bool hasMap() const
    {
        return map_received_;
    }


    // 是否允许A*进入UNKNOWN区域
    void setAllowUnknown(bool value)
    {
        allow_unknown_ = value;
    }


    // UNKNOWN区域移动代价倍率
    void setUnknownCost(double value)
    {
        unknown_cost_ =
            value < 1.0 ? 1.0 : value;
    }


    // ==============================
    // A*搜索
    // ==============================

    bool search(
        const std::array<double, 3> &start,
        const std::array<double, 3> &goal,
        std::vector<std::array<double, 3>> &path,
        std::string &message);


private:

    // ==============================
    // 地图相关
    // ==============================

    bool worldToGrid(
        double x,
        double y,
        double z,
        int &ix,
        int &iy,
        int &iz) const;


    std::array<double, 3>
    gridToWorld(
        int x,
        int y,
        int z) const;


    bool validGrid(
        int x,
        int y,
        int z) const;


    int toIndex(
        int x,
        int y,
        int z) const;


    void indexToGrid(
        int index,
        int &x,
        int &y,
        int &z) const;


    int8_t getState(
        int x,
        int y,
        int z) const;


    bool isTraversable(
        int x,
        int y,
        int z) const;


    // ==============================
    // 检查一步移动是否安全
    //
    // 防止3D对角运动从两个障碍物
    // 中间“穿角”
    // ==============================

    bool checkMove(
        int x,
        int y,
        int z,
        int dx,
        int dy,
        int dz,
        bool &passes_unknown) const;


    // 启发函数
    double heuristic(
        int x,
        int y,
        int z,
        int gx,
        int gy,
        int gz) const;


private:

    bool map_received_{false};

    double resolution_{0.2};

    double origin_x_{0.0};
    double origin_y_{0.0};
    double origin_z_{0.0};

    int size_x_{0};
    int size_y_{0};
    int size_z_{0};

    std::vector<int8_t> map_data_;

    bool allow_unknown_{true};

    double unknown_cost_{2.5};
};

}  // namespace astar_planner