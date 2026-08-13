#pragma once

#include <uav_mapping/msg/voxel_map.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>


namespace trajectory_optimizer
{

// ============================================================
// 一个轴对齐安全走廊
// ============================================================

struct CorridorBox
{
    Eigen::Vector3d min;
    Eigen::Vector3d max;
};


// ============================================================
// 优化参数
// ============================================================

struct OptimizerOptions
{
    // UNKNOWN是否允许轨迹进入
    bool unknown_is_free{true};

    // 路径简化时是否允许直接穿UNKNOWN
    // 建议false，防止把A*特意选择的已知自由路径直接剪掉
    bool simplify_allow_unknown{false};


    // B样条参考点间距
    double control_point_spacing{0.8};


    // 单个安全走廊最大扩张半径
    double corridor_max_radius{0.8};

    // 与voxel边界留一点距离
    double corridor_margin{0.02};


    // QP权重
    double weight_smooth{20.0};
    double weight_reference{1.0};


    // B样条节点时间间隔
    double initial_dt{0.4};


    // XYZ各轴约束
    double max_vel_axis{2.0};
    double max_acc_axis{2.5};


    // QP因为动力学约束不可行时，
    // 自动增加轨迹时间
    double time_scale_factor{1.25};
    int max_time_scaling_attempts{5};


    // 最终轨迹安全检查
    double validation_dt{0.05};
};


// ============================================================
// 优化结果
// ============================================================

struct OptimizationResult
{
    std::vector<Eigen::Vector3d> simplified_path;

    std::vector<Eigen::Vector3d> reference_points;

    std::vector<CorridorBox> corridors;

    std::vector<Eigen::Vector3d> control_points;

    double dt{0.0};

    double duration{0.0};
};


// ============================================================
// 轨迹优化器
// ============================================================

class TrajectoryOptimizer
{
public:

    TrajectoryOptimizer() = default;


    void setOptions(
        const OptimizerOptions &options);


    void setMap(
        const uav_mapping::msg::VoxelMap &map);


    bool hasMap() const
    {
        return map_received_;
    }


    // 主入口
    bool optimize(
        const std::vector<Eigen::Vector3d> &astar_path,
        OptimizationResult &result,
        std::string &message);


    // 三次均匀B样条求值
    static Eigen::Vector3d evaluateBSpline(
        const std::vector<Eigen::Vector3d> &control_points,
        double dt,
        double t);


    static std::vector<Eigen::Vector3d>
    sampleBSpline(
        const std::vector<Eigen::Vector3d> &control_points,
        double dt,
        double sample_dt);


private:

    // ============================
    // 地图
    // ============================

    bool validGrid(
        int x,
        int y,
        int z) const;


    int toIndex(
        int x,
        int y,
        int z) const;


    bool worldToGrid(
        const Eigen::Vector3d &p,
        int &x,
        int &y,
        int &z) const;


    bool cellSafe(
        int x,
        int y,
        int z,
        bool allow_unknown) const;


    bool pointSafe(
        const Eigen::Vector3d &p,
        bool allow_unknown) const;


    bool lineSafe(
        const Eigen::Vector3d &a,
        const Eigen::Vector3d &b,
        bool allow_unknown) const;


    // ============================
    // A*路径处理
    // ============================

    std::vector<Eigen::Vector3d>
    simplifyPath(
        const std::vector<Eigen::Vector3d> &path) const;


    std::vector<Eigen::Vector3d>
    resamplePath(
        const std::vector<Eigen::Vector3d> &path,
        double spacing) const;


    // ============================
    // 安全走廊
    // ============================

    bool boxSafe(
        int xmin,
        int xmax,
        int ymin,
        int ymax,
        int zmin,
        int zmax) const;


    bool buildCorridor(
        const Eigen::Vector3d &point,
        CorridorBox &corridor) const;


    // ============================
    // QP
    // ============================

    bool solveQP(
        const std::vector<Eigen::Vector3d> &reference,
        const std::vector<CorridorBox> &corridors,
        double dt,
        std::vector<Eigen::Vector3d> &control_points);


    bool validateTrajectory(
        const std::vector<Eigen::Vector3d> &control_points,
        double dt) const;


private:

    OptimizerOptions options_;


    bool map_received_{false};

    double resolution_{0.2};

    Eigen::Vector3d origin_{
        Eigen::Vector3d::Zero()};

    int size_x_{0};
    int size_y_{0};
    int size_z_{0};

    std::vector<int8_t> map_data_;
};

}  // namespace trajectory_optimizer