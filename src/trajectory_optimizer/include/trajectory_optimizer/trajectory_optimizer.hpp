#pragma once

#include <uav_mapping/msg/voxel_map.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>


namespace trajectory_optimizer
{

// ============================================================
// 轴对齐安全走廊
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
    // UNKNOWN 是否允许最终轨迹进入
    bool unknown_is_free{true};

    // 路径简化时是否允许直接穿 UNKNOWN
    bool simplify_allow_unknown{false};

    // 参考控制点间距
    double control_point_spacing{0.3};

    // 从每个路径段中心向外扩张安全 Box 的最大距离
    double corridor_max_radius{0.8};

    // 与体素边界留一点余量
    double corridor_margin{0.02};

    // QP 权重
    //
    // weight_smooth:
    //   二阶差分代价，抑制控制点曲率/加速度变化
    //
    // weight_jerk:
    //   三阶差分代价，抑制加速度突变，使轨迹进一步平顺
    //
    // weight_reference:
    //   防止优化结果偏离 A* 参考路径过远
    double weight_smooth{20.0};
    double weight_jerk{5.0};
    double weight_reference{2.0};

    // 初始 B 样条节点时间间隔
    double initial_dt{0.5};

    // XYZ 各轴动力学约束
    double max_vel_axis{2.0};
    double max_acc_axis{2.5};

    // QP 因动力学约束不可行时自动放慢
    double time_scale_factor{1.25};
    int max_time_scaling_attempts{5};

    // 最终轨迹碰撞检查采样间隔
    double validation_dt{0.05};
};


// ============================================================
// 优化结果
// ============================================================
struct OptimizationResult
{
    std::vector<Eigen::Vector3d> simplified_path;
    std::vector<Eigen::Vector3d> reference_points;

    // N 个 B 样条控制点对应 N-3 个 span。
    // 每个 span 使用一个凸安全 Box。
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

    void setOptions(const OptimizerOptions &options);

    void setMap(const uav_mapping::msg::VoxelMap &map);

    bool hasMap() const
    {
        return map_received_;
    }

    // 主入口
    bool optimize(
        const std::vector<Eigen::Vector3d> &astar_path,
        OptimizationResult &result,
        std::string &message);

    // 三次均匀 B 样条求值
    static Eigen::Vector3d evaluateBSpline(
        const std::vector<Eigen::Vector3d> &control_points,
        double dt,
        double t);

    static std::vector<Eigen::Vector3d> sampleBSpline(
        const std::vector<Eigen::Vector3d> &control_points,
        double dt,
        double sample_dt);

private:
    // ============================
    // 地图
    // ============================
    bool validGrid(int x, int y, int z) const;

    int toIndex(int x, int y, int z) const;

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
    // A* 路径处理
    // ============================
    std::vector<Eigen::Vector3d> simplifyPath(
        const std::vector<Eigen::Vector3d> &path) const;

    std::vector<Eigen::Vector3d> resamplePath(
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

    // 从一个安全种子点向 6 个方向扩张 AABB。
    bool buildPointCorridor(
        const Eigen::Vector3d &seed,
        CorridorBox &corridor) const;

    // 第 i 个 B 样条 span 的走廊种子取参考路径段
    // reference[i+1] -> reference[i+2] 的中点。
    bool buildSpanCorridor(
        const std::vector<Eigen::Vector3d> &reference,
        int span_index,
        CorridorBox &corridor) const;

    // ============================
    // QP
    // ============================
    bool solveQP(
        const std::vector<Eigen::Vector3d> &reference,
        const std::vector<CorridorBox> &corridors,
        double dt,
        std::vector<Eigen::Vector3d> &control_points,
        bool enforce_corridors = true);

    bool validateTrajectory(
        const std::vector<Eigen::Vector3d> &control_points,
        double dt) const;

private:
    OptimizerOptions options_;

    bool map_received_{false};

    double resolution_{0.2};

    Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};

    int size_x_{0};
    int size_y_{0};
    int size_z_{0};

    std::vector<int8_t> map_data_;
};

}  // namespace trajectory_optimizer
