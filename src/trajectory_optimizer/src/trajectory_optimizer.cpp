#include "trajectory_optimizer/trajectory_optimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>


namespace trajectory_optimizer
{

// ============================================================
// 参数
// ============================================================

void TrajectoryOptimizer::setOptions(
    const OptimizerOptions &options)
{
    options_ = options;
}


// ============================================================
// 地图
// ============================================================

void TrajectoryOptimizer::setMap(
    const uav_mapping::msg::VoxelMap &map)
{
    resolution_ =
        map.resolution;

    origin_ =
        Eigen::Vector3d(
            map.origin.x,
            map.origin.y,
            map.origin.z);

    size_x_ =
        static_cast<int>(
            map.size_x);

    size_y_ =
        static_cast<int>(
            map.size_y);

    size_z_ =
        static_cast<int>(
            map.size_z);

    map_data_ =
        map.data;


    const std::size_t expected =
        static_cast<std::size_t>(
            size_x_) *
        static_cast<std::size_t>(
            size_y_) *
        static_cast<std::size_t>(
            size_z_);


    map_received_ =
        resolution_ > 0.0 &&
        size_x_ > 0 &&
        size_y_ > 0 &&
        size_z_ > 0 &&
        map_data_.size() == expected;
}


// ============================================================
// Grid工具
// ============================================================

bool TrajectoryOptimizer::validGrid(
    int x,
    int y,
    int z) const
{
    return
        x >= 0 && x < size_x_ &&
        y >= 0 && y < size_y_ &&
        z >= 0 && z < size_z_;
}


int TrajectoryOptimizer::toIndex(
    int x,
    int y,
    int z) const
{
    return
        x +
        size_x_ *
        (y + size_y_ * z);
}


bool TrajectoryOptimizer::worldToGrid(
    const Eigen::Vector3d &p,
    int &x,
    int &y,
    int &z) const
{
    x =
        static_cast<int>(
            std::floor(
                (p.x() - origin_.x()) /
                resolution_));

    y =
        static_cast<int>(
            std::floor(
                (p.y() - origin_.y()) /
                resolution_));

    z =
        static_cast<int>(
            std::floor(
                (p.z() - origin_.z()) /
                resolution_));


    return validGrid(
        x,
        y,
        z);
}


// ============================================================
// voxel是否安全
// ============================================================

bool TrajectoryOptimizer::cellSafe(
    int x,
    int y,
    int z,
    bool allow_unknown) const
{
    if (!validGrid(x, y, z))
        return false;


    const int8_t state =
        map_data_[
            toIndex(x, y, z)];


    // inflated obstacle也已经被mapping写成100
    if (state >= 100)
        return false;


    if (state < 0)
        return allow_unknown;


    return true;
}


bool TrajectoryOptimizer::pointSafe(
    const Eigen::Vector3d &p,
    bool allow_unknown) const
{
    int x;
    int y;
    int z;


    if (!worldToGrid(
            p,
            x,
            y,
            z))
    {
        return false;
    }


    return cellSafe(
        x,
        y,
        z,
        allow_unknown);
}


// ============================================================
// 检查两点之间是否无碰撞
// ============================================================

bool TrajectoryOptimizer::lineSafe(
    const Eigen::Vector3d &a,
    const Eigen::Vector3d &b,
    bool allow_unknown) const
{
    const Eigen::Vector3d diff =
        b - a;


    const double length =
        diff.norm();


    if (length < 1e-8)
    {
        return pointSafe(
            a,
            allow_unknown);
    }


    const double step =
        resolution_ * 0.5;


    const int n =
        std::max(
            1,
            static_cast<int>(
                std::ceil(
                    length /
                    step)));


    for (int i = 0;
         i <= n;
         ++i)
    {
        const double alpha =
            static_cast<double>(i) /
            static_cast<double>(n);


        const Eigen::Vector3d p =
            a +
            alpha * diff;


        if (!pointSafe(
                p,
                allow_unknown))
        {
            return false;
        }
    }


    return true;
}


// ============================================================
// 路径简化
//
// 从当前点开始，尽量连接最远的可见路径点。
// ============================================================

std::vector<Eigen::Vector3d>
TrajectoryOptimizer::simplifyPath(
    const std::vector<Eigen::Vector3d> &path) const
{
    if (path.size() <= 2)
        return path;


    std::vector<Eigen::Vector3d>
        simplified;


    std::size_t current =
        0;


    simplified.push_back(
        path.front());


    while (current <
           path.size() - 1)
    {
        std::size_t next =
            current + 1;


        // 从终点往回找最远可直连点
        for (std::size_t candidate =
                 path.size() - 1;
             candidate >
                 current + 1;
             --candidate)
        {
            if (lineSafe(
                    path[current],
                    path[candidate],
                    options_.
                        simplify_allow_unknown))
            {
                next =
                    candidate;

                break;
            }
        }


        simplified.push_back(
            path[next]);


        current =
            next;
    }


    return simplified;
}


// ============================================================
// 沿折线路径等距离重采样
// ============================================================

std::vector<Eigen::Vector3d>
TrajectoryOptimizer::resamplePath(
    const std::vector<Eigen::Vector3d> &path,
    double spacing) const
{
    if (path.size() < 2)
        return path;


    spacing =
        std::max(
            0.1,
            spacing);


    std::vector<double>
        cumulative(
            path.size(),
            0.0);


    for (std::size_t i = 1;
         i < path.size();
         ++i)
    {
        cumulative[i] =
            cumulative[i - 1] +
            (path[i] -
             path[i - 1]).norm();
    }


    const double total_length =
        cumulative.back();


    if (total_length <
        1e-6)
    {
        return {
            path.front(),
            path.back()
        };
    }


    std::vector<Eigen::Vector3d>
        result;


    result.push_back(
        path.front());


    std::size_t segment =
        0;


    for (double s = spacing;
         s < total_length;
         s += spacing)
    {
        while (
            segment + 1 <
                cumulative.size() &&
            cumulative[
                segment + 1] <
                s)
        {
            ++segment;
        }


        if (segment + 1 >=
            path.size())
        {
            break;
        }


        const double segment_length =
            cumulative[
                segment + 1] -
            cumulative[
                segment];


        if (segment_length <
            1e-8)
        {
            continue;
        }


        const double alpha =
            (s -
             cumulative[
                 segment]) /
            segment_length;


        result.push_back(
            (1.0 - alpha) *
                path[segment] +
            alpha *
                path[segment + 1]);
    }


    if ((result.back() -
         path.back()).norm() >
        1e-6)
    {
        result.push_back(
            path.back());
    }


    return result;
}


// ============================================================
// 检查一个Grid Box是否全部可用
// ============================================================

bool TrajectoryOptimizer::boxSafe(
    int xmin,
    int xmax,
    int ymin,
    int ymax,
    int zmin,
    int zmax) const
{
    for (int z = zmin;
         z <= zmax;
         ++z)
    {
        for (int y = ymin;
             y <= ymax;
             ++y)
        {
            for (int x = xmin;
                 x <= xmax;
                 ++x)
            {
                if (!cellSafe(
                        x,
                        y,
                        z,
                        options_.
                            unknown_is_free))
                {
                    return false;
                }
            }
        }
    }


    return true;
}


// ============================================================
// 从一个参考点向6个方向扩张安全走廊
// ============================================================

bool TrajectoryOptimizer::buildCorridor(
    const Eigen::Vector3d &point,
    CorridorBox &corridor) const
{
    int cx;
    int cy;
    int cz;


    if (!worldToGrid(
            point,
            cx,
            cy,
            cz))
    {
        return false;
    }


    if (!cellSafe(
            cx,
            cy,
            cz,
            options_.unknown_is_free))
    {
        return false;
    }


    int xmin = cx;
    int xmax = cx;

    int ymin = cy;
    int ymax = cy;

    int zmin = cz;
    int zmax = cz;


    const int max_expand =
        std::max(
            1,
            static_cast<int>(
                std::ceil(
                    options_.
                        corridor_max_radius /
                    resolution_)));


    bool changed =
        true;


    while (changed)
    {
        changed =
            false;


        // -X
        if (cx - xmin <
                max_expand &&
            xmin - 1 >= 0 &&
            boxSafe(
                xmin - 1,
                xmin - 1,
                ymin,
                ymax,
                zmin,
                zmax))
        {
            --xmin;
            changed = true;
        }


        // +X
        if (xmax - cx <
                max_expand &&
            xmax + 1 <
                size_x_ &&
            boxSafe(
                xmax + 1,
                xmax + 1,
                ymin,
                ymax,
                zmin,
                zmax))
        {
            ++xmax;
            changed = true;
        }


        // -Y
        if (cy - ymin <
                max_expand &&
            ymin - 1 >= 0 &&
            boxSafe(
                xmin,
                xmax,
                ymin - 1,
                ymin - 1,
                zmin,
                zmax))
        {
            --ymin;
            changed = true;
        }


        // +Y
        if (ymax - cy <
                max_expand &&
            ymax + 1 <
                size_y_ &&
            boxSafe(
                xmin,
                xmax,
                ymax + 1,
                ymax + 1,
                zmin,
                zmax))
        {
            ++ymax;
            changed = true;
        }


        // -Z
        if (cz - zmin <
                max_expand &&
            zmin - 1 >= 0 &&
            boxSafe(
                xmin,
                xmax,
                ymin,
                ymax,
                zmin - 1,
                zmin - 1))
        {
            --zmin;
            changed = true;
        }


        // +Z
        if (zmax - cz <
                max_expand &&
            zmax + 1 <
                size_z_ &&
            boxSafe(
                xmin,
                xmax,
                ymin,
                ymax,
                zmax + 1,
                zmax + 1))
        {
            ++zmax;
            changed = true;
        }
    }


    double margin =
        std::min(
            options_.corridor_margin,
            resolution_ * 0.4);


    corridor.min =
        origin_ +
        resolution_ *
        Eigen::Vector3d(
            xmin,
            ymin,
            zmin) +
        Eigen::Vector3d::Constant(
            margin);


    corridor.max =
        origin_ +
        resolution_ *
        Eigen::Vector3d(
            xmax + 1,
            ymax + 1,
            zmax + 1) -
        Eigen::Vector3d::Constant(
            margin);


    // 保证参考点本身一定包含在box内
    for (int d = 0;
         d < 3;
         ++d)
    {
        corridor.min[d] =
            std::min(
                corridor.min[d],
                point[d]);

        corridor.max[d] =
            std::max(
                corridor.max[d],
                point[d]);
    }


    return true;
}


// ============================================================
// QP求解
// ============================================================

bool TrajectoryOptimizer::solveQP(
    const std::vector<Eigen::Vector3d> &reference,
    const std::vector<CorridorBox> &corridors,
    double dt,
    std::vector<Eigen::Vector3d> &control_points)
{
    const int N =
        static_cast<int>(
            reference.size());


    if (N < 6 ||
        corridors.size() !=
            reference.size())
    {
        return false;
    }


    const int n_var =
        3 * N;


    auto varIndex =
        [](int i, int dim)
        {
            return
                3 * i + dim;
        };


    // ========================================================
    // Hessian P
    // ========================================================

    std::vector<
        Eigen::Triplet<double>>
        h_triplets;


    Eigen::VectorXd gradient =
        Eigen::VectorXd::Zero(
            n_var);


    // --------------------------------------------------------
    // Reference cost
    //
    // wr * ||Q-R||²
    // --------------------------------------------------------

    for (int i = 0;
         i < N;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            const int idx =
                varIndex(
                    i,
                    d);


            h_triplets.emplace_back(
                idx,
                idx,
                2.0 *
                options_.
                    weight_reference);


            gradient[idx] +=
                -2.0 *
                options_.
                    weight_reference *
                reference[i][d];
        }
    }


    // --------------------------------------------------------
    // Smooth cost
    //
    // ws * ||Qi - 2Qi+1 + Qi+2||²
    // --------------------------------------------------------

    const double coeff[3] =
        {
            1.0,
            -2.0,
            1.0
        };


    for (int i = 0;
         i + 2 < N;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            for (int a = 0;
                 a < 3;
                 ++a)
            {
                for (int b = a;
                     b < 3;
                     ++b)
                {
                    const int ia =
                        varIndex(
                            i + a,
                            d);

                    const int ib =
                        varIndex(
                            i + b,
                            d);


                    h_triplets.emplace_back(
                        ia,
                        ib,
                        2.0 *
                        options_.
                            weight_smooth *
                        coeff[a] *
                        coeff[b]);
                }
            }
        }
    }


    Eigen::SparseMatrix<double>
        hessian(
            n_var,
            n_var);


    hessian.setFromTriplets(
        h_triplets.begin(),
        h_triplets.end());


    hessian.makeCompressed();


    // ========================================================
    // Constraint A
    // ========================================================

    std::vector<
        Eigen::Triplet<double>>
        a_triplets;


    std::vector<double>
        lower_values;

    std::vector<double>
        upper_values;


    int row =
        0;


    auto addRow =
        [&](
            const std::vector<
                std::pair<int, double>>
                &coefficients,

            double lower,
            double upper)
        {
            for (const auto &item :
                 coefficients)
            {
                a_triplets.emplace_back(
                    row,
                    item.first,
                    item.second);
            }


            lower_values.push_back(
                lower);

            upper_values.push_back(
                upper);

            ++row;
        };


    // --------------------------------------------------------
    // 1. 安全走廊
    // --------------------------------------------------------

    for (int i = 0;
         i < N;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            addRow(
                {
                    {
                        varIndex(i, d),
                        1.0
                    }
                },

                corridors[i].min[d],

                corridors[i].max[d]);
        }
    }


    // --------------------------------------------------------
    // 2. 起点固定
    //
    // Q0 = Q1 = Q2 = start
    // --------------------------------------------------------

    const Eigen::Vector3d start =
        reference.front();


    for (int i = 0;
         i < 3;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            addRow(
                {
                    {
                        varIndex(i, d),
                        1.0
                    }
                },

                start[d],
                start[d]);
        }
    }


    // --------------------------------------------------------
    // 3. 终点固定
    //
    // QN-3 = QN-2 = QN-1 = goal
    // --------------------------------------------------------

    const Eigen::Vector3d goal =
        reference.back();


    for (int i = N - 3;
         i < N;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            addRow(
                {
                    {
                        varIndex(i, d),
                        1.0
                    }
                },

                goal[d],
                goal[d]);
        }
    }


    // --------------------------------------------------------
    // 4. 速度约束
    //
    // |Q(i+1)-Qi| <= vmax * dt
    // --------------------------------------------------------

    const double velocity_bound =
        options_.max_vel_axis *
        dt;


    for (int i = 0;
         i + 1 < N;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            addRow(
                {
                    {
                        varIndex(i, d),
                        -1.0
                    },

                    {
                        varIndex(i + 1, d),
                        1.0
                    }
                },

                -velocity_bound,

                velocity_bound);
        }
    }


    // --------------------------------------------------------
    // 5. 加速度约束
    //
    // |Qi - 2Qi+1 + Qi+2|
    // <= amax * dt²
    // --------------------------------------------------------

    const double acceleration_bound =
        options_.max_acc_axis *
        dt *
        dt;


    for (int i = 0;
         i + 2 < N;
         ++i)
    {
        for (int d = 0;
             d < 3;
             ++d)
        {
            addRow(
                {
                    {
                        varIndex(i, d),
                        1.0
                    },

                    {
                        varIndex(i + 1, d),
                        -2.0
                    },

                    {
                        varIndex(i + 2, d),
                        1.0
                    }
                },

                -acceleration_bound,

                acceleration_bound);
        }
    }


    Eigen::SparseMatrix<double>
        constraint_matrix(
            row,
            n_var);


    constraint_matrix.setFromTriplets(
        a_triplets.begin(),
        a_triplets.end());


    constraint_matrix.makeCompressed();


    Eigen::VectorXd lower_bound(
        row);

    Eigen::VectorXd upper_bound(
        row);


    for (int i = 0;
         i < row;
         ++i)
    {
        lower_bound[i] =
            lower_values[i];

        upper_bound[i] =
            upper_values[i];
    }


    // ========================================================
    // OSQP
    // ========================================================

    OsqpEigen::Solver solver;


    solver.settings()->
        setVerbosity(
            false);

    solver.settings()->
        setWarmStart(
            true);


    solver.data()->
        setNumberOfVariables(
            n_var);

    solver.data()->
        setNumberOfConstraints(
            row);


    if (!solver.data()->
            setHessianMatrix(
                hessian))
    {
        return false;
    }


    if (!solver.data()->
            setGradient(
                gradient))
    {
        return false;
    }


    if (!solver.data()->
            setLinearConstraintsMatrix(
                constraint_matrix))
    {
        return false;
    }


    if (!solver.data()->
            setLowerBound(
                lower_bound))
    {
        return false;
    }


    if (!solver.data()->
            setUpperBound(
                upper_bound))
    {
        return false;
    }


    if (!solver.initSolver())
    {
        return false;
    }


    if (solver.solveProblem() !=
        OsqpEigen::
            ErrorExitFlag::NoError)
    {
        return false;
    }


    // 不仅API要成功，
    // QP本身也必须真的求解成功
    if (solver.getStatus() !=
        OsqpEigen::Status::Solved)
    {
        return false;
    }


    const Eigen::VectorXd solution =
        solver.getSolution();


    if (solution.size() !=
        n_var ||
        !solution.allFinite())
    {
        return false;
    }


    control_points.resize(
        N);


    for (int i = 0;
         i < N;
         ++i)
    {
        control_points[i] =
            Eigen::Vector3d(
                solution[
                    varIndex(i, 0)],

                solution[
                    varIndex(i, 1)],

                solution[
                    varIndex(i, 2)]);
    }


    return true;
}


// ============================================================
// 三次均匀B样条求值
// ============================================================

Eigen::Vector3d
TrajectoryOptimizer::evaluateBSpline(
    const std::vector<Eigen::Vector3d> &control_points,
    double dt,
    double t)
{
    if (control_points.size() <
            4 ||
        dt <= 0.0)
    {
        return
            Eigen::Vector3d::Zero();
    }


    const int segment_num =
        static_cast<int>(
            control_points.size()) -
        3;


    const double duration =
        segment_num *
        dt;


    t =
        std::clamp(
            t,
            0.0,
            duration);


    int segment =
        static_cast<int>(
            std::floor(
                t / dt));


    double u;


    if (segment >=
        segment_num)
    {
        segment =
            segment_num - 1;

        u =
            1.0;
    }
    else
    {
        u =
            (t -
             segment * dt) /
            dt;
    }


    const double u2 =
        u * u;

    const double u3 =
        u2 * u;


    // 三次均匀B样条basis
    const double B0 =
        (1.0 -
         3.0 * u +
         3.0 * u2 -
         u3) /
        6.0;


    const double B1 =
        (4.0 -
         6.0 * u2 +
         3.0 * u3) /
        6.0;


    const double B2 =
        (1.0 +
         3.0 * u +
         3.0 * u2 -
         3.0 * u3) /
        6.0;


    const double B3 =
        u3 /
        6.0;


    return
        B0 *
            control_points[
                segment] +

        B1 *
            control_points[
                segment + 1] +

        B2 *
            control_points[
                segment + 2] +

        B3 *
            control_points[
                segment + 3];
}


// ============================================================
// B样条采样
// ============================================================

std::vector<Eigen::Vector3d>
TrajectoryOptimizer::sampleBSpline(
    const std::vector<Eigen::Vector3d> &control_points,
    double dt,
    double sample_dt)
{
    std::vector<Eigen::Vector3d>
        result;


    if (control_points.size() <
            4 ||
        dt <= 0.0)
    {
        return result;
    }


    const double duration =
        (
            static_cast<int>(
                control_points.size()) -
            3
        ) *
        dt;


    sample_dt =
        std::max(
            0.01,
            sample_dt);


    for (double t = 0.0;
         t < duration;
         t += sample_dt)
    {
        result.push_back(
            evaluateBSpline(
                control_points,
                dt,
                t));
    }


    result.push_back(
        evaluateBSpline(
            control_points,
            dt,
            duration));


    return result;
}


// ============================================================
// 最终碰撞检查
// ============================================================

bool TrajectoryOptimizer::validateTrajectory(
    const std::vector<Eigen::Vector3d> &control_points,
    double dt) const
{
    const auto samples =
        sampleBSpline(
            control_points,
            dt,
            options_.validation_dt);


    for (const auto &p :
         samples)
    {
        if (!pointSafe(
                p,
                options_.
                    unknown_is_free))
        {
            return false;
        }
    }


    return true;
}


// ============================================================
// 主优化过程
// ============================================================

bool TrajectoryOptimizer::optimize(
    const std::vector<Eigen::Vector3d> &astar_path,
    OptimizationResult &result,
    std::string &message)
{
    result =
        OptimizationResult{};


    if (!map_received_)
    {
        message =
            "尚未收到VoxelMap";

        return false;
    }


    if (astar_path.size() <
        2)
    {
        message =
            "A*路径点数量不足";

        return false;
    }


    // ========================================================
    // 1. 路径简化
    // ========================================================

    result.simplified_path =
        simplifyPath(
            astar_path);


    if (result.simplified_path.size() <
        2)
    {
        message =
            "路径简化失败";

        return false;
    }


    // ========================================================
    // 2. 等距离重采样
    // ========================================================

    const auto sampled_path =
        resamplePath(
            result.simplified_path,
            options_.
                control_point_spacing);


    if (sampled_path.size() <
        2)
    {
        message =
            "路径重采样失败";

        return false;
    }


    // ========================================================
    // 3. 构造参考控制点
    //
    // [S,S] + [S,...,G] + [G,G]
    //
    // 最终前三个都是S，
    // 最后三个都是G。
    // ========================================================

    const Eigen::Vector3d start =
        sampled_path.front();

    const Eigen::Vector3d goal =
        sampled_path.back();


    result.reference_points.clear();


    result.reference_points.push_back(
        start);

    result.reference_points.push_back(
        start);


    for (const auto &p :
         sampled_path)
    {
        result.reference_points.push_back(
            p);
    }


    result.reference_points.push_back(
        goal);

    result.reference_points.push_back(
        goal);


    // ========================================================
    // 4. 安全走廊
    // ========================================================

    result.corridors.resize(
        result.reference_points.size());


    for (std::size_t i = 0;
         i <
         result.reference_points.size();
         ++i)
    {
        if (!buildCorridor(
                result.reference_points[i],
                result.corridors[i]))
        {
            message =
                "安全走廊生成失败，控制点编号: " +
                std::to_string(i);

            return false;
        }
    }


    // ========================================================
    // 5. QP
    //
    // 如果dt太小导致速度/加速度约束不可行，
    // 自动增加dt。
    // ========================================================

    double dt =
        options_.initial_dt;


    for (int attempt = 0;
         attempt <
             options_.
                 max_time_scaling_attempts;
         ++attempt)
    {
        std::vector<
            Eigen::Vector3d>
            control_points;


        const bool solved =
            solveQP(
                result.reference_points,
                result.corridors,
                dt,
                control_points);


        if (solved &&
            validateTrajectory(
                control_points,
                dt))
        {
            result.control_points =
                control_points;

            result.dt =
                dt;

            result.duration =
                (
                    static_cast<int>(
                        control_points.size()) -
                    3
                ) *
                dt;


            message =
                "B样条QP优化成功，控制点数=" +
                std::to_string(
                    control_points.size()) +
                "，dt=" +
                std::to_string(dt) +
                "，duration=" +
                std::to_string(
                    result.duration);


            return true;
        }


        // 轨迹时间变长以后，
        // 速度/加速度约束更容易满足
        dt *=
            options_.
                time_scale_factor;
    }


    message =
        "B样条QP求解失败或最终轨迹发生碰撞";

    return false;
}

}  // namespace trajectory_optimizer