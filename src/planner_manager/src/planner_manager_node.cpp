#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <astar_planner/srv/plan_path.hpp>
#include <planner_manager_interfaces/action/navigate_to_goal.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <trajectory_optimizer/msg/b_spline_trajectory.hpp>
#include <uav_mapping/msg/voxel_map.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


class PlannerManagerNode : public rclcpp::Node
{
public:
    using NavigateToGoal =
        planner_manager_interfaces::action::NavigateToGoal;

    using GoalHandleNavigate =
        rclcpp_action::ServerGoalHandle<NavigateToGoal>;

    using PlanPath =
        astar_planner::srv::PlanPath;


    PlannerManagerNode()
        : Node("planner_manager")
    {
        // =====================================================
        // 1. ROS接口参数
        // =====================================================
        action_name_ =
            declare_parameter<std::string>(
                "action_name",
                "/navigate_to_goal");

        astar_service_name_ =
            declare_parameter<std::string>(
                "astar_service_name",
                "/astar/plan");

        odom_topic_ =
            declare_parameter<std::string>(
                "odom_topic",
                "/fmu/out/vehicle_odometry");

        map_topic_ =
            declare_parameter<std::string>(
                "map_topic",
                "/uav_mapping/voxel_map");

        bspline_topic_ =
            declare_parameter<std::string>(
                "bspline_topic",
                "/trajectory_optimizer/bspline");


        // =====================================================
        // 2. 到达目标判定
        // =====================================================
        goal_tolerance_ =
            declare_parameter<double>(
                "goal_tolerance",
                0.30);

        goal_hold_time_ =
            declare_parameter<double>(
                "goal_hold_time",
                0.50);


        // =====================================================
        // 3. 初始起飞规划
        // =====================================================
        ground_z_threshold_ =
            declare_parameter<double>(
                "ground_z_threshold",
                0.50);

        takeoff_height_ =
            declare_parameter<double>(
                "takeoff_height",
                2.50);


        // =====================================================
        // 4. 在线轨迹安全检测 / Replan参数
        // =====================================================

        // 是否启用在线碰撞检测
        enable_replan_ =
            declare_parameter<bool>(
                "enable_replan",
                true);

        // 每秒检查几次当前B样条
        safety_check_rate_ =
            declare_parameter<double>(
                "safety_check_rate",
                10.0);

        // 只检查飞机前方这么长的一段轨迹。
        //
        // 不需要每次检查整条轨迹：
        // 真正需要马上处理的是“即将飞到”的部分。
        lookahead_distance_ =
            declare_parameter<double>(
                "lookahead_distance",
                4.0);

        // B样条按时间采样的间隔
        trajectory_sample_dt_ =
            declare_parameter<double>(
                "trajectory_sample_dt",
                0.05);

        // UNKNOWN 是否也认为危险。
        //
        // 对当前“未知静态环境探索式导航”：
        // 建议 false。
        //
        // 因为A*本身允许在UNKNOWN中搜索；
        // 真正发现障碍物并变成OCCUPIED后才触发重规划。
        unknown_is_collision_ =
            declare_parameter<bool>(
                "unknown_is_collision",
                false);

        // 轨迹跑出VoxelMap边界必须认为不安全
        outside_map_is_collision_ =
            declare_parameter<bool>(
                "outside_map_is_collision",
                true);

        // 两次重规划至少间隔这么久，
        // 防止同一地图更新连续触发。
        replan_cooldown_ =
            declare_parameter<double>(
                "replan_cooldown",
                1.0);

        // 单次任务最多允许多少次重规划
        max_replans_ =
            declare_parameter<int>(
                "max_replans",
                10);


        // =====================================================
        // 5. 超时参数
        // =====================================================
        service_wait_timeout_ =
            declare_parameter<double>(
                "service_wait_timeout",
                3.0);

        planning_timeout_ =
            declare_parameter<double>(
                "planning_timeout",
                10.0);

        optimizer_timeout_ =
            declare_parameter<double>(
                "optimizer_timeout",
                5.0);

        execution_timeout_ =
            declare_parameter<double>(
                "execution_timeout",
                120.0);

        feedback_rate_ =
            declare_parameter<double>(
                "feedback_rate",
                10.0);


        // =====================================================
        // 6. A* Service Client
        // =====================================================
        astar_client_ =
            create_client<PlanPath>(
                astar_service_name_);


        // =====================================================
        // 7. PX4 Odometry
        // =====================================================
        odom_sub_ =
            create_subscription<
                px4_msgs::msg::VehicleOdometry>(
                odom_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &PlannerManagerNode::odomCallback,
                    this,
                    std::placeholders::_1));


        // =====================================================
        // 8. VoxelMap
        //
        // 使用与规划器相同的VoxelMap。
        // 其中膨胀障碍应已经被标记为OCCUPIED。
        // =====================================================
        auto map_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        map_sub_ =
            create_subscription<
                uav_mapping::msg::VoxelMap>(
                map_topic_,
                map_qos,
                std::bind(
                    &PlannerManagerNode::mapCallback,
                    this,
                    std::placeholders::_1));


        // =====================================================
        // 9. B样条
        //
        // 一方面用于判断optimizer是否生成了新轨迹；
        // 另一方面用于飞行中的在线碰撞检测。
        // =====================================================
        auto trajectory_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        bspline_sub_ =
            create_subscription<
                trajectory_optimizer::msg::BSplineTrajectory>(
                bspline_topic_,
                trajectory_qos,
                std::bind(
                    &PlannerManagerNode::bsplineCallback,
                    this,
                    std::placeholders::_1));


        // =====================================================
        // 10. NavigateToGoal Action Server
        // =====================================================
        action_server_ =
            rclcpp_action::create_server<NavigateToGoal>(
                this,
                action_name_,
                std::bind(
                    &PlannerManagerNode::handleGoal,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
                std::bind(
                    &PlannerManagerNode::handleCancel,
                    this,
                    std::placeholders::_1),
                std::bind(
                    &PlannerManagerNode::handleAccepted,
                    this,
                    std::placeholders::_1));


        RCLCPP_INFO(
            get_logger(),
            "planner_manager启动");

        RCLCPP_INFO(
            get_logger(),
            "在线重规划: %s, lookahead=%.2f m, check_rate=%.1f Hz",
            enable_replan_ ? "ON" : "OFF",
            lookahead_distance_,
            safety_check_rate_);
    }


private:
    // =========================================================
    // PX4里程计：NED -> ENU
    // =========================================================
    void odomCallback(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        if (msg->pose_frame !=
            px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
        {
            return;
        }

        if (!std::isfinite(msg->position[0]) ||
            !std::isfinite(msg->position[1]) ||
            !std::isfinite(msg->position[2]))
        {
            return;
        }

        std::lock_guard<std::mutex>
            lock(state_mutex_);

        current_position_.x =
            msg->position[1];

        current_position_.y =
            msg->position[0];

        current_position_.z =
            -msg->position[2];

        odom_received_ =
            true;
    }


    // =========================================================
    // 更新地图
    // =========================================================
    void mapCallback(
        const uav_mapping::msg::VoxelMap::SharedPtr msg)
    {
        const std::size_t expected =
            static_cast<std::size_t>(msg->size_x) *
            static_cast<std::size_t>(msg->size_y) *
            static_cast<std::size_t>(msg->size_z);

        if (msg->resolution <= 0.0 ||
            msg->size_x == 0 ||
            msg->size_y == 0 ||
            msg->size_z == 0 ||
            msg->data.size() != expected)
        {
            return;
        }

        std::lock_guard<std::mutex>
            lock(map_mutex_);

        latest_map_ =
            *msg;

        map_received_ =
            true;
    }


    // =========================================================
    // 收到新的B样条
    // =========================================================
    void bsplineCallback(
        const trajectory_optimizer::msg::
            BSplineTrajectory::SharedPtr msg)
    {
        if (msg->degree != 3 ||
            msg->control_points.size() < 4 ||
            msg->dt <= 0.0)
        {
            return;
        }

        {
            std::lock_guard<std::mutex>
                lock(trajectory_mutex_);

            latest_trajectory_ =
                *msg;

            latest_trajectory_valid_ =
                true;

            ++trajectory_generation_;
        }

        trajectory_cv_.notify_all();
    }


    // =========================================================
    // Action Goal
    // =========================================================
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<
            const NavigateToGoal::Goal> goal)
    {
        const auto &p =
            goal->goal_pose.pose.position;

        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
        {
            RCLCPP_WARN(
                get_logger(),
                "拒绝目标：坐标非法");

            return
                rclcpp_action::GoalResponse::REJECT;
        }

        const std::string &frame =
            goal->goal_pose.header.frame_id;

        if (!frame.empty() &&
            frame != "map")
        {
            RCLCPP_WARN(
                get_logger(),
                "拒绝目标：当前只支持map/ENU，收到frame=%s",
                frame.c_str());

            return
                rclcpp_action::GoalResponse::REJECT;
        }

        {
            std::lock_guard<std::mutex>
                lock(state_mutex_);

            if (!odom_received_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "拒绝目标：尚未收到PX4里程计");

                return
                    rclcpp_action::GoalResponse::REJECT;
            }
        }

        {
            std::lock_guard<std::mutex>
                lock(map_mutex_);

            if (!map_received_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "拒绝目标：尚未收到VoxelMap");

                return
                    rclcpp_action::GoalResponse::REJECT;
            }
        }

        bool expected = false;

        if (!goal_active_.compare_exchange_strong(
                expected,
                true))
        {
            RCLCPP_WARN(
                get_logger(),
                "拒绝目标：已有导航任务正在执行");

            return
                rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(
            get_logger(),
            "接受目标: (%.2f, %.2f, %.2f)",
            p.x,
            p.y,
            p.z);

        return
            rclcpp_action::GoalResponse::
                ACCEPT_AND_EXECUTE;
    }


    // =========================================================
    // Cancel
    //
    // 当前controller还没有独立的安全HOLD/STOP服务，
    // 因此这里仍然拒绝Action cancel。
    // =========================================================
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<
            GoalHandleNavigate>)
    {
        RCLCPP_WARN(
            get_logger(),
            "当前版本暂不支持安全Cancel");

        return
            rclcpp_action::CancelResponse::REJECT;
    }


    void handleAccepted(
        const std::shared_ptr<
            GoalHandleNavigate> goal_handle)
    {
        std::thread(
            &PlannerManagerNode::execute,
            this,
            goal_handle)
            .detach();
    }


    // =========================================================
    // 获取当前位置
    // =========================================================
    geometry_msgs::msg::Point getCurrentPosition()
    {
        std::lock_guard<std::mutex>
            lock(state_mutex_);

        return
            current_position_;
    }


    // =========================================================
    // 欧式距离
    // =========================================================
    static double distance(
        const geometry_msgs::msg::Point &a,
        const geometry_msgs::msg::Point &b)
    {
        const double dx =
            a.x - b.x;

        const double dy =
            a.y - b.y;

        const double dz =
            a.z - b.z;

        return
            std::sqrt(
                dx * dx +
                dy * dy +
                dz * dz);
    }


    // =========================================================
    // Action Feedback
    // =========================================================
    void publishFeedback(
        const std::shared_ptr<
            GoalHandleNavigate> &goal_handle,
        const std::string &state,
        const geometry_msgs::msg::Point &goal,
        double initial_distance)
    {
        const auto current =
            getCurrentPosition();

        const double dist =
            distance(
                current,
                goal);

        double progress = 0.0;

        if (initial_distance < 1e-6)
        {
            progress = 1.0;
        }
        else
        {
            progress =
                1.0 -
                dist /
                initial_distance;

            progress =
                std::clamp(
                    progress,
                    0.0,
                    1.0);
        }

        auto feedback =
            std::make_shared<
                NavigateToGoal::Feedback>();

        feedback->state =
            state;

        feedback->distance_to_goal =
            static_cast<float>(
                dist);

        feedback->progress =
            static_cast<float>(
                progress);

        goal_handle->publish_feedback(
            feedback);
    }


    // =========================================================
    // Abort
    // =========================================================
    void abortGoal(
        const std::shared_ptr<
            GoalHandleNavigate> &goal_handle,
        const std::string &message,
        double start_time)
    {
        auto result =
            std::make_shared<
                NavigateToGoal::Result>();

        result->success =
            false;

        result->message =
            message;

        result->total_time =
            static_cast<float>(
                std::max(
                    0.0,
                    now().seconds() -
                    start_time));

        goal_handle->abort(
            result);

        goal_active_ =
            false;

        RCLCPP_ERROR(
            get_logger(),
            "导航失败: %s",
            message.c_str());
    }


    // =========================================================
    // 一次完整的：
    //
    // A* -> /astar/path -> Optimizer -> 新B样条
    //
    // 返回true表示新轨迹已经生成。
    // =========================================================
    bool planAndWaitTrajectory(
        const geometry_msgs::msg::Point &start,
        const geometry_msgs::msg::Point &goal,
        const std::shared_ptr<
            GoalHandleNavigate> &goal_handle,
        double initial_distance,
        bool is_replan,
        std::string &error_message)
    {
        // -----------------------------------------------------
        // 记录旧B样条版本号
        // -----------------------------------------------------
        uint64_t generation_before;

        {
            std::lock_guard<std::mutex>
                lock(trajectory_mutex_);

            generation_before =
                trajectory_generation_;
        }


        // -----------------------------------------------------
        // 等A* Service
        // -----------------------------------------------------
        if (!astar_client_->wait_for_service(
                std::chrono::duration<double>(
                    service_wait_timeout_)))
        {
            error_message =
                "等待 /astar/plan Service 超时";

            return false;
        }


        publishFeedback(
            goal_handle,
            is_replan ?
                "REPLANNING" :
                "PLANNING",
            goal,
            initial_distance);


        auto request =
            std::make_shared<
                PlanPath::Request>();

        request->start =
            start;

        request->goal =
            goal;


        RCLCPP_INFO(
            get_logger(),
            "%sA*: start=(%.2f %.2f %.2f), goal=(%.2f %.2f %.2f)",
            is_replan ? "重新" : "",
            start.x,
            start.y,
            start.z,
            goal.x,
            goal.y,
            goal.z);


        auto future =
            astar_client_->async_send_request(
                request);


        if (future.wait_for(
                std::chrono::duration<double>(
                    planning_timeout_)) !=
            std::future_status::ready)
        {
            error_message =
                is_replan ?
                "重新A*规划超时" :
                "A*规划超时";

            return false;
        }


        const auto response =
            future.get();


        if (!response->success)
        {
            error_message =
                std::string(
                    is_replan ?
                    "重新A*失败: " :
                    "A*失败: ") +
                response->message;

            return false;
        }


        RCLCPP_INFO(
            get_logger(),
            "%sA*成功，路径点=%zu",
            is_replan ? "重新" : "",
            response->path.poses.size());


        // -----------------------------------------------------
        // astar_node成功后会发布 /astar/path，
        // trajectory_optimizer会自动开始优化。
        // -----------------------------------------------------
        publishFeedback(
            goal_handle,
            is_replan ?
                "REOPTIMIZING" :
                "OPTIMIZING",
            goal,
            initial_distance);


        {
            std::unique_lock<std::mutex>
                lock(trajectory_mutex_);

            const bool got_new_trajectory =
                trajectory_cv_.wait_for(
                    lock,
                    std::chrono::duration<double>(
                        optimizer_timeout_),
                    [&]()
                    {
                        return
                            trajectory_generation_ >
                            generation_before;
                    });

            if (!got_new_trajectory)
            {
                error_message =
                    is_replan ?
                    "重新规划A*成功，但没有收到新的B样条" :
                    "A*成功，但没有收到新的B样条";

                return false;
            }
        }


        RCLCPP_INFO(
            get_logger(),
            "%s轨迹生成成功",
            is_replan ?
                "重新规划" :
                "初始规划");


        return true;
    }


    // =========================================================
    // 三次均匀B样条位置求值
    //
    // 这里只需要位置，不需要v/a/jerk。
    // =========================================================
    static geometry_msgs::msg::Point evaluateBSpline(
        const trajectory_optimizer::msg::
            BSplineTrajectory &trajectory,
        double t)
    {
        geometry_msgs::msg::Point result;

        const int N =
            static_cast<int>(
                trajectory.control_points.size());

        if (N < 4 ||
            trajectory.dt <= 0.0)
        {
            return result;
        }

        const int segment_num =
            N - 3;

        const double duration =
            segment_num *
            trajectory.dt;

        t =
            std::clamp(
                t,
                0.0,
                duration);

        int segment =
            static_cast<int>(
                std::floor(
                    t /
                    trajectory.dt));

        double u = 0.0;

        if (segment >= segment_num)
        {
            segment =
                segment_num - 1;

            u =
                1.0;
        }
        else
        {
            u =
                (
                    t -
                    segment *
                    trajectory.dt
                ) /
                trajectory.dt;
        }

        const double u2 =
            u * u;

        const double u3 =
            u2 * u;

        const double B0 =
            (
                1.0 -
                3.0 * u +
                3.0 * u2 -
                u3
            ) /
            6.0;

        const double B1 =
            (
                4.0 -
                6.0 * u2 +
                3.0 * u3
            ) /
            6.0;

        const double B2 =
            (
                1.0 +
                3.0 * u +
                3.0 * u2 -
                3.0 * u3
            ) /
            6.0;

        const double B3 =
            u3 /
            6.0;


        const auto &Q0 =
            trajectory.control_points[
                segment];

        const auto &Q1 =
            trajectory.control_points[
                segment + 1];

        const auto &Q2 =
            trajectory.control_points[
                segment + 2];

        const auto &Q3 =
            trajectory.control_points[
                segment + 3];


        result.x =
            B0 * Q0.x +
            B1 * Q1.x +
            B2 * Q2.x +
            B3 * Q3.x;

        result.y =
            B0 * Q0.y +
            B1 * Q1.y +
            B2 * Q2.y +
            B3 * Q3.y;

        result.z =
            B0 * Q0.z +
            B1 * Q1.z +
            B2 * Q2.z +
            B3 * Q3.z;

        return result;
    }


    // =========================================================
    // 世界坐标 -> voxel index
    // =========================================================
    static bool worldToGrid(
        const uav_mapping::msg::VoxelMap &map,
        const geometry_msgs::msg::Point &p,
        int &x,
        int &y,
        int &z)
    {
        x =
            static_cast<int>(
                std::floor(
                    (
                        p.x -
                        map.origin.x
                    ) /
                    map.resolution));

        y =
            static_cast<int>(
                std::floor(
                    (
                        p.y -
                        map.origin.y
                    ) /
                    map.resolution));

        z =
            static_cast<int>(
                std::floor(
                    (
                        p.z -
                        map.origin.z
                    ) /
                    map.resolution));


        return
            x >= 0 &&
            x <
                static_cast<int>(
                    map.size_x) &&

            y >= 0 &&
            y <
                static_cast<int>(
                    map.size_y) &&

            z >= 0 &&
            z <
                static_cast<int>(
                    map.size_z);
    }


    // =========================================================
    // 一个点在最新VoxelMap里是否碰撞
    // =========================================================
    bool pointCollision(
        const uav_mapping::msg::VoxelMap &map,
        const geometry_msgs::msg::Point &p) const
    {
        int x;
        int y;
        int z;

        if (!worldToGrid(
                map,
                p,
                x,
                y,
                z))
        {
            return
                outside_map_is_collision_;
        }


        const int index =
            x +
            static_cast<int>(map.size_x) *
            (
                y +
                static_cast<int>(map.size_y) *
                z
            );


        if (index < 0 ||
            index >=
                static_cast<int>(
                    map.data.size()))
        {
            return
                outside_map_is_collision_;
        }


        const int8_t state =
            map.data[index];


        // OCCUPIED / inflated obstacle
        if (state >= 100)
        {
            return true;
        }


        // UNKNOWN
        if (state < 0)
        {
            return
                unknown_is_collision_;
        }


        return false;
    }


    // =========================================================
    // 两个轨迹采样点之间继续做空间采样。
    //
    // 这样即使B样条时间采样刚好跨过一个障碍体素，
    // 也不会漏检。
    // =========================================================
    bool segmentCollision(
        const uav_mapping::msg::VoxelMap &map,
        const geometry_msgs::msg::Point &a,
        const geometry_msgs::msg::Point &b,
        geometry_msgs::msg::Point &collision_point) const
    {
        const double length =
            distance(
                a,
                b);

        if (length < 1e-8)
        {
            if (pointCollision(
                    map,
                    a))
            {
                collision_point =
                    a;

                return true;
            }

            return false;
        }


        // 保证空间检测步长不超过半个voxel。
        const double step =
            std::max(
                0.02,
                std::min(
                    0.10,
                    0.5 *
                    static_cast<double>(
                        map.resolution)));


        const int sample_num =
            std::max(
                1,
                static_cast<int>(
                    std::ceil(
                        length /
                        step)));


        for (int i = 0;
             i <= sample_num;
             ++i)
        {
            const double alpha =
                static_cast<double>(i) /
                static_cast<double>(
                    sample_num);

            geometry_msgs::msg::Point p;

            p.x =
                a.x +
                alpha *
                (b.x - a.x);

            p.y =
                a.y +
                alpha *
                (b.y - a.y);

            p.z =
                a.z +
                alpha *
                (b.z - a.z);


            if (pointCollision(
                    map,
                    p))
            {
                collision_point =
                    p;

                return true;
            }
        }


        return false;
    }


    // =========================================================
    // 检查“从无人机当前位置开始，前方lookahead_distance”
    // 的B样条是否已经被最新地图中的障碍物挡住。
    //
    // 核心：
    //
    // 1. 对完整B样条采样；
    // 2. 找到离当前无人机最近的轨迹采样点；
    // 3. 只从这个点向前检查；
    // 4. 最多检查lookahead_distance。
    //
    // 因此不会因为轨迹后方已经经过的障碍而重规划。
    // =========================================================
    bool trajectoryCollisionAhead(
        const geometry_msgs::msg::Point &current,
        geometry_msgs::msg::Point &collision_point)
    {
        uav_mapping::msg::VoxelMap map;

        trajectory_optimizer::msg::
            BSplineTrajectory trajectory;


        {
            std::lock_guard<std::mutex>
                lock(map_mutex_);

            if (!map_received_)
            {
                return false;
            }

            map =
                latest_map_;
        }


        {
            std::lock_guard<std::mutex>
                lock(trajectory_mutex_);

            if (!latest_trajectory_valid_)
            {
                return false;
            }

            trajectory =
                latest_trajectory_;
        }


        const double duration =
            (
                static_cast<int>(
                    trajectory.control_points.size()) -
                3
            ) *
            trajectory.dt;


        if (duration <= 0.0)
        {
            return false;
        }


        const double sample_dt =
            std::max(
                0.01,
                trajectory_sample_dt_);


        std::vector<
            geometry_msgs::msg::Point>
            samples;


        for (double t = 0.0;
             t < duration;
             t += sample_dt)
        {
            samples.push_back(
                evaluateBSpline(
                    trajectory,
                    t));
        }


        samples.push_back(
            evaluateBSpline(
                trajectory,
                duration));


        if (samples.size() < 2)
        {
            return false;
        }


        // -----------------------------------------------------
        // 找到离飞机当前位置最近的轨迹点。
        // -----------------------------------------------------
        std::size_t closest_index =
            0;

        double closest_distance =
            std::numeric_limits<double>::
                infinity();


        for (std::size_t i = 0;
             i < samples.size();
             ++i)
        {
            const double d =
                distance(
                    current,
                    samples[i]);

            if (d <
                closest_distance)
            {
                closest_distance =
                    d;

                closest_index =
                    i;
            }
        }


        // -----------------------------------------------------
        // 从closest_index向前检查
        // -----------------------------------------------------
        double checked_distance =
            0.0;


        // 先检查closest点本身
        if (pointCollision(
                map,
                samples[closest_index]))
        {
            collision_point =
                samples[closest_index];

            return true;
        }


        for (std::size_t i =
                 closest_index;
             i + 1 <
                 samples.size();
             ++i)
        {
            if (segmentCollision(
                    map,
                    samples[i],
                    samples[i + 1],
                    collision_point))
            {
                return true;
            }


            checked_distance +=
                distance(
                    samples[i],
                    samples[i + 1]);


            if (checked_distance >=
                lookahead_distance_)
            {
                break;
            }
        }


        return false;
    }


    // =========================================================
    // Action执行
    // =========================================================
    void execute(
        const std::shared_ptr<
            GoalHandleNavigate> goal_handle)
    {
        const double action_start_time =
            now().seconds();


        const auto goal_msg =
            goal_handle->get_goal();


        geometry_msgs::msg::Point target;

        target.x =
            goal_msg->
                goal_pose.pose.position.x;

        target.y =
            goal_msg->
                goal_pose.pose.position.y;

        target.z =
            goal_msg->
                goal_pose.pose.position.z;


        const auto actual_start =
            getCurrentPosition();


        const double initial_distance =
            distance(
                actual_start,
                target);


        // =====================================================
        // 已经在目标附近
        // =====================================================
        if (initial_distance <=
            goal_tolerance_)
        {
            auto result =
                std::make_shared<
                    NavigateToGoal::Result>();

            result->success =
                true;

            result->message =
                "已经位于目标附近";

            result->total_time =
                0.0f;


            publishFeedback(
                goal_handle,
                "SUCCEEDED",
                target,
                initial_distance);


            goal_handle->succeed(
                result);


            goal_active_ =
                false;

            return;
        }


        // =====================================================
        // 初始规划起点
        // =====================================================
        geometry_msgs::msg::Point planning_start =
            actual_start;


        if (planning_start.z <
            ground_z_threshold_)
        {
            planning_start.z =
                takeoff_height_;

            RCLCPP_INFO(
                get_logger(),
                "地面起飞：A*起点高度 %.2f -> %.2f m",
                actual_start.z,
                planning_start.z);
        }


        // =====================================================
        // 第一次规划
        // =====================================================
        std::string error_message;


        if (!planAndWaitTrajectory(
                planning_start,
                target,
                goal_handle,
                initial_distance,
                false,
                error_message))
        {
            abortGoal(
                goal_handle,
                error_message,
                action_start_time);

            return;
        }


        RCLCPP_INFO(
            get_logger(),
            "初始轨迹已下发，进入EXECUTING");


        // =====================================================
        // 飞行监控 + 在线重规划
        // =====================================================
        const auto execution_begin =
            std::chrono::steady_clock::now();


        const double feedback_period =
            1.0 /
            std::max(
                1.0,
                feedback_rate_);


        const double safety_period =
            1.0 /
            std::max(
                1.0,
                safety_check_rate_);


        auto last_safety_check =
            std::chrono::steady_clock::now();


        // 允许第一次立即重规划
        auto last_replan_time =
            std::chrono::steady_clock::now() -
            std::chrono::duration<double>(
                replan_cooldown_);


        int replan_count =
            0;


        bool inside_goal =
            false;


        std::chrono::steady_clock::time_point
            inside_goal_since;


        while (rclcpp::ok())
        {
            const auto current =
                getCurrentPosition();


            const double dist =
                distance(
                    current,
                    target);


            publishFeedback(
                goal_handle,
                "EXECUTING",
                target,
                initial_distance);


            // =================================================
            // 1. 到达目标
            // =================================================
            if (dist <=
                goal_tolerance_)
            {
                if (!inside_goal)
                {
                    inside_goal =
                        true;

                    inside_goal_since =
                        std::chrono::
                            steady_clock::now();
                }


                const double hold_time =
                    std::chrono::duration<double>(
                        std::chrono::
                            steady_clock::now() -
                        inside_goal_since)
                        .count();


                if (hold_time >=
                    goal_hold_time_)
                {
                    auto result =
                        std::make_shared<
                            NavigateToGoal::Result>();

                    result->success =
                        true;

                    result->message =
                        "目标到达，重规划次数=" +
                        std::to_string(
                            replan_count);

                    result->total_time =
                        static_cast<float>(
                            std::max(
                                0.0,
                                now().seconds() -
                                action_start_time));


                    publishFeedback(
                        goal_handle,
                        "SUCCEEDED",
                        target,
                        initial_distance);


                    goal_handle->succeed(
                        result);


                    goal_active_ =
                        false;


                    RCLCPP_INFO(
                        get_logger(),
                        "导航成功，replan=%d",
                        replan_count);

                    return;
                }
            }
            else
            {
                inside_goal =
                    false;
            }


            // =================================================
            // 2. 在线轨迹碰撞检测
            // =================================================
            const auto now_steady =
                std::chrono::steady_clock::now();


            const double since_safety_check =
                std::chrono::duration<double>(
                    now_steady -
                    last_safety_check)
                    .count();


            const double since_replan =
                std::chrono::duration<double>(
                    now_steady -
                    last_replan_time)
                    .count();


            if (enable_replan_ &&
                since_safety_check >=
                    safety_period &&
                since_replan >=
                    replan_cooldown_)
            {
                last_safety_check =
                    now_steady;


                geometry_msgs::msg::Point
                    collision_point;


                if (trajectoryCollisionAhead(
                        current,
                        collision_point))
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "检测到未来轨迹碰撞："
                        "(%.2f, %.2f, %.2f)，准备重规划",
                        collision_point.x,
                        collision_point.y,
                        collision_point.z);


                    if (replan_count >=
                        max_replans_)
                    {
                        abortGoal(
                            goal_handle,
                            "超过最大重规划次数",
                            action_start_time);

                        return;
                    }


                    publishFeedback(
                        goal_handle,
                        "REPLANNING",
                        target,
                        initial_distance);


                    // -----------------------------------------
                    // 重规划起点直接使用当前无人机实际位置。
                    //
                    // 当前controller收到新B样条后，
                    // 如果仍处于armed+offboard，
                    // 会自动切换到MOVE_TO_START，
                    // 再执行新轨迹。
                    // -----------------------------------------
                    const auto replan_start =
                        getCurrentPosition();


                    std::string replan_error;


                    if (!planAndWaitTrajectory(
                            replan_start,
                            target,
                            goal_handle,
                            initial_distance,
                            true,
                            replan_error))
                    {
                        abortGoal(
                            goal_handle,
                            replan_error,
                            action_start_time);

                        return;
                    }


                    ++replan_count;


                    last_replan_time =
                        std::chrono::
                            steady_clock::now();


                    inside_goal =
                        false;


                    RCLCPP_INFO(
                        get_logger(),
                        "第%d次重规划成功，继续执行新轨迹",
                        replan_count);


                    // 当前循环不要继续使用旧检查状态
                    continue;
                }
            }


            // =================================================
            // 3. 总执行超时
            // =================================================
            const double execution_elapsed =
                std::chrono::duration<double>(
                    std::chrono::
                        steady_clock::now() -
                    execution_begin)
                    .count();


            if (execution_elapsed >
                execution_timeout_)
            {
                abortGoal(
                    goal_handle,
                    "导航执行超时",
                    action_start_time);

                return;
            }


            std::this_thread::sleep_for(
                std::chrono::duration<double>(
                    feedback_period));
        }


        abortGoal(
            goal_handle,
            "ROS已退出",
            action_start_time);
    }


private:
    // =========================================================
    // Parameters
    // =========================================================
    std::string action_name_;
    std::string astar_service_name_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string bspline_topic_;

    double goal_tolerance_{0.30};
    double goal_hold_time_{0.50};

    double ground_z_threshold_{0.50};
    double takeoff_height_{2.50};

    bool enable_replan_{true};
    double safety_check_rate_{10.0};
    double lookahead_distance_{4.0};
    double trajectory_sample_dt_{0.05};

    bool unknown_is_collision_{false};
    bool outside_map_is_collision_{true};

    double replan_cooldown_{1.0};
    int max_replans_{10};

    double service_wait_timeout_{3.0};
    double planning_timeout_{10.0};
    double optimizer_timeout_{5.0};
    double execution_timeout_{120.0};
    double feedback_rate_{10.0};


    // =========================================================
    // Vehicle state
    // =========================================================
    std::mutex state_mutex_;

    bool odom_received_{false};

    geometry_msgs::msg::Point
        current_position_;


    // =========================================================
    // Map
    // =========================================================
    std::mutex map_mutex_;

    bool map_received_{false};

    uav_mapping::msg::VoxelMap
        latest_map_;


    // =========================================================
    // Trajectory
    // =========================================================
    std::mutex trajectory_mutex_;

    std::condition_variable
        trajectory_cv_;

    bool latest_trajectory_valid_{false};

    uint64_t trajectory_generation_{0};

    trajectory_optimizer::msg::
        BSplineTrajectory
        latest_trajectory_;


    // =========================================================
    // Action
    // =========================================================
    std::atomic_bool
        goal_active_{false};


    // =========================================================
    // ROS interfaces
    // =========================================================
    rclcpp::Client<
        PlanPath>::SharedPtr
        astar_client_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::
        SharedPtr odom_sub_;

    rclcpp::Subscription<
        uav_mapping::msg::VoxelMap>::
        SharedPtr map_sub_;

    rclcpp::Subscription<
        trajectory_optimizer::msg::
            BSplineTrajectory>::
        SharedPtr bspline_sub_;

    rclcpp_action::Server<
        NavigateToGoal>::SharedPtr
        action_server_;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);

    rclcpp::spin(
        std::make_shared<
            PlannerManagerNode>());

    rclcpp::shutdown();

    return 0;
}
