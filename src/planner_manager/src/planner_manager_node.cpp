#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <astar_planner/srv/plan_path.hpp>
#include <planner_manager_interfaces/action/navigate_to_goal.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <trajectory_optimizer/msg/b_spline_trajectory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;


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
        // 1. 参数
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

        bspline_topic_ =
            declare_parameter<std::string>(
                "bspline_topic",
                "/trajectory_optimizer/bspline");


        // 到达目标判定
        goal_tolerance_ =
            declare_parameter<double>(
                "goal_tolerance",
                0.30);

        // 必须连续在目标范围内一段时间，
        // 避免仅仅从目标附近快速经过就判定成功。
        goal_hold_time_ =
            declare_parameter<double>(
                "goal_hold_time",
                0.50);


        // 初始在地面时，不直接以 z≈0 作为 A* 起点。
        //
        // 你的 px4_controller 本身会先飞到 B 样条起点，
        // 因此这里把首次规划起点抬到 takeoff_height。
        ground_z_threshold_ =
            declare_parameter<double>(
                "ground_z_threshold",
                0.50);

        takeoff_height_ =
            declare_parameter<double>(
                "takeoff_height",
                2.50);


        // 超时参数
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
        // 2. A* Service Client
        // =====================================================
        astar_client_ =
            create_client<PlanPath>(
                astar_service_name_);


        // =====================================================
        // 3. PX4 Odometry
        //
        // PX4输出使用SensorDataQoS。
        // 在这里转换 NED -> ENU/map。
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
        // 4. 监听优化器输出的 B 样条
        //
        // 当前你的工程是：
        //
        // A* service成功
        //      ↓
        // astar_node发布 /astar/path
        //      ↓
        // trajectory_optimizer自动优化
        //      ↓
        // 发布 /trajectory_optimizer/bspline
        //      ↓
        // px4_controller自动执行
        //
        // 所以 manager 不需要再修改现有 optimizer/controller。
        //
        // manager只需要确认：
        // “本次规划以后，确实产生了一条新的B样条。”
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
        // 5. NavigateToGoal Action Server
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
            "Action: %s",
            action_name_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "A* Service: %s",
            astar_service_name_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "等待PX4里程计: %s",
            odom_topic_.c_str());
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

        // ENU:
        // x = East  = NED y
        // y = North = NED x
        // z = Up    = -NED z
        current_position_.x =
            msg->position[1];

        current_position_.y =
            msg->position[0];

        current_position_.z =
            -msg->position[2];

        odom_received_ = true;
    }


    // =========================================================
    // 收到一条新的优化B样条
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

            ++trajectory_generation_;

            latest_trajectory_duration_ =
                msg->duration;
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
                "拒绝Action目标：目标坐标非法");

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
                "拒绝Action目标：目前只支持map/ENU坐标系，收到frame=%s",
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
                    "拒绝Action目标：尚未收到PX4 VehicleOdometry");

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
                "拒绝Action目标：已有导航任务正在执行");

            return
                rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(
            get_logger(),
            "接受导航目标: (%.2f, %.2f, %.2f)",
            p.x,
            p.y,
            p.z);

        return
            rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    // =========================================================
    // Cancel
    //
    // 第一版暂时拒绝cancel。
    //
    // 原因：
    // 当前px4_controller收到B样条后会独立继续执行，
    // manager自身停止Action并不能真正让飞机停止。
    //
    // 下一阶段加入“hold/cancel controller”接口后，
    // 再把这里改成ACCEPT。
    // =========================================================
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<
            GoalHandleNavigate>)
    {
        RCLCPP_WARN(
            get_logger(),
            "当前第一版暂不支持安全取消导航任务");

        return
            rclcpp_action::CancelResponse::REJECT;
    }


    // =========================================================
    // Goal accepted
    //
    // Action执行放到独立线程中，避免阻塞ROS回调。
    // =========================================================
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

        return current_position_;
    }


    // =========================================================
    // 距离
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

        return std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz);
    }


    // =========================================================
    // Feedback
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
            static_cast<float>(dist);

        feedback->progress =
            static_cast<float>(progress);

        goal_handle->publish_feedback(
            feedback);
    }


    // =========================================================
    // Abort helper
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

        result->success = false;
        result->message = message;

        result->total_time =
            static_cast<float>(
                std::max(
                    0.0,
                    now().seconds() -
                    start_time));

        goal_handle->abort(result);

        goal_active_ = false;

        RCLCPP_ERROR(
            get_logger(),
            "导航失败: %s",
            message.c_str());
    }


    // =========================================================
    // Action执行主过程
    // =========================================================
    void execute(
        const std::shared_ptr<
            GoalHandleNavigate> goal_handle)
    {
        const double action_start_time =
            now().seconds();

        const auto goal =
            goal_handle->get_goal();

        geometry_msgs::msg::Point target;

        target.x =
            goal->goal_pose.pose.position.x;

        target.y =
            goal->goal_pose.pose.position.y;

        target.z =
            goal->goal_pose.pose.position.z;


        // =====================================================
        // 1. 当前实际位置
        // =====================================================
        const auto actual_start =
            getCurrentPosition();

        const double initial_distance =
            distance(
                actual_start,
                target);


        // 已经在目标附近
        if (initial_distance <=
            goal_tolerance_)
        {
            auto result =
                std::make_shared<
                    NavigateToGoal::Result>();

            result->success = true;
            result->message =
                "已经位于目标附近";

            result->total_time = 0.0f;

            publishFeedback(
                goal_handle,
                "SUCCEEDED",
                target,
                initial_distance);

            goal_handle->succeed(result);

            goal_active_ = false;

            return;
        }


        // =====================================================
        // 2. 构造A*规划起点
        //
        // 无人机还在地面时：
        //
        // actual_start.z ≈ 0
        //
        // 如果直接从地面做3D A*，很容易与地面膨胀障碍冲突。
        //
        // 因此首次规划使用：
        // (current_x, current_y, takeoff_height)
        //
        // px4_controller随后会自动先飞到这一B样条起点。
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
                "当前无人机位于地面附近，"
                "A*起点高度由 %.2f m 调整为 %.2f m",
                actual_start.z,
                planning_start.z);
        }


        // =====================================================
        // 3. 记录当前B样条版本号
        //
        // 后面必须等 generation 变大，
        // 才能确认是本次A*触发的新轨迹，
        // 而不是transient_local保存的旧轨迹。
        // =====================================================
        uint64_t trajectory_generation_before;

        {
            std::lock_guard<std::mutex>
                lock(trajectory_mutex_);

            trajectory_generation_before =
                trajectory_generation_;
        }


        // =====================================================
        // 4. PLANNING：调用现有 /astar/plan
        // =====================================================
        publishFeedback(
            goal_handle,
            "PLANNING",
            target,
            initial_distance);

        if (!astar_client_->wait_for_service(
                std::chrono::duration<double>(
                    service_wait_timeout_)))
        {
            abortGoal(
                goal_handle,
                "等待 /astar/plan Service 超时",
                action_start_time);

            return;
        }

        auto request =
            std::make_shared<
                PlanPath::Request>();

        request->start =
            planning_start;

        request->goal =
            target;

        RCLCPP_INFO(
            get_logger(),
            "调用A*: start=(%.2f %.2f %.2f), goal=(%.2f %.2f %.2f)",
            planning_start.x,
            planning_start.y,
            planning_start.z,
            target.x,
            target.y,
            target.z);

        auto future =
            astar_client_->
                async_send_request(
                    request);

        if (future.wait_for(
                std::chrono::duration<double>(
                    planning_timeout_)) !=
            std::future_status::ready)
        {
            abortGoal(
                goal_handle,
                "A*规划超时",
                action_start_time);

            return;
        }

        const auto response =
            future.get();

        if (!response->success)
        {
            abortGoal(
                goal_handle,
                "A*失败: " +
                    response->message,
                action_start_time);

            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "A*成功，路径点数=%zu",
            response->path.poses.size());


        // =====================================================
        // 5. OPTIMIZING
        //
        // 注意：
        // 你的astar_node在Service成功时已经会发布 /astar/path。
        //
        // 所以这里“不需要重新publish Path”。
        //
        // trajectory_optimizer会自动收到 /astar/path，
        // 完成QP并发布新的 /trajectory_optimizer/bspline。
        // =====================================================
        publishFeedback(
            goal_handle,
            "OPTIMIZING",
            target,
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
                            trajectory_generation_before;
                    });

            if (!got_new_trajectory)
            {
                lock.unlock();

                abortGoal(
                    goal_handle,
                    "轨迹优化超时："
                    "A*已成功，但没有收到新的B样条",
                    action_start_time);

                return;
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "收到本次规划对应的新B样条，"
            "px4_controller将自动开始执行");


        // =====================================================
        // 6. EXECUTING
        //
        // manager直接监视PX4实际位置。
        //
        // 当前controller已经能：
        // - 自动Offboard
        // - 自动Arm
        // - 先到B样条起点
        // - 执行轨迹
        // - 最后悬停
        //
        // 所以这里不重复写控制逻辑。
        // =====================================================
        const auto execution_begin =
            std::chrono::steady_clock::now();

        const double feedback_period =
            1.0 /
            std::max(
                1.0,
                feedback_rate_);

        bool inside_goal = false;

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


            // ---------------------------------------------
            // 到达目标判定
            // ---------------------------------------------
            if (dist <=
                goal_tolerance_)
            {
                if (!inside_goal)
                {
                    inside_goal = true;

                    inside_goal_since =
                        std::chrono::steady_clock::now();
                }

                const double hold_time =
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() -
                        inside_goal_since)
                        .count();

                if (hold_time >=
                    goal_hold_time_)
                {
                    auto result =
                        std::make_shared<
                            NavigateToGoal::Result>();

                    result->success = true;

                    result->message =
                        "目标到达";

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

                    goal_active_ = false;

                    RCLCPP_INFO(
                        get_logger(),
                        "导航成功，目标=(%.2f %.2f %.2f)",
                        target.x,
                        target.y,
                        target.z);

                    return;
                }
            }
            else
            {
                inside_goal = false;
            }


            // ---------------------------------------------
            // 执行超时
            // ---------------------------------------------
            const double execution_elapsed =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() -
                    execution_begin)
                    .count();

            if (execution_elapsed >
                execution_timeout_)
            {
                abortGoal(
                    goal_handle,
                    "轨迹执行超时",
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
    std::string bspline_topic_;

    double goal_tolerance_{0.30};
    double goal_hold_time_{0.50};

    double ground_z_threshold_{0.50};
    double takeoff_height_{2.50};

    double service_wait_timeout_{3.0};
    double planning_timeout_{10.0};
    double optimizer_timeout_{5.0};
    double execution_timeout_{120.0};
    double feedback_rate_{10.0};


    // =========================================================
    // Current vehicle state
    // =========================================================
    std::mutex state_mutex_;

    bool odom_received_{false};

    geometry_msgs::msg::Point
        current_position_;


    // =========================================================
    // Trajectory generation monitor
    // =========================================================
    std::mutex trajectory_mutex_;

    std::condition_variable
        trajectory_cv_;

    uint64_t trajectory_generation_{0};

    double latest_trajectory_duration_{0.0};


    // =========================================================
    // One active navigation goal
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
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            PlannerManagerNode>());

    rclcpp::shutdown();

    return 0;
}
