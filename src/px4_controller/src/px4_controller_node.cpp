#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <trajectory_optimizer/msg/b_spline_trajectory.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>


using namespace std::chrono_literals;


class Px4ControllerNode : public rclcpp::Node
{
public:
    Px4ControllerNode()
        : Node("px4_controller")
    {
        // =====================================================
        // 1. 参数
        // =====================================================
        trajectory_topic_ =
            declare_parameter<std::string>(
                "trajectory_topic",
                "/trajectory_optimizer/bspline");

        control_rate_ =
            declare_parameter<double>(
                "control_rate",
                50.0);

        // 进入 Offboard 前预发送 setpoint 的时间
        prestream_duration_ =
            declare_parameter<double>(
                "prestream_duration",
                1.0);

        // 等待 PX4 进入 armed + offboard 的最大时间
        mode_timeout_ =
            declare_parameter<double>(
                "mode_timeout",
                5.0);

        // 飞到 B 样条起点后，距离小于该值才正式计时执行
        start_position_tolerance_ =
            declare_parameter<double>(
                "start_position_tolerance",
                0.25);

        // 初始轨迹的水平起点不能离飞机太远。
        // 允许 z 方向不同，因此可以由地面自动爬升到轨迹起点高度。
        max_start_xy_error_ =
            declare_parameter<double>(
                "max_start_xy_error",
                1.0);

        // 轨迹完成后一直悬停在终点，不自动降落/解锁
        hold_end_ =
            declare_parameter<bool>(
                "hold_end",
                true);


        // =====================================================
        // 2. PX4 publishers
        // =====================================================
        offboard_mode_pub_ =
            create_publisher<
                px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode",
                10);

        trajectory_setpoint_pub_ =
            create_publisher<
                px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint",
                10);

        vehicle_command_pub_ =
            create_publisher<
                px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command",
                10);


        // =====================================================
        // 3. PX4 subscribers
        //
        // PX4 -> ROS2 通常使用 best-effort，
        // 因此这里使用 SensorDataQoS。
        // =====================================================
        odom_sub_ =
            create_subscription<
                px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &Px4ControllerNode::odomCallback,
                    this,
                    std::placeholders::_1));

        status_sub_ =
            create_subscription<
                px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &Px4ControllerNode::statusCallback,
                    this,
                    std::placeholders::_1));


        // =====================================================
        // 4. B 样条轨迹 subscriber
        //
        // trajectory_optimizer 发布 transient_local，
        // 所以后启动 controller 也可以收到最后一条轨迹。
        // =====================================================
        auto trajectory_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        trajectory_sub_ =
            create_subscription<
                trajectory_optimizer::msg::
                    BSplineTrajectory>(
                trajectory_topic_,
                trajectory_qos,
                std::bind(
                    &Px4ControllerNode::
                        trajectoryCallback,
                    this,
                    std::placeholders::_1));


        // =====================================================
        // 5. 控制定时器
        // =====================================================
        const double safe_rate =
            std::max(
                10.0,
                control_rate_);

        timer_ =
            create_wall_timer(
                std::chrono::duration<double>(
                    1.0 / safe_rate),
                std::bind(
                    &Px4ControllerNode::
                        controlLoop,
                    this));


        RCLCPP_INFO(
            get_logger(),
            "px4_controller启动，控制频率=%.1f Hz",
            safe_rate);

        RCLCPP_INFO(
            get_logger(),
            "等待B样条: %s",
            trajectory_topic_.c_str());
    }


private:
    // =========================================================
    // 状态机
    // =========================================================
    enum class State
    {
        WAIT_TRAJECTORY,

        // 进入 Offboard 前先连续发送 heartbeat + 起点 setpoint
        PRESTREAM,

        // 已发送 Offboard/Arm 命令，等待 PX4 状态确认
        WAIT_OFFBOARD,

        // 先飞到 B 样条起点
        MOVE_TO_START,

        // 正式按时间执行 B 样条
        TRACKING,

        // 轨迹结束后悬停在终点
        HOLD_END,

        ERROR
    };


    // =========================================================
    // 简单轨迹状态
    // =========================================================
    struct TrajectoryState
    {
        Eigen::Vector3d position{
            Eigen::Vector3d::Zero()};

        Eigen::Vector3d velocity{
            Eigen::Vector3d::Zero()};

        Eigen::Vector3d acceleration{
            Eigen::Vector3d::Zero()};

        Eigen::Vector3d jerk{
            Eigen::Vector3d::Zero()};
    };


    // =========================================================
    // 时间：ROS/Gazebo sim time -> 微秒
    // =========================================================
    uint64_t nowUs() const
    {
        return static_cast<uint64_t>(
            get_clock()->now().nanoseconds() /
            1000);
    }


    double nowSec() const
    {
        return
            get_clock()->now().seconds();
    }


    // =========================================================
    // PX4 odometry
    // =========================================================
    void odomCallback(
        const px4_msgs::msg::VehicleOdometry::
            SharedPtr msg)
    {
        if (msg->pose_frame !=
            px4_msgs::msg::VehicleOdometry::
                POSE_FRAME_NED)
        {
            return;
        }

        if (!std::isfinite(msg->position[0]) ||
            !std::isfinite(msg->position[1]) ||
            !std::isfinite(msg->position[2]))
        {
            return;
        }

        // PX4 NED -> ROS/map ENU
        //
        // x_enu = y_ned
        // y_enu = x_ned
        // z_enu = -z_ned
        current_position_enu_ =
            Eigen::Vector3d(
                msg->position[1],
                msg->position[0],
                -msg->position[2]);

        odom_received_ = true;
    }


    // =========================================================
    // PX4 vehicle status
    // =========================================================
    void statusCallback(
        const px4_msgs::msg::VehicleStatus::
            SharedPtr msg)
    {
        armed_ =
            msg->arming_state ==
            px4_msgs::msg::VehicleStatus::
                ARMING_STATE_ARMED;

        offboard_ =
            msg->nav_state ==
            px4_msgs::msg::VehicleStatus::
                NAVIGATION_STATE_OFFBOARD;

        vehicle_status_received_ = true;
    }


    // =========================================================
    // 收到新的 B 样条
    // =========================================================
    void trajectoryCallback(
        const trajectory_optimizer::msg::
            BSplineTrajectory::SharedPtr msg)
    {
        if (msg->degree != 3)
        {
            RCLCPP_WARN(
                get_logger(),
                "当前controller只支持三次B样条，degree=%u",
                static_cast<unsigned int>(msg->degree));
            return;
        }

        if (msg->control_points.size() < 4 ||
            msg->dt <= 0.0)
        {
            RCLCPP_WARN(
                get_logger(),
                "收到无效B样条");
            return;
        }

        std::vector<Eigen::Vector3d>
            new_control_points;

        new_control_points.reserve(
            msg->control_points.size());

        for (const auto &p :
             msg->control_points)
        {
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "B样条存在非法控制点");
                return;
            }

            new_control_points.emplace_back(
                p.x,
                p.y,
                p.z);
        }

        const double new_duration =
            (
                static_cast<int>(
                    new_control_points.size()) -
                3
            ) *
            msg->dt;

        // 起点
        const TrajectoryState start_state =
            evaluateTrajectory(
                new_control_points,
                msg->dt,
                0.0);

        // 如果已经有里程计，则限制水平方向起点距离。
        // z 不限制，以便初始时从地面爬升到路径起点高度。
        if (odom_received_)
        {
            const Eigen::Vector2d diff_xy =
                (
                    start_state.position -
                    current_position_enu_
                ).head<2>();

            if (diff_xy.norm() >
                max_start_xy_error_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "拒绝轨迹：轨迹起点与飞机水平距离 %.2f m "
                    "超过限制 %.2f m",
                    diff_xy.norm(),
                    max_start_xy_error_);
                return;
            }
        }

        control_points_ =
            std::move(
                new_control_points);

        spline_dt_ =
            msg->dt;

        trajectory_duration_ =
            new_duration;

        trajectory_received_ =
            true;

        // 如果飞机已经处在 armed + offboard，
        // 说明这是飞行中的重规划轨迹。
        // 直接先对齐新轨迹起点。
        if (armed_ && offboard_)
        {
            state_ =
                State::MOVE_TO_START;
        }
        else
        {
            prestream_start_time_ =
                nowSec();

            command_start_time_ =
                0.0;

            last_command_time_ =
                -1.0;

            state_ =
                State::PRESTREAM;
        }

        RCLCPP_INFO(
            get_logger(),
            "收到B样条：控制点=%zu, dt=%.3f, duration=%.3f s",
            control_points_.size(),
            spline_dt_,
            trajectory_duration_);
    }


    // =========================================================
    // B 样条求 p/v/a/jerk
    // =========================================================
    static TrajectoryState evaluateTrajectory(
        const std::vector<Eigen::Vector3d> &cp,
        double dt,
        double t)
    {
        TrajectoryState state;

        if (cp.size() < 4 ||
            dt <= 0.0)
        {
            return state;
        }

        const int segment_num =
            static_cast<int>(
                cp.size()) -
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
                    t /
                    dt));

        double u;

        if (segment >=
            segment_num)
        {
            segment =
                segment_num -
                1;

            u = 1.0;
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

        // -----------------------------------------------------
        // 三次均匀 B 样条位置 basis
        // -----------------------------------------------------
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

        // -----------------------------------------------------
        // 对 u 的一阶导
        // v = dp/du * du/dt
        // -----------------------------------------------------
        const double dB0 =
            (-3.0 +
             6.0 * u -
             3.0 * u2) /
            6.0;

        const double dB1 =
            (-12.0 * u +
             9.0 * u2) /
            6.0;

        const double dB2 =
            (3.0 +
             6.0 * u -
             9.0 * u2) /
            6.0;

        const double dB3 =
            3.0 * u2 /
            6.0;

        // -----------------------------------------------------
        // 对 u 的二阶导
        // -----------------------------------------------------
        const double ddB0 =
            1.0 - u;

        const double ddB1 =
            -2.0 +
            3.0 * u;

        const double ddB2 =
            1.0 -
            3.0 * u;

        const double ddB3 =
            u;

        // -----------------------------------------------------
        // 对 u 的三阶导
        // -----------------------------------------------------
        const double dddB0 =
            -1.0;

        const double dddB1 =
            3.0;

        const double dddB2 =
            -3.0;

        const double dddB3 =
            1.0;

        const auto &Q0 =
            cp[segment];

        const auto &Q1 =
            cp[segment + 1];

        const auto &Q2 =
            cp[segment + 2];

        const auto &Q3 =
            cp[segment + 3];

        state.position =
            B0 * Q0 +
            B1 * Q1 +
            B2 * Q2 +
            B3 * Q3;

        state.velocity =
            (
                dB0 * Q0 +
                dB1 * Q1 +
                dB2 * Q2 +
                dB3 * Q3
            ) /
            dt;

        state.acceleration =
            (
                ddB0 * Q0 +
                ddB1 * Q1 +
                ddB2 * Q2 +
                ddB3 * Q3
            ) /
            (dt * dt);

        state.jerk =
            (
                dddB0 * Q0 +
                dddB1 * Q1 +
                dddB2 * Q2 +
                dddB3 * Q3
            ) /
            (dt * dt * dt);

        return state;
    }


    // =========================================================
    // ENU -> NED
    // =========================================================
    static Eigen::Vector3d enuToNed(
        const Eigen::Vector3d &v)
    {
        return Eigen::Vector3d(
            v.y(),
            v.x(),
            -v.z());
    }


    // =========================================================
    // Offboard heartbeat
    //
    // position=true：
    // 使用 PX4 位置控制器。
    //
    // TrajectorySetpoint 中非 NaN 的 velocity/acceleration
    // 会作为位置控制的前馈项。
    // =========================================================
    void publishOffboardControlMode()
    {
        px4_msgs::msg::
            OffboardControlMode msg{};

        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;

        msg.timestamp =
            nowUs();

        offboard_mode_pub_->
            publish(msg);
    }


    // =========================================================
    // 发布轨迹状态到 PX4
    // =========================================================
    void publishSetpoint(
        const TrajectoryState &state,
        bool use_feedforward)
    {
        const Eigen::Vector3d p_ned =
            enuToNed(
                state.position);

        Eigen::Vector3d v_ned =
            Eigen::Vector3d::Zero();

        Eigen::Vector3d a_ned =
            Eigen::Vector3d::Zero();

        Eigen::Vector3d j_ned =
            Eigen::Vector3d::Zero();

        if (use_feedforward)
        {
            v_ned =
                enuToNed(
                    state.velocity);

            a_ned =
                enuToNed(
                    state.acceleration);

            j_ned =
                enuToNed(
                    state.jerk);
        }

        px4_msgs::msg::
            TrajectorySetpoint msg{};

        msg.position = {
            static_cast<float>(
                p_ned.x()),
            static_cast<float>(
                p_ned.y()),
            static_cast<float>(
                p_ned.z())
        };

        msg.velocity = {
            static_cast<float>(
                v_ned.x()),
            static_cast<float>(
                v_ned.y()),
            static_cast<float>(
                v_ned.z())
        };

        msg.acceleration = {
            static_cast<float>(
                a_ned.x()),
            static_cast<float>(
                a_ned.y()),
            static_cast<float>(
                a_ned.z())
        };

        // PX4 当前将 jerk 字段用于日志；
        // 仍发布真实值，方便后续分析。
        msg.jerk = {
            static_cast<float>(
                j_ned.x()),
            static_cast<float>(
                j_ned.y()),
            static_cast<float>(
                j_ned.z())
        };

        // 第一版暂不控制机头朝向。
        // NaN 表示该状态不由此 setpoint 控制。
        const float nan =
            std::numeric_limits<float>::
                quiet_NaN();

        msg.yaw = nan;
        msg.yawspeed = nan;

        msg.timestamp =
            nowUs();

        trajectory_setpoint_pub_->
            publish(msg);
    }


    // =========================================================
    // PX4 VehicleCommand
    // =========================================================
    void publishVehicleCommand(
        uint32_t command,
        float param1 = 0.0f,
        float param2 = 0.0f)
    {
        px4_msgs::msg::
            VehicleCommand msg{};

        msg.param1 =
            param1;

        msg.param2 =
            param2;

        msg.command =
            command;

        msg.target_system =
            1;

        msg.target_component =
            1;

        msg.source_system =
            1;

        msg.source_component =
            1;

        msg.from_external =
            true;

        msg.timestamp =
            nowUs();

        vehicle_command_pub_->
            publish(msg);
    }


    void requestOffboardAndArm()
    {
        // PX4 官方 ROS2 Offboard 示例：
        // param1=1, param2=6 -> Offboard mode
        publishVehicleCommand(
            px4_msgs::msg::
                VehicleCommand::
                VEHICLE_CMD_DO_SET_MODE,
            1.0f,
            6.0f);

        // Arm
        publishVehicleCommand(
            px4_msgs::msg::
                VehicleCommand::
                VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f,
            0.0f);
    }


    // =========================================================
    // 当前状态与目标点距离
    // =========================================================
    double distanceTo(
        const Eigen::Vector3d &p) const
    {
        if (!odom_received_)
        {
            return
                std::numeric_limits<double>::
                    infinity();
        }

        return
            (
                current_position_enu_ -
                p
            ).norm();
    }


    // =========================================================
    // 主控制循环
    // =========================================================
    void controlLoop()
    {
        if (!trajectory_received_)
        {
            state_ =
                State::WAIT_TRAJECTORY;

            return;
        }

        const TrajectoryState start_state =
            evaluateTrajectory(
                control_points_,
                spline_dt_,
                0.0);

        const TrajectoryState end_state =
            evaluateTrajectory(
                control_points_,
                spline_dt_,
                trajectory_duration_);

        switch (state_)
        {
            // -------------------------------------------------
            // 等待轨迹
            // -------------------------------------------------
            case State::WAIT_TRAJECTORY:
            {
                break;
            }


            // -------------------------------------------------
            // 进入 Offboard 前：
            // 连续发送 heartbeat + 起点 setpoint
            // -------------------------------------------------
            case State::PRESTREAM:
            {
                publishOffboardControlMode();

                // 起点使用位置控制，不使用 v/a 前馈
                publishSetpoint(
                    start_state,
                    false);

                if (nowSec() -
                        prestream_start_time_ >=
                    prestream_duration_)
                {
                    requestOffboardAndArm();

                    command_start_time_ =
                        nowSec();

                    last_command_time_ =
                        nowSec();

                    state_ =
                        State::WAIT_OFFBOARD;

                    RCLCPP_INFO(
                        get_logger(),
                        "已请求Offboard + Arm");
                }

                break;
            }


            // -------------------------------------------------
            // 等待 PX4 真正进入 armed + offboard
            // -------------------------------------------------
            case State::WAIT_OFFBOARD:
            {
                publishOffboardControlMode();

                publishSetpoint(
                    start_state,
                    false);

                if (armed_ &&
                    offboard_)
                {
                    state_ =
                        State::MOVE_TO_START;

                    RCLCPP_INFO(
                        get_logger(),
                        "PX4已进入Offboard并解锁，"
                        "开始移动到轨迹起点");

                    break;
                }

                // 每 1 秒重发一次命令
                if (nowSec() -
                        last_command_time_ >
                    1.0)
                {
                    requestOffboardAndArm();

                    last_command_time_ =
                        nowSec();
                }

                if (nowSec() -
                        command_start_time_ >
                    mode_timeout_)
                {
                    state_ =
                        State::ERROR;

                    RCLCPP_ERROR(
                        get_logger(),
                        "进入Offboard/Arm超时；"
                        "请检查PX4 preflight状态和topic连接");
                }

                break;
            }


            // -------------------------------------------------
            // 飞到 B 样条起点
            //
            // 这一步也承担首次从地面爬升到规划高度的作用。
            // -------------------------------------------------
            case State::MOVE_TO_START:
            {
                publishOffboardControlMode();

                publishSetpoint(
                    start_state,
                    false);

                if (!armed_ ||
                    !offboard_)
                {
                    state_ =
                        State::ERROR;

                    RCLCPP_ERROR(
                        get_logger(),
                        "移动到轨迹起点过程中离开Offboard或解锁");

                    break;
                }

                const double error =
                    distanceTo(
                        start_state.position);

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "正在到达轨迹起点，误差=%.2f m",
                    error);

                if (error <=
                    start_position_tolerance_)
                {
                    trajectory_start_time_ =
                        nowSec();

                    state_ =
                        State::TRACKING;

                    RCLCPP_INFO(
                        get_logger(),
                        "到达轨迹起点，开始执行B样条");
                }

                break;
            }


            // -------------------------------------------------
            // 正式执行 B 样条
            // -------------------------------------------------
            case State::TRACKING:
            {
                publishOffboardControlMode();

                if (!armed_ ||
                    !offboard_)
                {
                    state_ =
                        State::ERROR;

                    RCLCPP_ERROR(
                        get_logger(),
                        "轨迹执行过程中离开Offboard或解锁");

                    break;
                }

                const double t =
                    nowSec() -
                    trajectory_start_time_;

                if (t >=
                    trajectory_duration_)
                {
                    publishSetpoint(
                        end_state,
                        false);

                    state_ =
                        State::HOLD_END;

                    RCLCPP_INFO(
                        get_logger(),
                        "B样条执行完成，进入终点悬停");

                    break;
                }

                const TrajectoryState state =
                    evaluateTrajectory(
                        control_points_,
                        spline_dt_,
                        t);

                // 正式跟踪时同时发送 p/v/a 前馈
                publishSetpoint(
                    state,
                    true);

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "轨迹执行: %.1f / %.1f s",
                    t,
                    trajectory_duration_);

                break;
            }


            // -------------------------------------------------
            // 终点悬停
            // -------------------------------------------------
            case State::HOLD_END:
            {
                if (!hold_end_)
                    break;

                publishOffboardControlMode();

                // 终点位置 + 零速度/零加速度
                publishSetpoint(
                    end_state,
                    false);

                break;
            }


            // -------------------------------------------------
            // ERROR：
            // 不再继续发布外部 setpoint。
            // PX4 将按自己的 Offboard-loss failsafe 处理。
            // -------------------------------------------------
            case State::ERROR:
            {
                break;
            }
        }
    }


private:
    // =========================================================
    // Parameters
    // =========================================================
    std::string trajectory_topic_;

    double control_rate_{50.0};
    double prestream_duration_{1.0};
    double mode_timeout_{5.0};

    double start_position_tolerance_{0.25};
    double max_start_xy_error_{1.0};

    bool hold_end_{true};


    // =========================================================
    // Trajectory
    // =========================================================
    std::vector<Eigen::Vector3d>
        control_points_;

    double spline_dt_{0.0};
    double trajectory_duration_{0.0};

    bool trajectory_received_{false};


    // =========================================================
    // PX4 state
    // =========================================================
    Eigen::Vector3d current_position_enu_{
        Eigen::Vector3d::Zero()};

    bool odom_received_{false};
    bool vehicle_status_received_{false};

    bool armed_{false};
    bool offboard_{false};


    // =========================================================
    // State machine timing
    // =========================================================
    State state_{
        State::WAIT_TRAJECTORY};

    double prestream_start_time_{0.0};
    double command_start_time_{0.0};
    double last_command_time_{-1.0};
    double trajectory_start_time_{0.0};


    // =========================================================
    // ROS interfaces
    // =========================================================
    rclcpp::Publisher<
        px4_msgs::msg::OffboardControlMode>::
        SharedPtr offboard_mode_pub_;

    rclcpp::Publisher<
        px4_msgs::msg::TrajectorySetpoint>::
        SharedPtr trajectory_setpoint_pub_;

    rclcpp::Publisher<
        px4_msgs::msg::VehicleCommand>::
        SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::
        SharedPtr odom_sub_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleStatus>::
        SharedPtr status_sub_;

    rclcpp::Subscription<
        trajectory_optimizer::msg::
            BSplineTrajectory>::
        SharedPtr trajectory_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};


// =============================================================
// main
// =============================================================
int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);

    rclcpp::spin(
        std::make_shared<
            Px4ControllerNode>());

    rclcpp::shutdown();

    return 0;
}