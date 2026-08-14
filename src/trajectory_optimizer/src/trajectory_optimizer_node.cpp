#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <uav_mapping/msg/voxel_map.hpp>

#include <trajectory_optimizer/msg/b_spline_trajectory.hpp>

#include "trajectory_optimizer/trajectory_optimizer.hpp"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>


class TrajectoryOptimizerNode
    : public rclcpp::Node
{
public:
    TrajectoryOptimizerNode()
        : Node("trajectory_optimizer")
    {
        // =====================================================
        // 参数
        // =====================================================
        trajectory_optimizer::
            OptimizerOptions options;

        options.unknown_is_free =
            declare_parameter<bool>(
                "unknown_is_free",
                true);

        options.simplify_allow_unknown =
            declare_parameter<bool>(
                "simplify_allow_unknown",
                false);

        options.control_point_spacing =
            declare_parameter<double>(
                "control_point_spacing",
                0.3);

        options.corridor_max_radius =
            declare_parameter<double>(
                "corridor_max_radius",
                1.0);

        options.corridor_margin =
            declare_parameter<double>(
                "corridor_margin",
                0.02);

        // 二阶差分平滑项
        options.weight_smooth =
            declare_parameter<double>(
                "weight_smooth",
                20.0);

        // 三阶差分 Jerk 项
        options.weight_jerk =
            declare_parameter<double>(
                "weight_jerk",
                5.0);

        // 参考路径项
        options.weight_reference =
            declare_parameter<double>(
                "weight_reference",
                2.0);

        options.initial_dt =
            declare_parameter<double>(
                "initial_dt",
                0.5);

        options.max_vel_axis =
            declare_parameter<double>(
                "max_vel_axis",
                2.0);

        options.max_acc_axis =
            declare_parameter<double>(
                "max_acc_axis",
                2.5);

        options.time_scale_factor =
            declare_parameter<double>(
                "time_scale_factor",
                1.25);

        options.max_time_scaling_attempts =
            declare_parameter<int>(
                "max_time_scaling_attempts",
                5);

        options.validation_dt =
            declare_parameter<double>(
                "validation_dt",
                0.05);

        optimizer_.setOptions(options);

        map_topic_ =
            declare_parameter<std::string>(
                "map_topic",
                "/uav_mapping/voxel_map");

        path_topic_ =
            declare_parameter<std::string>(
                "path_topic",
                "/astar/path");

        // =====================================================
        // Subscriber
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
                    &TrajectoryOptimizerNode::mapCallback,
                    this,
                    std::placeholders::_1));

        path_sub_ =
            create_subscription<
                nav_msgs::msg::Path>(
                path_topic_,
                rclcpp::QoS(1)
                    .reliable()
                    .transient_local(),
                std::bind(
                    &TrajectoryOptimizerNode::pathCallback,
                    this,
                    std::placeholders::_1));

        // =====================================================
        // Publisher
        // =====================================================
        auto output_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();

        simplified_pub_ =
            create_publisher<
                nav_msgs::msg::Path>(
                "/trajectory_optimizer/simplified_path",
                output_qos);

        control_points_pub_ =
            create_publisher<
                nav_msgs::msg::Path>(
                "/trajectory_optimizer/control_points",
                output_qos);

        optimized_path_pub_ =
            create_publisher<
                nav_msgs::msg::Path>(
                "/trajectory_optimizer/optimized_path",
                output_qos);

        corridor_pub_ =
            create_publisher<
                visualization_msgs::msg::MarkerArray>(
                "/trajectory_optimizer/corridors",
                output_qos);

        bspline_pub_ =
            create_publisher<
                trajectory_optimizer::msg::
                    BSplineTrajectory>(
                "/trajectory_optimizer/bspline",
                output_qos);

        RCLCPP_INFO(
            get_logger(),
            "trajectory_optimizer启动 "
            "(smooth=%.2f, jerk=%.2f, reference=%.2f)",
            options.weight_smooth,
            options.weight_jerk,
            options.weight_reference);
    }


private:
    void mapCallback(
        const uav_mapping::msg::VoxelMap::
            SharedPtr msg)
    {
        optimizer_.setMap(*msg);

        if (!map_received_once_ &&
            optimizer_.hasMap())
        {
            map_received_once_ = true;

            RCLCPP_INFO(
                get_logger(),
                "收到VoxelMap");
        }
    }


    void pathCallback(
        const nav_msgs::msg::Path::
            SharedPtr msg)
    {
        if (!optimizer_.hasMap())
        {
            RCLCPP_WARN(
                get_logger(),
                "尚未收到地图，暂不优化");
            return;
        }

        if (msg->poses.size() < 2)
        {
            RCLCPP_WARN(
                get_logger(),
                "A*路径点数量不足");
            return;
        }

        std::vector<Eigen::Vector3d>
            astar_path;

        astar_path.reserve(
            msg->poses.size());

        for (const auto &pose :
             msg->poses)
        {
            astar_path.emplace_back(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z);
        }

        trajectory_optimizer::
            OptimizationResult result;

        std::string message;

        const bool success =
            optimizer_.optimize(
                astar_path,
                result,
                message);

        if (!success)
        {
            RCLCPP_WARN(
                get_logger(),
                "%s",
                message.c_str());
            return;
        }

        publishPath(
            simplified_pub_,
            result.simplified_path);

        publishPath(
            control_points_pub_,
            result.control_points);

        const auto trajectory_points =
            trajectory_optimizer::
                TrajectoryOptimizer::
                sampleBSpline(
                    result.control_points,
                    result.dt,
                    0.03);

        publishPath(
            optimized_path_pub_,
            trajectory_points);

        publishCorridors(
            result.corridors);

        publishBSpline(
            result);

        RCLCPP_INFO(
            get_logger(),
            "%s",
            message.c_str());
    }


    void publishPath(
        const rclcpp::Publisher<
            nav_msgs::msg::Path>::
            SharedPtr &publisher,
        const std::vector<
            Eigen::Vector3d> &points)
    {
        nav_msgs::msg::Path path;

        path.header.stamp = now();
        path.header.frame_id = "map";

        for (const auto &p :
             points)
        {
            geometry_msgs::msg::
                PoseStamped pose;

            pose.header = path.header;

            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();

            pose.pose.orientation.w = 1.0;

            path.poses.push_back(pose);
        }

        publisher->publish(path);
    }


    void publishCorridors(
        const std::vector<
            trajectory_optimizer::
                CorridorBox> &corridors)
    {
        visualization_msgs::msg::
            MarkerArray array;

        visualization_msgs::msg::
            Marker clear;

        clear.action =
            visualization_msgs::msg::
                Marker::DELETEALL;

        array.markers.push_back(clear);

        int id = 0;

        for (const auto &box :
             corridors)
        {
            visualization_msgs::msg::
                Marker marker;

            marker.header.stamp = now();
            marker.header.frame_id = "map";

            marker.ns = "corridors";
            marker.id = id++;

            marker.type =
                visualization_msgs::msg::
                    Marker::CUBE;

            marker.action =
                visualization_msgs::msg::
                    Marker::ADD;

            const Eigen::Vector3d center =
                0.5 * (box.min + box.max);

            const Eigen::Vector3d size =
                box.max - box.min;

            marker.pose.position.x =
                center.x();

            marker.pose.position.y =
                center.y();

            marker.pose.position.z =
                center.z();

            marker.pose.orientation.w =
                1.0;

            marker.scale.x = size.x();
            marker.scale.y = size.y();
            marker.scale.z = size.z();

            marker.color.r = 0.2f;
            marker.color.g = 0.6f;
            marker.color.b = 1.0f;
            marker.color.a = 0.12f;

            array.markers.push_back(marker);
        }

        corridor_pub_->publish(array);
    }


    void publishBSpline(
        const trajectory_optimizer::
            OptimizationResult &result)
    {
        trajectory_optimizer::msg::
            BSplineTrajectory msg;

        msg.header.stamp = now();
        msg.header.frame_id = "map";

        msg.degree = 3;
        msg.dt = result.dt;
        msg.duration = result.duration;

        msg.control_points.reserve(
            result.control_points.size());

        for (const auto &p :
             result.control_points)
        {
            geometry_msgs::msg::Point point;

            point.x = p.x();
            point.y = p.y();
            point.z = p.z();

            msg.control_points.push_back(point);
        }

        bspline_pub_->publish(msg);
    }


private:
    trajectory_optimizer::
        TrajectoryOptimizer optimizer_;

    bool map_received_once_{false};

    std::string map_topic_;
    std::string path_topic_;

    rclcpp::Subscription<
        uav_mapping::msg::VoxelMap>::
        SharedPtr map_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Path>::
        SharedPtr path_sub_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::
        SharedPtr simplified_pub_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::
        SharedPtr control_points_pub_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::
        SharedPtr optimized_path_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::MarkerArray>::
        SharedPtr corridor_pub_;

    rclcpp::Publisher<
        trajectory_optimizer::msg::
            BSplineTrajectory>::
        SharedPtr bspline_pub_;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            TrajectoryOptimizerNode>());

    rclcpp::shutdown();

    return 0;
}