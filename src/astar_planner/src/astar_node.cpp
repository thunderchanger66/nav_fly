#include <rclcpp/rclcpp.hpp>

#include <uav_mapping/msg/voxel_map.hpp>

#include <astar_planner/srv/plan_path.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "astar_planner/astar_3d.hpp"

#include <memory>
#include <string>
#include <vector>


class AStarNode : public rclcpp::Node
{
public:

    AStarNode()
        : Node("astar_planner")
    {
        // ============================
        // 参数
        // ============================

        map_topic_ =
            declare_parameter<std::string>(
                "map_topic",
                "/uav_mapping/voxel_map");


        path_topic_ =
            declare_parameter<std::string>(
                "path_topic",
                "/astar/path");


        const bool allow_unknown =
            declare_parameter<bool>(
                "allow_unknown",
                true);


        const double unknown_cost =
            declare_parameter<double>(
                "unknown_cost",
                2.5);


        astar_.setAllowUnknown(
            allow_unknown);

        astar_.setUnknownCost(
            unknown_cost);


        // ============================
        // VoxelMap subscriber
        //
        // mapping端使用transient_local，
        // 这里同样使用transient_local。
        // ============================

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
                    &AStarNode::mapCallback,
                    this,
                    std::placeholders::_1));


        // ============================
        // Path publisher
        // ============================

        path_pub_ =
            create_publisher<
                nav_msgs::msg::Path>(
                path_topic_,
                rclcpp::QoS(1)
                    .reliable()
                    .transient_local());


        // ============================
        // Plan service
        // ============================

        plan_service_ =
            create_service<
                astar_planner::srv::PlanPath>(
                "/astar/plan",

                std::bind(
                    &AStarNode::planCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));


        RCLCPP_INFO(
            get_logger(),
            "astar_planner启动");

        RCLCPP_INFO(
            get_logger(),
            "allow_unknown = %s, unknown_cost = %.2f",
            allow_unknown ? "true" : "false",
            unknown_cost);
    }


private:

    // ========================================================
    // 地图更新
    // ========================================================

    void mapCallback(
        const uav_mapping::msg::VoxelMap::
            SharedPtr msg)
    {
        astar_.setMap(
            *msg);


        if (!astar_.hasMap())
        {
            RCLCPP_ERROR(
                get_logger(),
                "收到的VoxelMap无效");

            return;
        }


        if (!map_received_once_)
        {
            RCLCPP_INFO(
                get_logger(),
                "收到VoxelMap: %u x %u x %u, resolution=%.3f",
                msg->size_x,
                msg->size_y,
                msg->size_z,
                msg->resolution);

            map_received_once_ =
                true;
        }
    }


    // ========================================================
    // A* service
    // ========================================================

    void planCallback(
        const std::shared_ptr<
            astar_planner::srv::PlanPath::Request>
            request,

        std::shared_ptr<
            astar_planner::srv::PlanPath::Response>
            response)
    {
        if (!astar_.hasMap())
        {
            response->success =
                false;

            response->message =
                "尚未收到VoxelMap";

            return;
        }


        const std::array<double, 3>
            start = {
                request->start.x,
                request->start.y,
                request->start.z
            };


        const std::array<double, 3>
            goal = {
                request->goal.x,
                request->goal.y,
                request->goal.z
            };


        RCLCPP_INFO(
            get_logger(),
            "规划: start=(%.2f %.2f %.2f), goal=(%.2f %.2f %.2f)",
            start[0],
            start[1],
            start[2],
            goal[0],
            goal[1],
            goal[2]);


        std::vector<
            std::array<double, 3>>
            points;


        std::string message;


        const bool success =
            astar_.search(
                start,
                goal,
                points,
                message);


        response->success =
            success;

        response->message =
            message;


        if (!success)
        {
            RCLCPP_WARN(
                get_logger(),
                "%s",
                message.c_str());

            return;
        }


        // ============================
        // 转成nav_msgs/Path
        // ============================

        nav_msgs::msg::Path path;


        path.header.stamp =
            now();

        path.header.frame_id =
            "map";


        path.poses.reserve(
            points.size());


        for (const auto &point :
             points)
        {
            geometry_msgs::msg::PoseStamped
                pose;


            pose.header =
                path.header;


            pose.pose.position.x =
                point[0];

            pose.pose.position.y =
                point[1];

            pose.pose.position.z =
                point[2];


            // A*目前不定义姿态
            pose.pose.orientation.w =
                1.0;


            path.poses.push_back(
                pose);
        }


        response->path =
            path;


        path_pub_->publish(
            path);


        RCLCPP_INFO(
            get_logger(),
            "%s",
            message.c_str());
    }


private:

    astar_planner::AStar3D
        astar_;


    bool map_received_once_{
        false};


    std::string map_topic_;
    std::string path_topic_;


    rclcpp::Subscription<
        uav_mapping::msg::VoxelMap>::
        SharedPtr map_sub_;


    rclcpp::Publisher<
        nav_msgs::msg::Path>::
        SharedPtr path_pub_;


    rclcpp::Service<
        astar_planner::srv::PlanPath>::
        SharedPtr plan_service_;
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
            AStarNode>());


    rclcpp::shutdown();

    return 0;
}