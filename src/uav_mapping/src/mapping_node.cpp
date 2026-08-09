#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <uav_mapping/msg/voxel_map.hpp>

#include "uav_mapping/voxel_map.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <vector>


class MappingNode : public rclcpp::Node
{
public:

    MappingNode()
        : Node("uav_mapping")
    {
        // =====================================================
        // 1. ROS话题
        // =====================================================

        odom_topic_ =
            declare_parameter<std::string>(
                "odom_topic",
                "/fmu/out/vehicle_odometry");

        cloud_topic_ =
            declare_parameter<std::string>(
                "cloud_topic",
                "/lidar_3d/points");


        // =====================================================
        // 2. 地图参数
        // =====================================================

        resolution_ =
            declare_parameter<double>(
                "resolution",
                0.20);

        inflation_radius_ =
            declare_parameter<double>(
                "inflation_radius",
                0.45);


        auto map_min =
            declare_parameter<
                std::vector<double>>(
                "map_min",
                {-15.0, -15.0, -1.0});

        auto map_max =
            declare_parameter<
                std::vector<double>>(
                "map_max",
                {15.0, 15.0, 8.0});


        if (map_min.size() != 3 ||
            map_max.size() != 3)
        {
            throw std::runtime_error(
                "map_min/map_max必须是3维");
        }


        Eigen::Vector3d min_point(
            map_min[0],
            map_min[1],
            map_min[2]);

        Eigen::Vector3d max_point(
            map_max[0],
            map_max[1],
            map_max[2]);


        voxel_map_ =
            std::make_unique<
                uav_mapping::VoxelMap>(
                resolution_,
                min_point,
                max_point,
                inflation_radius_);


        // =====================================================
        // 3. LiDAR参数
        // =====================================================

        min_range_ =
            declare_parameter<double>(
                "min_range",
                0.3);

        max_range_ =
            declare_parameter<double>(
                "max_range",
                20.0);

        point_stride_ =
            declare_parameter<int>(
                "point_stride",
                1);


        // LiDAR原点相对于body FLU
        auto lidar_xyz =
            declare_parameter<
                std::vector<double>>(
                "lidar_xyz",
                {0.0, 0.0, 0.0});


        // LiDAR坐标系 -> Body FLU
        auto lidar_rpy =
            declare_parameter<
                std::vector<double>>(
                "lidar_rpy",
                {0.0, 0.0, 0.0});


        if (lidar_xyz.size() != 3 ||
            lidar_rpy.size() != 3)
        {
            throw std::runtime_error(
                "lidar_xyz/lidar_rpy必须是3维");
        }


        t_body_lidar_ =
            Eigen::Vector3d(
                lidar_xyz[0],
                lidar_xyz[1],
                lidar_xyz[2]);


        const double roll =
            lidar_rpy[0];

        const double pitch =
            lidar_rpy[1];

        const double yaw =
            lidar_rpy[2];


        R_body_lidar_ =
            Eigen::AngleAxisd(
                yaw,
                Eigen::Vector3d::UnitZ()) *

            Eigen::AngleAxisd(
                pitch,
                Eigen::Vector3d::UnitY()) *

            Eigen::AngleAxisd(
                roll,
                Eigen::Vector3d::UnitX());


        // =====================================================
        // 4. PX4 -> ROS坐标转换
        // =====================================================

        // NED -> ENU
        C_enu_ned_ <<
            0.0, 1.0,  0.0,
            1.0, 0.0,  0.0,
            0.0, 0.0, -1.0;


        // FLU -> FRD
        //
        // x不变
        // y取反
        // z取反
        C_frd_flu_ <<
            1.0,  0.0,  0.0,
            0.0, -1.0,  0.0,
            0.0,  0.0, -1.0;


        // =====================================================
        // 5. Subscriber
        // =====================================================

        // PX4官方推荐订阅PX4输出时使用
        // sensor data类型QoS。
        auto sensor_qos =
            rclcpp::SensorDataQoS();


        odom_sub_ =
            create_subscription<
                px4_msgs::msg::VehicleOdometry>(
                odom_topic_,
                sensor_qos,
                std::bind(
                    &MappingNode::odomCallback,
                    this,
                    std::placeholders::_1));


        cloud_sub_ =
            create_subscription<
                sensor_msgs::msg::PointCloud2>(
                cloud_topic_,
                sensor_qos,
                std::bind(
                    &MappingNode::cloudCallback,
                    this,
                    std::placeholders::_1));


        // =====================================================
        // 6. Publisher
        // =====================================================

        auto map_qos =
            rclcpp::QoS(1)
                .reliable()
                .transient_local();


        map_pub_ =
            create_publisher<
                uav_mapping::msg::VoxelMap>(
                "/uav_mapping/voxel_map",
                map_qos);


        occupied_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/uav_mapping/occupied_cloud",
                1);


        inflated_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/uav_mapping/inflated_cloud",
                1);


        // =====================================================
        // 7. 定时发布地图
        // =====================================================

        const double publish_rate =
            declare_parameter<double>(
                "publish_rate",
                2.0);


        timer_ =
            create_wall_timer(
                std::chrono::duration<double>(
                    1.0 / publish_rate),
                std::bind(
                    &MappingNode::publishMap,
                    this));


        RCLCPP_INFO(
            get_logger(),
            "uav_mapping启动");

        RCLCPP_INFO(
            get_logger(),
            "Map size = %d x %d x %d",
            voxel_map_->sizeX(),
            voxel_map_->sizeY(),
            voxel_map_->sizeZ());
    }


private:

    // =========================================================
    // PX4 Odometry
    // =========================================================

    void odomCallback(
        const px4_msgs::msg::VehicleOdometry::
            SharedPtr msg)
    {
        // 当前系统只使用NED位姿
        if (msg->pose_frame !=
            px4_msgs::msg::VehicleOdometry::
                POSE_FRAME_NED)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "当前VehicleOdometry不是NED");

            return;
        }


        // 检查位置
        if (!std::isfinite(msg->position[0]) ||
            !std::isfinite(msg->position[1]) ||
            !std::isfinite(msg->position[2]))
        {
            return;
        }


        // ------------------------------------
        // PX4位置 NED
        // ------------------------------------

        Eigen::Vector3d p_ned(
            msg->position[0],
            msg->position[1],
            msg->position[2]);


        // NED -> ENU
        p_map_body_ =
            C_enu_ned_ *
            p_ned;


        // ------------------------------------
        // PX4姿态
        //
        // q = [w,x,y,z]
        //
        // FRD -> NED
        // ------------------------------------

        Eigen::Quaterniond q_ned_frd(
            msg->q[0],
            msg->q[1],
            msg->q[2],
            msg->q[3]);


        if (!std::isfinite(q_ned_frd.w()) ||
            !std::isfinite(q_ned_frd.x()) ||
            !std::isfinite(q_ned_frd.y()) ||
            !std::isfinite(q_ned_frd.z()))
        {
            return;
        }


        if (q_ned_frd.norm() < 1e-6)
            return;


        q_ned_frd.normalize();


        const Eigen::Matrix3d R_ned_frd =
            q_ned_frd.toRotationMatrix();


        // ------------------------------------
        // FLU -> FRD -> NED -> ENU
        //
        // 得到：
        //
        // Body FLU -> Map ENU
        // ------------------------------------

        R_map_body_ =
            C_enu_ned_ *
            R_ned_frd *
            C_frd_flu_;


        odom_received_ =
            true;
    }


    // =========================================================
    // LiDAR PointCloud
    // =========================================================

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::
            SharedPtr msg)
    {
        if (!odom_received_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "等待PX4 Odometry");

            return;
        }


        // LiDAR原点在map中的位置
        const Eigen::Vector3d lidar_origin_map =
            p_map_body_ +
            R_map_body_ *
            t_body_lidar_;


        if (!voxel_map_->insideMap(
                lidar_origin_map))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "无人机/LiDAR超出地图");

            return;
        }


        sensor_msgs::
            PointCloud2ConstIterator<float>
            iter_x(*msg, "x");

        sensor_msgs::
            PointCloud2ConstIterator<float>
            iter_y(*msg, "y");

        sensor_msgs::
            PointCloud2ConstIterator<float>
            iter_z(*msg, "z");


        int count = 0;


        for (;
             iter_x != iter_x.end();
             ++iter_x,
             ++iter_y,
             ++iter_z)
        {
            ++count;


            // 点太多时可以抽样
            if (point_stride_ > 1 &&
                count % point_stride_ != 0)
            {
                continue;
            }


            Eigen::Vector3d p_lidar(
                *iter_x,
                *iter_y,
                *iter_z);


            if (!p_lidar.allFinite())
                continue;


            const double range =
                p_lidar.norm();


            if (range < min_range_ ||
                range > max_range_)
            {
                continue;
            }


            // ---------------------------------
            // LiDAR -> Body FLU
            // ---------------------------------

            const Eigen::Vector3d p_body =
                R_body_lidar_ *
                p_lidar +
                t_body_lidar_;


            // ---------------------------------
            // Body FLU -> Map ENU
            // ---------------------------------

            const Eigen::Vector3d p_map =
                R_map_body_ *
                p_body +
                p_map_body_;


            if (!voxel_map_->
                    insideMap(p_map))
            {
                continue;
            }


            // ---------------------------------
            // Raycasting
            // ---------------------------------

            voxel_map_->integrateRay(
                lidar_origin_map,
                p_map);
        }
    }


    // =========================================================
    // 发布地图
    // =========================================================

    void publishMap()
    {
        publishVoxelMap();

        publishCloud(
            occupied_pub_,
            false);

        publishCloud(
            inflated_pub_,
            true);
    }


    // =========================================================
    // 发布VoxelMap.msg
    // =========================================================

    void publishVoxelMap()
    {
        uav_mapping::msg::VoxelMap msg;


        msg.header.stamp =
            now();

        msg.header.frame_id =
            "map";


        msg.resolution =
            voxel_map_->resolution();


        msg.origin.x =
            voxel_map_->mapMin().x();

        msg.origin.y =
            voxel_map_->mapMin().y();

        msg.origin.z =
            voxel_map_->mapMin().z();


        msg.size_x =
            voxel_map_->sizeX();

        msg.size_y =
            voxel_map_->sizeY();

        msg.size_z =
            voxel_map_->sizeZ();


        voxel_map_->getPlanningData(
            msg.data);


        map_pub_->publish(
            msg);
    }


    // =========================================================
    // 发布PointCloud2给RViz
    // =========================================================

    void publishCloud(
        const rclcpp::Publisher<
            sensor_msgs::msg::PointCloud2>::
            SharedPtr &publisher,

        bool include_inflation)
    {
        const auto points =
            voxel_map_->getOccupiedPoints(
                include_inflation);


        sensor_msgs::msg::PointCloud2 cloud;


        cloud.header.stamp =
            now();

        cloud.header.frame_id =
            "map";


        sensor_msgs::PointCloud2Modifier
            modifier(cloud);


        modifier.setPointCloud2FieldsByString(
            1,
            "xyz");


        modifier.resize(
            points.size());


        sensor_msgs::
            PointCloud2Iterator<float>
            iter_x(cloud, "x");

        sensor_msgs::
            PointCloud2Iterator<float>
            iter_y(cloud, "y");

        sensor_msgs::
            PointCloud2Iterator<float>
            iter_z(cloud, "z");


        for (const auto &p : points)
        {
            *iter_x =
                static_cast<float>(p.x());

            *iter_y =
                static_cast<float>(p.y());

            *iter_z =
                static_cast<float>(p.z());


            ++iter_x;
            ++iter_y;
            ++iter_z;
        }


        publisher->publish(
            cloud);
    }


private:

    // 地图
    std::unique_ptr<
        uav_mapping::VoxelMap>
        voxel_map_;


    // Topics
    std::string odom_topic_;
    std::string cloud_topic_;


    // Map parameters
    double resolution_;
    double inflation_radius_;


    // LiDAR
    double min_range_;
    double max_range_;

    int point_stride_;


    Eigen::Vector3d
        t_body_lidar_{
            Eigen::Vector3d::Zero()};

    Eigen::Matrix3d
        R_body_lidar_{
            Eigen::Matrix3d::Identity()};


    // 当前无人机ENU/FLU位姿
    bool odom_received_{false};

    Eigen::Vector3d
        p_map_body_{
            Eigen::Vector3d::Zero()};

    Eigen::Matrix3d
        R_map_body_{
            Eigen::Matrix3d::Identity()};


    // 坐标系转换矩阵
    Eigen::Matrix3d
        C_enu_ned_;

    Eigen::Matrix3d
        C_frd_flu_;


    // ROS
    rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::
        SharedPtr odom_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::PointCloud2>::
        SharedPtr cloud_sub_;


    rclcpp::Publisher<
        uav_mapping::msg::VoxelMap>::
        SharedPtr map_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::
        SharedPtr occupied_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::
        SharedPtr inflated_pub_;


    rclcpp::TimerBase::SharedPtr
        timer_;
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
            MappingNode>());


    rclcpp::shutdown();

    return 0;
}