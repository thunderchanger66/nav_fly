#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <uav_mapping/msg/voxel_map.hpp>

#include "uav_mapping/voxel_map.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>


class MappingNode : public rclcpp::Node
{
public:

    struct PoseData
    {
        // 仿真时间，单位 s
        double time{0.0};

        // Body FLU 原点在 map ENU 中的位置
        Eigen::Vector3d position{
            Eigen::Vector3d::Zero()};

        // Body FLU -> Map ENU
        Eigen::Quaterniond orientation{
            Eigen::Quaterniond::Identity()};
    };


    MappingNode()
        : Node("uav_mapping")
    {
        // =====================================================
        // 1. 参数
        // =====================================================

        self_filter_enable_ =
            declare_parameter<bool>(
                "self_filter_enable",
                true);
        auto self_filter_min =
            declare_parameter<std::vector<double>>(
                "self_filter_min",
                {-0.45, -0.45, -0.20});
        auto self_filter_max =
            declare_parameter<std::vector<double>>(
                "self_filter_max",
                {0.45, 0.45, 0.25});
        if (self_filter_min.size() != 3 ||
            self_filter_max.size() != 3)
        {
            throw std::runtime_error(
                "self_filter_min/max 必须为3维");
        }
        self_filter_min_ =
            Eigen::Vector3d(
                self_filter_min[0],
                self_filter_min[1],
                self_filter_min[2]);
        self_filter_max_ =
            Eigen::Vector3d(
                self_filter_max[0],
                self_filter_max[1],
                self_filter_max[2]);

        lidar_time_offset_ =
            declare_parameter<double>(
                "lidar_time_offset",
                0.0);

        odom_topic_ =
            declare_parameter<std::string>(
                "odom_topic",
                "/fmu/out/vehicle_odometry");

        cloud_topic_ =
            declare_parameter<std::string>(
                "cloud_topic",
                "/lidar_3d/points");

        frame_id_ =
            declare_parameter<std::string>(
                "frame_id",
                "map");


        resolution_ =
            declare_parameter<double>(
                "resolution",
                0.20);

        inflation_radius_ =
            declare_parameter<double>(
                "inflation_radius",
                0.45);


        auto map_min =
            declare_parameter<std::vector<double>>(
                "map_min",
                {-15.0, -15.0, -1.0});

        auto map_max =
            declare_parameter<std::vector<double>>(
                "map_max",
                {15.0, 15.0, 8.0});


        if (map_min.size() != 3 ||
            map_max.size() != 3)
        {
            throw std::runtime_error(
                "map_min/map_max 必须包含3个元素");
        }


        voxel_map_ =
            std::make_unique<uav_mapping::VoxelMap>(
                resolution_,
                Eigen::Vector3d(
                    map_min[0],
                    map_min[1],
                    map_min[2]),
                Eigen::Vector3d(
                    map_max[0],
                    map_max[1],
                    map_max[2]),
                inflation_radius_);


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


        pose_buffer_duration_ =
            declare_parameter<double>(
                "pose_buffer_duration",
                3.0);

        max_cloud_queue_size_ =
            declare_parameter<int>(
                "max_cloud_queue_size",
                10);


        // =====================================================
        // 2. LiDAR外参
        //
        // p_body =
        // R_body_lidar * p_lidar + t_body_lidar
        // =====================================================

        auto lidar_xyz =
            declare_parameter<std::vector<double>>(
                "lidar_xyz",
                {0.0, 0.0, 0.0});

        auto lidar_rpy =
            declare_parameter<std::vector<double>>(
                "lidar_rpy",
                {0.0, 0.0, 0.0});


        if (lidar_xyz.size() != 3 ||
            lidar_rpy.size() != 3)
        {
            throw std::runtime_error(
                "lidar_xyz/lidar_rpy 必须包含3个元素");
        }


        t_body_lidar_ =
            Eigen::Vector3d(
                lidar_xyz[0],
                lidar_xyz[1],
                lidar_xyz[2]);


        R_body_lidar_ =
            Eigen::AngleAxisd(
                lidar_rpy[2],
                Eigen::Vector3d::UnitZ()) *

            Eigen::AngleAxisd(
                lidar_rpy[1],
                Eigen::Vector3d::UnitY()) *

            Eigen::AngleAxisd(
                lidar_rpy[0],
                Eigen::Vector3d::UnitX());


        // =====================================================
        // 3. PX4 -> ROS 坐标转换
        // =====================================================

        // NED -> ENU
        C_enu_ned_ <<
            0.0, 1.0,  0.0,
            1.0, 0.0,  0.0,
            0.0, 0.0, -1.0;


        // FLU -> FRD
        C_frd_flu_ <<
            1.0,  0.0,  0.0,
            0.0, -1.0,  0.0,
            0.0,  0.0, -1.0;


        // =====================================================
        // 4. ROS通信
        // =====================================================

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


        // 当前单帧点云转换到map以后直接发布
        // 专门用于检查坐标变换是否正确
        current_cloud_pub_ =
            create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/uav_mapping/current_cloud_map",
                1);


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
            "uav_mapping 时间对齐版本启动");

        RCLCPP_INFO(
            get_logger(),
            "Map = %d x %d x %d, resolution = %.2f",
            voxel_map_->sizeX(),
            voxel_map_->sizeY(),
            voxel_map_->sizeZ(),
            resolution_);
    }


private:

    // =========================================================
    // PX4 odometry callback
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


        // -------------------------
        // PX4 NED位置
        // -------------------------

        const Eigen::Vector3d p_ned(
            msg->position[0],
            msg->position[1],
            msg->position[2]);


        // -------------------------
        // PX4四元数：
        // [w, x, y, z]
        // FRD -> NED
        // -------------------------

        Eigen::Quaterniond q_ned_frd(
            msg->q[0],
            msg->q[1],
            msg->q[2],
            msg->q[3]);


        if (!q_ned_frd.coeffs().allFinite() ||
            q_ned_frd.norm() < 1e-6)
        {
            return;
        }


        q_ned_frd.normalize();


        const Eigen::Matrix3d R_ned_frd =
            q_ned_frd.toRotationMatrix();


        // -------------------------
        // 转成 ROS：
        // Body FLU -> Map ENU
        // -------------------------

        const Eigen::Vector3d p_map_body =
            C_enu_ned_ *
            p_ned;


        const Eigen::Matrix3d R_map_body =
            C_enu_ned_ *
            R_ned_frd *
            C_frd_flu_;


        Eigen::Quaterniond q_map_body(
            R_map_body);

        q_map_body.normalize();


        // -------------------------
        // 保存历史位姿
        // timestamp_sample 单位 us
        // -------------------------

        PoseData pose;

        pose.time =
            static_cast<double>(
                msg->timestamp_sample) *
            1e-6;

        pose.position =
            p_map_body;

        pose.orientation =
            q_map_body;


        // 防止偶尔收到乱序数据
        if (!pose_buffer_.empty() &&
            pose.time <
            pose_buffer_.back().time)
        {
            return;
        }


        pose_buffer_.push_back(
            pose);


        // 只保留最近几秒
        while (!pose_buffer_.empty() &&
               pose.time -
               pose_buffer_.front().time >
               pose_buffer_duration_)
        {
            pose_buffer_.pop_front();
        }


        // 新位姿到达以后，
        // 尝试处理之前等待时间对齐的点云
        processPendingClouds();
    }


    // =========================================================
    // LiDAR callback
    //
    // 不立即强行使用“最新姿态”。
    // 先放进队列，等PX4位姿覆盖该时间。
    // =========================================================

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::
            SharedPtr msg)
    {
        cloud_queue_.push_back(
            msg);


        // 防止异常情况下无限增长
        while (static_cast<int>(
                   cloud_queue_.size()) >
               max_cloud_queue_size_)
        {
            cloud_queue_.pop_front();

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "LiDAR等待队列过长，丢弃最旧点云");
        }


        processPendingClouds();
    }


    // =========================================================
    // ROS时间戳 -> 秒
    // =========================================================

    double cloudTime(
        const sensor_msgs::msg::PointCloud2 &msg) const
    {
        return
            static_cast<double>(
                msg.header.stamp.sec) +

            static_cast<double>(
                msg.header.stamp.nanosec) *
            1e-9;
    }


    // =========================================================
    // 根据指定时间插值PX4位姿
    // =========================================================

    bool getPoseAtTime(
        double query_time,
        Eigen::Vector3d &position,
        Eigen::Matrix3d &rotation) const
    {
        if (pose_buffer_.size() < 2)
            return false;


        // 比缓存中最旧的还早
        if (query_time <
            pose_buffer_.front().time)
        {
            return false;
        }


        // 比最新位姿还新：
        // 不能使用latest pose硬顶，
        // 必须继续等待后续odometry。
        if (query_time >
            pose_buffer_.back().time)
        {
            return false;
        }


        // 正好等于最后一个
        if (std::abs(
                query_time -
                pose_buffer_.back().time) <
            1e-7)
        {
            position =
                pose_buffer_.back().position;

            rotation =
                pose_buffer_.back()
                    .orientation
                    .toRotationMatrix();

            return true;
        }


        for (std::size_t i = 0;
             i + 1 < pose_buffer_.size();
             ++i)
        {
            const PoseData &p1 =
                pose_buffer_[i];

            const PoseData &p2 =
                pose_buffer_[i + 1];


            if (query_time <
                    p1.time ||
                query_time >
                    p2.time)
            {
                continue;
            }


            const double dt =
                p2.time -
                p1.time;


            if (dt < 1e-9)
            {
                position =
                    p1.position;

                rotation =
                    p1.orientation
                        .toRotationMatrix();

                return true;
            }


            double alpha =
                (query_time -
                 p1.time) /
                dt;


            alpha =
                std::clamp(
                    alpha,
                    0.0,
                    1.0);


            // ---------------------
            // 位置：线性插值
            // ---------------------

            position =
                (1.0 - alpha) *
                    p1.position +
                alpha *
                    p2.position;


            // ---------------------
            // 姿态：四元数SLERP
            // ---------------------

            Eigen::Quaterniond q =
                p1.orientation.slerp(
                    alpha,
                    p2.orientation);


            q.normalize();


            rotation =
                q.toRotationMatrix();


            return true;
        }


        return false;
    }


    // =========================================================
    // 处理等待中的LiDAR
    // =========================================================

    void processPendingClouds()
    {
        if (pose_buffer_.size() < 2)
            return;


        while (!cloud_queue_.empty())
        {
            const auto &cloud =
                cloud_queue_.front();


            // Use the same corrected timestamp as processCloud().  Without
            // this, a non-zero LiDAR offset can make a valid cloud leave the
            // queue before the corresponding odometry sample is available.
            const double t =
                cloudTime(
                    *cloud) +
                lidar_time_offset_;


            // 点云太旧，连缓存最早位姿都比它新
            if (t <
                pose_buffer_.front().time)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "LiDAR点云 %.6f 太旧，无法找到对应PX4位姿",
                    t);

                cloud_queue_.pop_front();

                continue;
            }


            // 当前PX4还没有运行到该点云时间
            // 等下一帧odometry再处理
            if (t >
                pose_buffer_.back().time)
            {
                return;
            }


            auto cloud_to_process =
                cloud_queue_.front();

            cloud_queue_.pop_front();


            processCloud(
                cloud_to_process);
        }
    }


    // =========================================================
    // 真正处理一帧点云
    // =========================================================

    void processCloud(
        const sensor_msgs::msg::PointCloud2::
            SharedPtr msg)
    {
        const double t =
            cloudTime(
                *msg) + lidar_time_offset_;


        Eigen::Vector3d p_map_body;

        Eigen::Matrix3d R_map_body;


        if (!getPoseAtTime(
                t,
                p_map_body,
                R_map_body))
        {
            return;
        }


        // 当前 LiDAR 原点在 map 中的位置
        const Eigen::Vector3d lidar_origin_map =
            p_map_body +
            R_map_body *
            t_body_lidar_;


        if (!voxel_map_->
                insideMap(
                    lidar_origin_map))
        {
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


        std::vector<Eigen::Vector3d>
            current_points;


        current_points.reserve(
            static_cast<std::size_t>(
                msg->width *
                msg->height /
                std::max(
                    1,
                    point_stride_)));


        int count = 0;


        for (;
             iter_x != iter_x.end();
             ++iter_x,
             ++iter_y,
             ++iter_z)
        {
            ++count;


            if (point_stride_ > 1 &&
                count %
                    point_stride_ != 0)
            {
                continue;
            }


            const Eigen::Vector3d p_lidar(
                *iter_x,
                *iter_y,
                *iter_z);


            if (!p_lidar.allFinite())
                continue;


            const double range =
                p_lidar.norm();


            if (range <
                    min_range_ ||
                range >
                    max_range_)
            {
                continue;
            }


            // LiDAR -> Body FLU
            const Eigen::Vector3d p_body =
                R_body_lidar_ *
                p_lidar +
                t_body_lidar_;

            // 无人机自身反射点不进入地图
            if (isSelfPoint(p_body))
            {
                continue;
            }

            // Body FLU -> Map ENU
            const Eigen::Vector3d p_map =
                R_map_body *
                p_body +
                p_map_body;


            if (!voxel_map_->
                    insideMap(
                        p_map))
            {
                continue;
            }


            current_points.push_back(
                p_map);


            voxel_map_->integrateRay(
                lidar_origin_map,
                p_map);
        }


        // 发布“当前这一帧”转换后的点云
        // 用它判断坐标转换是否稳定
        publishPointCloud(
            current_cloud_pub_,
            current_points);
    }


    // =========================================================
    // 定时发布累积地图
    // =========================================================

    void publishMap()
    {
        publishVoxelMap();


        publishPointCloud(
            occupied_pub_,
            voxel_map_->
                getOccupiedPoints(
                    false));


        publishPointCloud(
            inflated_pub_,
            voxel_map_->
                getOccupiedPoints(
                    true));
    }


    // =========================================================
    // 发布VoxelMap
    // =========================================================

    void publishVoxelMap()
    {
        uav_mapping::msg::VoxelMap msg;


        msg.header.stamp =
            now();

        msg.header.frame_id =
            frame_id_;


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


        voxel_map_->
            getPlanningData(
                msg.data);


        map_pub_->publish(
            msg);
    }


    // =========================================================
    // 发布Eigen点集为PointCloud2
    // =========================================================

    void publishPointCloud(
        const rclcpp::Publisher<
            sensor_msgs::msg::PointCloud2>::
            SharedPtr &publisher,

        const std::vector<
            Eigen::Vector3d> &points)
    {
        sensor_msgs::msg::PointCloud2
            cloud;


        cloud.header.stamp =
            now();

        cloud.header.frame_id =
            frame_id_;


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


        for (const auto &p :
             points)
        {
            *iter_x =
                static_cast<float>(
                    p.x());

            *iter_y =
                static_cast<float>(
                    p.y());

            *iter_z =
                static_cast<float>(
                    p.z());


            ++iter_x;
            ++iter_y;
            ++iter_z;
        }


        publisher->publish(
            cloud);
    }

    // =========================================================
    // 判断激光点是否打到了无人机自身
    //
    // 输入必须是 Body FLU 坐标系下的点
    // =========================================================
    bool isSelfPoint(
        const Eigen::Vector3d &p_body) const
    {
        if (!self_filter_enable_)
            return false;


        return
            p_body.x() >= self_filter_min_.x() &&
            p_body.x() <= self_filter_max_.x() &&

            p_body.y() >= self_filter_min_.y() &&
            p_body.y() <= self_filter_max_.y() &&

            p_body.z() >= self_filter_min_.z() &&
            p_body.z() <= self_filter_max_.z();
    }

private:

    bool self_filter_enable_{true};
    Eigen::Vector3d self_filter_min_{
        -0.45, -0.45, -0.20};
    Eigen::Vector3d self_filter_max_{
        0.45,  0.45,  0.25};

    double lidar_time_offset_{0.0};

    // =========================================================
    // Map
    // =========================================================

    std::unique_ptr<
        uav_mapping::VoxelMap>
        voxel_map_;


    double resolution_{0.2};
    double inflation_radius_{0.45};

    std::string frame_id_{"map"};


    // =========================================================
    // LiDAR
    // =========================================================

    std::string cloud_topic_;

    double min_range_{0.3};
    double max_range_{20.0};

    int point_stride_{1};


    Eigen::Vector3d
        t_body_lidar_{
            Eigen::Vector3d::Zero()};

    Eigen::Matrix3d
        R_body_lidar_{
            Eigen::Matrix3d::Identity()};


    // =========================================================
    // PX4 pose buffer
    // =========================================================

    std::string odom_topic_;

    std::deque<PoseData>
        pose_buffer_;

    std::deque<
        sensor_msgs::msg::PointCloud2::
            SharedPtr>
        cloud_queue_;


    double pose_buffer_duration_{3.0};

    int max_cloud_queue_size_{10};


    Eigen::Matrix3d
        C_enu_ned_;

    Eigen::Matrix3d
        C_frd_flu_;


    // =========================================================
    // ROS
    // =========================================================

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

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::
        SharedPtr current_cloud_pub_;


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
