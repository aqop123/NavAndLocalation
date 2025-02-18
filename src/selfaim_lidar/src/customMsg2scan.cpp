#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Core>
#include <cmath>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#define X_TARGET
#define Y_TARGET
#define Z_TARGET    0

class CustomPoint2PointCloud2 : public rclcpp::Node
{
public:
    CustomPoint2PointCloud2(const rclcpp::NodeOptions & options) : Node("customp2pcl")
    {
        RCLCPP_ERROR(get_logger(), "Start ODOM Subscriber!");
        basket_target_ = (X_TARGET, Y_TARGET, Z_TARGET);
        global_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_cloud", rclcpp::SensorDataQoS());
        // split_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/split_cloud", 5);
        target_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_cloud", 5);
        yaw_pub = this->create_publisher<std_msgs::msg::Float64>("/aim_target", 1); 

        this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 10, std::bind(&CustomPoint2PointCloud2::odometry_callback, this, std::placeholders::_1));
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 10, std::bind(&CustomPoint2PointCloud2::CustomMsg_to_pointcloud2_callback, this, std::placeholders::_1));
    }

private:
    Eigen::Vector3d transformToRobotFrame(const Eigen::Vector3d& mid360_pos, 
        const Eigen::Matrix3d& rotation_matrix, 
        const Eigen::Vector3d& translation_vector) {
        // 应用旋转和平移
        Eigen::Vector3d robot_pos = rotation_matrix * mid360_pos + translation_vector;
        return robot_pos;
    }
    void transformAnglesToRobotFrame(double mid360_yaw, double mid360_pitch, 
                                double& robot_yaw, double& robot_pitch) {
        // 转换yaw角：减去90度（顺时针旋转90度），转换为弧度
        double yaw_offset = -M_PI / 2.0;  // -90度对应的弧度值
        robot_yaw = imu_yaw + yaw_offset;

        // 确保yaw角在[-π, π]范围内
        if (robot_yaw > M_PI) {
        robot_yaw -= 2.0 * M_PI;
        } else if (robot_yaw < -M_PI) {
        robot_yaw += 2.0 * M_PI;
        }

        // Pitch角不变
        robot_pitch = imu_pitch;
    }
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(msg == nullptr){
            return;
        }
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        geometry_msgs::msg::Pose::_orientation_type orientation = msg->pose.pose.orientation;
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
        
        Eigen::Vector3d mid360_pos(x, y, z);
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << 0, 1, 0,
                        -1, 0, 0,
                        0, 0, 1;
        // 定义平移向量（mid360坐标系到机器人坐标系）
        Eigen::Vector3d translation_vector(-0.077, -0.292, 0);  
        Eigen::Vector3d robot_pos = transformToRobotFrame(mid360_pos, rotation_matrix, translation_vector);
        double robot_yaw, robot_pitch;
        transformAnglesToRobotFrame(yaw, pitch, robot_yaw, robot_pitch);

        RCLCPP_INFO(get_logger(), "lidar:%f, %f, %f",x,y,yaw);
        RCLCPP_INFO(get_logger(), "robot:%f, %f, %f",robot_pos(0),robot_pos(1),robot_yaw);

        if(cur_pos_.empty()){
            cur_pos_.emplace_back(robot_pos(0));
            cur_pos_.emplace_back(robot_pos(1));
            cur_pos_.emplace_back(robot_yaw);
        }else{
            cur_pos_.emplace(cur_pos_.begin(), robot_pos(0));
            cur_pos_.emplace(cur_pos_.begin() + 1, robot_pos(1));
            cur_pos_.emplace(cur_pos_.begin() + 2, robot_yaw);
        }
    }

    pcl::PointXYZ convertToPCL(const livox_ros_driver2::msg::CustomPoint_<std::allocator<void>> &point) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        return pcl_point;
    }
    void CustomMsg_to_pointcloud2_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        if(msg == nullptr){
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = rclcpp::Time(msg->header.stamp);
        cloud_msg.header.frame_id = "body";

        sensor_msgs::msg::PointField field_x;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;
        sensor_msgs::msg::PointField field_y;
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;
        sensor_msgs::msg::PointField field_z;
        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;
        cloud_msg.fields = {field_x,field_y,field_z};

        // 设置消息中每个点的步长和每行的数据点云的长度
        cloud_msg.point_step = 12;
        cloud_msg.row_step = cloud_msg.point_step * 3;

        cloud_msg.height = 1;
        cloud_msg.is_dense = true;
        std::vector<uint8_t> points_data;

        cloud_points_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        int width = 0;

        for (const auto& point : msg->points)   // 筛选中间层的点云
        {
            pcl::PointXYZ pcl_point = convertToPCL(point);
            if (point.z > 0.07 && point.z < 0.35) 
            {
                float x = pcl_point.x;
                float y = pcl_point.y;
                float z = 1.0;
                points_data.insert(points_data.end(), reinterpret_cast<const uint8_t*>(&x), reinterpret_cast<const uint8_t*>(&x) + sizeof(x));
                points_data.insert(points_data.end(), reinterpret_cast<const uint8_t*>(&y), reinterpret_cast<const uint8_t*>(&y) + sizeof(y));
                points_data.insert(points_data.end(), reinterpret_cast<const uint8_t*>(&z), reinterpret_cast<const uint8_t*>(&z) + sizeof(z));

                if (!std::isinf(x) && !std::isinf(y))
                {
                    cloud_points_->push_back(pcl_point);
                }
                width++;
            }
        }

        cloud_msg.data = std::move(points_data);

        cloud_msg.width = width;
        cloud_msg.height = 1;
        cloud_msg.is_bigendian = true;

        // 发布PointCloud2消息——筛选高度后的全局点云
        global_cloud_pub->publish(cloud_msg);

        trans_target_ = point_trans();
        if(cloud_points_->size() > 0){  // 确保点云不为空
            match_cloud(cloud_points_);
            sensor_msgs::msg::PointCloud2 target_cloud_msg = ros2PointCloud2(target_point_);
            target_cloud_pub->publish(target_cloud_msg);
            pub_pos();
        }
    }

    // 将pcl::PointCloud转换为sensor_msgs::PointCloud2，能在rviz查看点云成像图
    sensor_msgs::msg::PointCloud2 ros2PointCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points){
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = rclcpp::Clock().now();
        cloud_msg.header.frame_id = "body";

        sensor_msgs::msg::PointField field_x;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;
        sensor_msgs::msg::PointField field_y;
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;
        sensor_msgs::msg::PointField field_z;
        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;
        cloud_msg.fields = {field_x,field_y,field_z};

        // 设置消息中每个点的步长和偏移量
        cloud_msg.point_step = 12;
        cloud_msg.row_step = cloud_msg.point_step * 3;

        int width = 0;
        cloud_msg.height = 1;
        cloud_msg.is_dense = true;
        std::vector<uint8_t> points_data;
        for (const auto& point : cloud_points->points)   // 筛选中间层的点云
        {
            float x = point.x;
            float y = point.y;
            float z = 1.0;
            points_data.insert(points_data.end(), reinterpret_cast<const uint8_t*>(&x), reinterpret_cast<const uint8_t*>(&x) + sizeof(x));
            points_data.insert(points_data.end(), reinterpret_cast<const uint8_t*>(&y), reinterpret_cast<const uint8_t*>(&y) + sizeof(float));
            points_data.insert(points_data.end(), reinterpret_cast<const uint8_t*>(&z), reinterpret_cast<const uint8_t*>(&z) + sizeof(float));
            width++;
        }

        cloud_msg.data = std::vector<uint8_t>(points_data);
        cloud_msg.width = width;
        cloud_msg.height = 1;
        cloud_msg.is_dense = true;

        return cloud_msg;
    }

    std::vector<double> point_trans(){
        double x,y,yaw;
        x = cur_pos_[0];
        y = cur_pos_[1];
        yaw = cur_pos_[2];

        /*-------获得目标点在机器人系下的坐标-------*/
        // std::vector<double> trans_target;
        // Eigen::Matrix2d yaw_matrix;
        // yaw_matrix << cos(yaw), -sin(yaw), 
        //               sin(yaw), cos(yaw);
        
        // double x_old = basket_target_[0];
        // double y_old = basket_target_[1];
        // Eigen::Vector2d trans_vector(x_old - x, y_old - y);
        // Eigen::Vector2d target_trans = yaw_matrix * trans_vector;
        // double new_yaw = atan2(target_trans(1), target_trans(0));
        // trans_target = {target_trans(0), target_trans(1), new_yaw};

        Egien::Vector3d robot_pos = (x,y,0);
        Egien::Maxtrix3d R_WorldRobot;
        R_WorldRobot << cos(yaw), -sin(yaw), 0,
                        sin(yaw), cos(yaw), 0,
                         0, 0, 1;
        
        Eigen::Vector3d BasketToRobot_pos = R_WorldRobot*basket_target_ + robot_pos;
        
        
        // 计算 yaw 角度
        double diff_yaw = atan2(BasketToRobot_pos(1), BasketToRobot_pos(0));
        std::vector<double> trans_target = {BasketToRobot_pos(0), BasketToRobot_pos(1), diff_yaw};
        return trans_target;
    }

    // 将目标点周围的点云成像
    void match_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points){
        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(cloud_points);
        
        pcl::PointXYZ searchPoint;
        searchPoint.x = trans_target_.at(0);
        searchPoint.y = trans_target_.at(1);
        searchPoint.z = trans_target_.at(2);

        /*-------------半径搜索---------------*/
        std::vector<int> pointIdx_radius_search; // 存储半径搜索得到的点云的点索引
	      std::vector<float> radius_squared_distance; // 存储距离目标点的距离
        float radius = 0.5;
        tree.radiusSearch(searchPoint, radius, pointIdx_radius_search, radius_squared_distance);

        if(pointIdx_radius_search.empty()){
            return;
        }
        // 筛选出目标周围的点云
        for (const auto& idx : pointIdx_radius_search) {
            target_point_->push_back(cloud_points->points[idx]);
        }

    }

    void pub_pos(){
        geometry_msgs::msg::Vector3 pose_msg;
        pose_msg.x = trans_target_[0];
        pose_msg.y = trans_target_[1];
        pose_msg.z = trans_target_[2];
        yaw_pub->publish(pose_msg);
    }

};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim::CustomPoint2PointCloud)

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CustomPoint2PointCloud2>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }