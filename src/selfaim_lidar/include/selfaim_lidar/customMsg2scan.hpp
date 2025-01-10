#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/Vector3.hpp"
#include <Eigen/Core>
#include <cmath>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

class CustomPoint2PointCloud : public rclcpp::Node
{
public:
    CustomPoint2PointCloud();

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    pcl::PointXYZ convertToPCL(const livox_ros_driver2::msg::CustomPoint_<std::allocator<void>> &point);

    void CustomMsg_to_pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    // 将pcl::PointCloud转换为sensor_msgs::PointCloud2，能在rviz查看点云成像图
    sensor_msgs::msg::PointCloud2 ros2PointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points);

    std::vector<double> point_trans();

    // 将目标点周围的点云成像
    void match_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points);

    void pub_basketlocation();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr split_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub;
    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_pub_;
    std::vector<double> cur_pos_;
    std::vector<double> trans_target_;
    std::vector<double> global_target_;

};