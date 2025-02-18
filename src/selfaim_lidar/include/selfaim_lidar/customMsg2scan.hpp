#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <Eigen/Core>
#include <cmath>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace auto_aim{
class CustomPoint2PointCloud : public rclcpp::Node
{
public:
    CustomPoint2PointCloud(const rclcpp::NodeOptions & options);

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    Eigen::Vector3d transformToRobotFrame(const Eigen::Vector3d& imu_point, 
                                            const Eigen::Matrix3d& rotation_matrix, 
                                            const Eigen::Vector3d& translation_vector);
    void transformAnglesToRobotFrame(double imu_yaw, double imu_pitch, 
                                     double& robot_yaw, double& robot_pitch) ;

    pcl::PointXYZ convertToPCL(const livox_ros_driver2::msg::CustomPoint_<std::allocator<void>> &point);
    void CustomMsg_to_pointcloud2_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    sensor_msgs::msg::PointCloud2 ros2PointCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points);
    std::vector<double> point_trans();
    void match_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points);
    void pub_pos();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr split_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub;
    std::vector<double> cur_pos_;
    std::vector<double> trans_target_;
    std::vector<double> global_target_;
    Eigen::Vector3d basket_target_;
    double basket_height;

};
}