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

private:

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_pub_;
    std::vector<double> cur_pos_;
    std::vector<double> trans_target_;
    std::vector<double> global_target_;

};
}