#include "selfaim_lidar/customMsg2scan.hpp"

#define BASKET_HEIGHT 2.5    // 篮筐高度
namespace auto_aim
{
    CustomPoint2PointCloud::CustomPoint2PointCloud(const rclcpp::NodeOptions & options) : Node("customp2pcl", options)
    {
        global_target_ = {};    // 存储目标点的x,y坐标

        pose_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/aim_target", 1); 

        this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 10, std::bind(&CustomPoint2PointCloud::odometry_callback, this, std::placeholders::_1));
    }

    void CustomPoint2PointCloud::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(msg == nullptr){
            return;
        }
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        geometry_msgs::msg::Pose::_orientation_type orientation = msg->pose.pose.orientation;
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
        
        geometry_msgs::msg::Vector3 pose_msg;
        pose_msg.x = x;
        pose_msg.y = y;
        pose_msg.z = yaw;
        pose_pub_->publish(pose_msg);

    }

}

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