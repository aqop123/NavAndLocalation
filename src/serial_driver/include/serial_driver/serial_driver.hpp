// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/Vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/time_info.hpp"
#include "buff_interfaces/msg/rune.hpp"
#include "buff_interfaces/msg/time_info.hpp"

namespace serial_driver
{
class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options);

  ~SerialDriver() override;

private:
  void getParams();

  // void receiveData();

  // void sendBasketData(const auto_aim_interfaces::msg::Target::ConstSharedPtr msg);
  void sendBasketData(const geometry_msgs::msg::Vector3::ConstSharedPtr msg);

  void reopenPort();

  // void setParam(const rclcpp::Parameter & param);


private:
  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_down_;
  std::string device_up_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_down_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_up_;


  // Broadcast tf from odom to gimbal_link
  // double timestamp_offset_ = 0;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr aim_sub_;

  message_filters::Subscriber<geometry_msgs::msg::Vector3> aim_sub_;


  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::Vector3> aim_syncpolicy;
  typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;
  std::shared_ptr<AimSync> aim_sync_;


  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::thread receive_thread_;

  // Time message
  rclcpp::Publisher<auto_aim_interfaces::msg::TimeInfo>::SharedPtr aim_time_info_pub_;
  rclcpp::Publisher<buff_interfaces::msg::TimeInfo>::SharedPtr buff_time_info_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_controller_pub_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
