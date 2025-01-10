// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "serial_driver/packet.hpp"
#include "serial_driver/rm_serial_driver.hpp"

namespace serial_driver
{
SerialDriver::SerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // aim_time_info_pub_ =
  //   this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("/time_info/aim", 10);
  // record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("/record_controller", 10);


  try {
    serial_driver_down_->init_port(device_down_, *device_config_);
    if (!serial_driver_down_->port()->is_open()) {
      serial_driver_down_->port()->open();
      receive_thread2_ = std::thread(&SerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_down_.c_str(), ex.what());
    throw ex;
  }
  try {
    serial_driver_up_->init_port(device_up_, *device_config_);
    if (!serial_driver_up_->port()->is_open()) {
      serial_driver_up_->port()->open();
      receive_thread1_ = std::thread(&SerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_up_.c_str(), ex.what());
    throw ex;
  }
  
  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  // aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&SerialDriver::sendBasketData, this, std::placeholders::_1));
  aim_sub_.subscribe(this, "/aim_target", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_);
  aim_sync_->registerCallback(
    std::bind(&SerialDriver::sendBasketData, this, std::placeholders::_1));

}

SerialDriver::~SerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

// void SerialDriver::receiveData()
// {
//   // 头部信息长度为1
//   std::vector<uint8_t> header(1);
//   // 初始化接收数据的变量
//   std::vector<uint8_t> data;
//   data.reserve(sizeof(ReceivePacket));

//   while (rclcpp::ok()) {
//     try {
//       serial_driver_->port()->receive(header);  // 接收头部信息

//       // 0x5A开头
//       if (header[0] == '[') {
//         data.resize(sizeof(ReceivePacket) - 1);
//         serial_driver_->port()->receive(data);  // 接收数据

//         // 将头部信息添加到数据中，避免丢
//         data.insert(data.begin(), header[0]);
//         // 将接收到的数据转换为数据包结构
//         ReceivePacket packet = fromVector(data);
        
//         // if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
//         //   setParam(rclcpp::Parameter("detect_color", packet.detect_color));
//         //   previous_receive_color_ = packet.detect_color;
//         // }

//         // RCLCPP_DEBUG(
//         //   get_logger(), "Game time: %d, Task mode: %d, Theory task: %s", packet.game_time,
//         //   packet.task_mode, theory_task.c_str());

//         // std_msgs::msg::String record_controller;
//         // record_controller.data = packet.is_play ? "start" : "stop";
//         // record_controller_pub_->publish(record_controller);

//         // 发布变换信息
//         geometry_msgs::msg::TransformStamped t;
//         timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
//         t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
//         t.header.frame_id = "odom";
//         t.child_frame_id = "gimbal_link";
//         tf2::Quaternion q;
//         q.setRPY(packet.roll, packet.pitch, packet.yaw);
//         t.transform.rotation = tf2::toMsg(q);
//         tf_broadcaster_->sendTransform(t);

//         // publish time
//         auto_aim_interfaces::msg::TimeInfo aim_time_info;
//         buff_interfaces::msg::TimeInfo buff_time_info;
//         aim_time_info.header = t.header;
//         aim_time_info.time = packet.timestamp;
//         buff_time_info.header = t.header;
//         buff_time_info.time = packet.timestamp;
//         aim_time_info_pub_->publish(aim_time_info);
//         buff_time_info_pub_->publish(buff_time_info);

//         if (abs(packet.aim_x) > 0.01) {
//           aiming_point_.header.stamp = this->now();
//           aiming_point_.pose.position.x = packet.aim_x;
//           aiming_point_.pose.position.y = packet.aim_y;
//           aiming_point_.pose.position.z = packet.aim_z;
//           marker_pub_->publish(aiming_point_);
//         }
        
//       } else {
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
//       }
//     } catch (const std::exception & ex) {
//       RCLCPP_ERROR_THROTTLE(
//         get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
//       reopenPort();
//     }
//   }
// }

void SerialDriver::sendBasketData(const geometry_msgs::msg::Vector3::ConstSharedPtr msg)
{
  try {
    SendPacket packet;
    packet.x = msg->x;
    packet.y = msg->y;
    packet.yaw = msg->z;
    // 20240329 ZY: Eliminate communication latency

    std::vector<uint8_t> data_up = toVector(packet);   // data：准备发的数据
    serial_driver_up_->port()->send(data_up);   // 发送数据

    serial_driver_down_->port()->send(data_down);

    // std_msgs::msg::Float64 latency;
    // latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    // RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    // latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}


void SerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE; // 流控制
  auto pt = Parity::NONE;      // 校验位
  auto sb = StopBits::ONE;     // 停止位

  try {
    device_down_ = declare_parameter<std::string>("device_name_down", "");
    device_up_   = declare_parameter<std::string>("device_name_up", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or "
        "hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

// 重新打开串口
void SerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_up_->port()->is_open()) {
      serial_driver_up_->port()->close();
    }
    if (serial_driver_down_->port()->is_open()) {
      serial_driver_down_->port()->close();
    }
    serial_driver_up_->port()->open();
    serial_driver_down_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}


}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriver)
