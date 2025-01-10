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

#include "rc_serial_driver/packet.hpp"
#include "rc_serial_driver/rc_serial_driver.hpp"

namespace rc_serial_driver
{
RCSerialDriver::RCSerialDriver(const rclcpp::NodeOptions & options)
: Node("rcserial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_down_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  getParams();

  // Create Subscription
  aim_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "/aim_target", rclcpp::SensorDataQoS(),
    std::bind(&RCSerialDriver::sendBasketData, this, std::placeholders::_1));
  // aim_sub_.subscribe(this, "/aim_target", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  // aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_);
  // aim_sync_->registerCallback(
  //   std::bind(&SerialDriver::sendBasketData, this, std::placeholders::_1));

}

RCSerialDriver::~RCSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_down_->port()->is_open()) {
    serial_driver_down_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


void RCSerialDriver::sendBasketData(const geometry_msgs::msg::Vector3::ConstSharedPtr msg)
{
  try {
    SendPacket packet;
    packet.x = msg->x;
    packet.y = msg->y;
    packet.yaw = msg->z;
    // 20240329 ZY: Eliminate communication latency

    std::vector<uint8_t> data_down = toVector(packet);   // 
    serial_driver_down_->port()->send(data_down);   // 

    //serial_driver_down_->port()->send(data_down);

    // std_msgs::msg::Float64 latency;
    // latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    // RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    // latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}


void RCSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE; // 
  auto pt = Parity::NONE;      // 
  auto sb = StopBits::ONE;     // 

  try {
    device_down_ = declare_parameter<std::string>("device_name_down", "");
    // device_up_   = declare_parameter<std::string>("device_name_up", "");
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


void RCSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_down_->port()->is_open()) {
      serial_driver_down_->port()->close();
    }
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
RCLCPP_COMPONENTS_REGISTER_NODE(rc_serial_driver::RCSerialDriver)
