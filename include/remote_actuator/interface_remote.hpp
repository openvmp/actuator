/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */
#ifndef OPENVMP_REMOTE_ACTUATOR_INTERFACE_REMOTE_H
#define OPENVMP_REMOTE_ACTUATOR_INTERFACE_REMOTE_H

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/interface.hpp"
#include "std_msgs/msg/float64.hpp"

namespace remote_actuator {

class RemoteInterface final : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node,
                  const std::string &default_actuator_prefix = "");
  virtual ~RemoteInterface();

  virtual bool has_position() override;
  virtual bool has_velocity() override;
  virtual void position_set(double) override;
  virtual void velocity_set(double) override;

 private:
  bool has_position_;
  bool has_velocity_;

  // publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_position_set_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_velocity_set_;

  // clients
  rclcpp::Client<remote_actuator::srv::PositionSet>::SharedPtr
      clnt_position_set_;
  rclcpp::Client<remote_actuator::srv::VelocitySet>::SharedPtr
      clnt_velocity_set_;

  void get_clnt_position_set_();
  void get_clnt_velocity_set_();
};

}  // namespace remote_actuator

#endif  // OPENVMP_REMOTE_ACTUATOR_INTERFACE_REMOTE_H
