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

namespace remote_actuator {

class RemoteInterface final : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node,
                  const std::string &default_actuator_prefix = "");
  virtual ~RemoteInterface() {}

  virtual bool has_position() override;
  virtual bool has_velocity() override;
  virtual void position_set(double) override;
  virtual void velocity_set(double) override;

 private:
  bool has_position_;
  bool has_velocity_;

  rclcpp::Client<remote_actuator::srv::PositionSet>::SharedPtr
      clnt_position_set_;
  rclcpp::Client<remote_actuator::srv::VelocitySet>::SharedPtr
      clnt_velocity_set_;

  rclcpp::Client<remote_actuator::srv::PositionSet>::SharedPtr
  get_clnt_position_set_();
  rclcpp::Client<remote_actuator::srv::VelocitySet>::SharedPtr
  get_clnt_velocity_set_();
};

}  // namespace remote_actuator

#endif  // OPENVMP_REMOTE_ACTUATOR_INTERFACE_REMOTE_H
