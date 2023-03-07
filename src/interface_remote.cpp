/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/interface_remote.hpp"

#include <functional>

namespace remote_actuator {

RemoteInterface::RemoteInterface(rclcpp::Node *node,
                                 const std::string &default_actuator_prefix)
    : Interface(node, default_actuator_prefix) {
  auto prefix = get_prefix_();
  auto callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  RCLCPP_DEBUG(
      node_->get_logger(),
      "remote_actuator::RemoteInterface::RemoteInterface(): Connecting to the "
      "remote interface: %s",
      prefix.c_str());

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

rclcpp::Client<remote_actuator::srv::PositionSet>::SharedPtr
RemoteInterface::get_clnt_position_set_() {
  if (!clnt_position_set_) {
    auto prefix = get_prefix_();

    clnt_position_set_ =
        node_->create_client<remote_actuator::srv::PositionSet>(
            prefix + REMOTE_ACTUATOR_SERVICE_POSITION_SET,
            ::rmw_qos_profile_default, callback_group_);

    has_position_ =
        clnt_velocity_set_->wait_for_service(std::chrono::milliseconds(100));
  }
  return clnt_position_set_;
}

rclcpp::Client<remote_actuator::srv::VelocitySet>::SharedPtr
RemoteInterface::get_clnt_velocity_set_() {
  if (!clnt_velocity_set_) {
    auto prefix = get_prefix_();

    clnt_velocity_set_ =
        node_->create_client<remote_actuator::srv::VelocitySet>(
            prefix + REMOTE_ACTUATOR_SERVICE_VELOCITY_SET,
            ::rmw_qos_profile_default, callback_group_);

    has_velocity_ =
        clnt_velocity_set_->wait_for_service(std::chrono::milliseconds(100));
  }
  return clnt_velocity_set_;
}

bool RemoteInterface::has_position() {
  (void)get_clnt_position_set_();
  return has_position_;
}

bool RemoteInterface::has_velocity() {
  (void)get_clnt_velocity_set_();
  return has_velocity_;
}

void RemoteInterface::position_set(double position) {
  if (!has_position()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "RemoteInterface::position_set(): not connected");
    return;
  }

  auto req = std::make_shared<remote_actuator::srv::PositionSet::Request>();
  auto resp = std::make_shared<remote_actuator::srv::PositionSet::Response>();

  req->position = position;

  auto f = clnt_position_set_->async_send_request(req);
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::position_set(): response received");
}

void RemoteInterface::velocity_set(double velocity) {
  if (!has_velocity()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "RemoteInterface::velocity_set(): not connected");
    return;
  }

  auto req = std::make_shared<remote_actuator::srv::VelocitySet::Request>();
  auto resp = std::make_shared<remote_actuator::srv::VelocitySet::Response>();

  req->velocity = velocity;

  auto f = clnt_velocity_set_->async_send_request(req);
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::velocity_set(): response received");
}

}  // namespace remote_actuator
