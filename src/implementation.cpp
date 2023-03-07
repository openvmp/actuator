/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/implementation.hpp"

namespace remote_actuator {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  topic_position_ = node->create_publisher<std_msgs::msg::Float32>(
      prefix + REMOTE_ACTUATOR_TOPIC_POSITION,
      rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  topic_velocity_ = node->create_publisher<std_msgs::msg::Float32>(
      prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY,
      rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  srv_position_set_ = node_->create_service<remote_actuator::srv::PositionSet>(
      prefix + REMOTE_ACTUATOR_SERVICE_POSITION_SET,
      std::bind(&Implementation::position_set_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  srv_velocity_set_ = node_->create_service<remote_actuator::srv::VelocitySet>(
      prefix + REMOTE_ACTUATOR_SERVICE_VELOCITY_SET,
      std::bind(&Implementation::velocity_set_handler_, this,
                std::placeholders::_1, std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
}

rclcpp::FutureReturnCode Implementation::position_set_handler_(
    const std::shared_ptr<remote_actuator::srv::PositionSet::Request> request,
    std::shared_ptr<remote_actuator::srv::PositionSet::Response> response) {
  (void)response;
  position_set(request->position);
  return rclcpp::FutureReturnCode::SUCCESS;
}

rclcpp::FutureReturnCode Implementation::velocity_set_handler_(
    const std::shared_ptr<remote_actuator::srv::VelocitySet::Request> request,
    std::shared_ptr<remote_actuator::srv::VelocitySet::Response> response) {
  (void)response;
  velocity_set(request->velocity);
  return rclcpp::FutureReturnCode::SUCCESS;
}

void Implementation::position_set(double position) {
  position_set_real_(position);

  std_msgs::msg::Float32 msg;
  msg.data = position;
  topic_position_->publish(msg);
}

void Implementation::velocity_set(double velocity) {
  velocity_set_real_(velocity);

  std_msgs::msg::Float32 msg;
  msg.data = velocity;
  topic_velocity_->publish(msg);
}

}  // namespace remote_actuator