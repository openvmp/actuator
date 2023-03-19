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

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {}

void Implementation::init_actuator() {
  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  if (has_position()) {
    topic_position_ = node_->create_publisher<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_POSITION, qos);

    subscription_position_ = node_->create_subscription<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_POSITION_SET, qos,
        std::bind(&Implementation::sub_position_handler_, this,
                  std::placeholders::_1));
    srv_position_set_ =
        node_->create_service<remote_actuator::srv::PositionSet>(
            prefix + REMOTE_ACTUATOR_SERVICE_POSITION_SET,
            std::bind(&Implementation::position_set_handler_, this,
                      std::placeholders::_1, std::placeholders::_2),
            ::rmw_qos_profile_default, callback_group_);
  }

  if (has_velocity()) {
    topic_velocity_ = node_->create_publisher<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY, qos);

    subscription_velocity_ = node_->create_subscription<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY_SET, qos,
        std::bind(&Implementation::sub_velocity_handler_, this,
                  std::placeholders::_1));
    srv_velocity_set_ =
        node_->create_service<remote_actuator::srv::VelocitySet>(
            prefix + REMOTE_ACTUATOR_SERVICE_VELOCITY_SET,
            std::bind(&Implementation::velocity_set_handler_, this,
                      std::placeholders::_1, std::placeholders::_2),
            ::rmw_qos_profile_default, callback_group_);
  }
}

void Implementation::sub_position_handler_(
    const std_msgs::msg::Float64::SharedPtr msg) {
  position_set(msg->data);
}

void Implementation::sub_velocity_handler_(
    const std_msgs::msg::Float64::SharedPtr msg) {
  velocity_set(msg->data);
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

  std_msgs::msg::Float64 msg;
  msg.data = position;
  topic_position_->publish(msg);
}

void Implementation::velocity_set(double velocity) {
  velocity_set_real_(velocity);

  std_msgs::msg::Float64 msg;
  msg.data = velocity;
  topic_velocity_->publish(msg);
}

}  // namespace remote_actuator
