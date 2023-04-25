/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/implementation.hpp"

#include <limits>

namespace remote_actuator {

Implementation::Implementation(rclcpp::Node *node,
                               const std::string &default_actuator_prefix)
    : Interface(node, default_actuator_prefix) {
  if (!node_->has_parameter("actuator_position_min")) {
    node_->declare_parameter("actuator_position_min",
                             std::numeric_limits<double>::lowest());
  }
  node_->get_parameter("actuator_position_min", param_position_min_);

  if (!node_->has_parameter("actuator_position_max")) {
    node_->declare_parameter("actuator_position_max",
                             std::numeric_limits<double>::max());
  }
  node_->get_parameter("actuator_position_max", param_position_max_);

  if (!node_->has_parameter("actuator_velocity_min")) {
    node_->declare_parameter("actuator_velocity_min",
                             std::numeric_limits<double>::lowest());
  }
  node_->get_parameter("actuator_velocity_min", param_velocity_min_);

  if (!node_->has_parameter("actuator_velocity_max")) {
    node_->declare_parameter("actuator_velocity_max",
                             std::numeric_limits<double>::max());
  }
  node_->get_parameter("actuator_velocity_max", param_velocity_max_);

  cb_position_minmax_();
  cb_velocity_minmax_();

#ifdef NO_PARAMETER_EVENT_HANDLER
  node->add_on_set_parameters_callback(
      std::bind(&Implementation::cb_minmax_, this, std::placeholders::_1));
#else
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
  position_min_cb_handle_ = param_subscriber_->add_parameter_callback(
      "actuator_position_min",
      std::bind(&Implementation::cb_position_minmax_, this));
  position_max_cb_handle_ = param_subscriber_->add_parameter_callback(
      "actuator_position_max",
      std::bind(&Implementation::cb_position_minmax_, this));
  velocity_min_cb_handle_ = param_subscriber_->add_parameter_callback(
      "actuator_velocity_min",
      std::bind(&Implementation::cb_position_minmax_, this));
  velocity_max_cb_handle_ = param_subscriber_->add_parameter_callback(
      "actuator_velocity_max",
      std::bind(&Implementation::cb_position_minmax_, this));
#endif
}

#ifdef NO_PARAMETER_EVENT_HANDLER
rcl_interfaces::msg::SetParametersResult Implementation::cb_minmax_(
    const std::vector<rclcpp::Parameter> &parameters) {
  cb_position_minmax_();
  cb_velocity_minmax_();
}
#endif

void Implementation::cb_position_minmax_() {
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  position_min_ = param_position_min_.as_double();
  position_max_ = param_position_max_.as_double();

  double position_min_mod = std::fabs(position_min_);
  double position_max_mod = std::fabs(position_max_);
  position_mod_ =
      position_max_mod > position_min_mod ? position_max_mod : position_min_mod;
}

void Implementation::cb_velocity_minmax_() {
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  velocity_min_ = param_velocity_min_.as_double();
  velocity_max_ = param_velocity_max_.as_double();

  double velocity_min_mod = std::fabs(velocity_min_);
  double velocity_max_mod = std::fabs(velocity_max_);
  velocity_mod_ =
      velocity_max_mod > velocity_min_mod ? velocity_max_mod : velocity_min_mod;
}

void Implementation::init_actuator() {
  auto prefix = get_prefix_();

  if (has_position()) {
    topic_position_ = node_->create_publisher<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_POSITION, 10);

    subscription_position_ = node_->create_subscription<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_POSITION_SET, 10,
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
        prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY, 10);

    subscription_velocity_ = node_->create_subscription<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY_SET, 10,
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
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  if (position > position_max_) {
    position = position_max_;
  } else if (position < position_min_) {
    position = position_min_;
  }

  position_set_real_(position);
}

void Implementation::velocity_set(double velocity) {
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  if (velocity > velocity_max_) {
    velocity = velocity_max_;
  } else if (velocity < velocity_min_) {
    velocity = velocity_min_;
  }

  velocity_set_real_(velocity);
}

void Implementation::position_did_set_(double position) {
  std_msgs::msg::Float64 msg;
  msg.data = position;
  topic_position_->publish(msg);
}

void Implementation::velocity_did_set_(double velocity) {
  std_msgs::msg::Float64 msg;
  msg.data = velocity;
  topic_velocity_->publish(msg);
}

}  // namespace remote_actuator
