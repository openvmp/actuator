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
    : Interface(node, default_actuator_prefix),
      has_position_{false},
      has_velocity_{false} {
  auto callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

RemoteInterface::~RemoteInterface() {
  RCLCPP_ERROR(node_->get_logger(),
               "remote_actuator::RemoteInterface::RemoteInterface(): "
               "Destroyed");
}

rclcpp::Client<remote_actuator::srv::PositionSet>::SharedPtr
RemoteInterface::get_clnt_position_set_() {
  if (!clnt_position_set_) {
    auto prefix = get_prefix_();

    RCLCPP_DEBUG(node_->get_logger(),
                 "remote_actuator::RemoteInterface::RemoteInterface(): "
                 "Connecting to the "
                 "remote interface: %s",
                 prefix.c_str());

#ifdef REMOTE_ACTUATOR_USES_TOPICS
    publisher_position_set_ = node_->create_publisher<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_POSITION_SET,
        rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT |
            rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    has_position_ = publisher_position_set_->get_subscription_count() > 0;
#else
    clnt_position_set_ =
        node_->create_client<remote_actuator::srv::PositionSet>(
            prefix + REMOTE_ACTUATOR_SERVICE_POSITION_SET,
            ::rmw_qos_profile_default, callback_group_);

    if (clnt_position_set_) {
      has_position_ =
          clnt_position_set_->wait_for_service(std::chrono::milliseconds(250));
    } else {
      has_position_ = false;
    }
#endif

    if (has_position_) {
      RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
                   prefix.c_str());
    }
  }
  return clnt_position_set_;
}

rclcpp::Client<remote_actuator::srv::VelocitySet>::SharedPtr
RemoteInterface::get_clnt_velocity_set_() {
  if (!clnt_velocity_set_) {
    auto prefix = get_prefix_();

#ifdef REMOTE_ACTUATOR_USES_TOPICS
    publisher_velocity_set_ = node_->create_publisher<std_msgs::msg::Float64>(
        prefix + REMOTE_ACTUATOR_TOPIC_VELOCITY_SET,
        rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT |
            rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    has_velocity_ = publisher_velocity_set_->get_subscription_count() > 0;
#else
    clnt_velocity_set_ =
        node_->create_client<remote_actuator::srv::VelocitySet>(
            prefix + REMOTE_ACTUATOR_SERVICE_VELOCITY_SET,
            ::rmw_qos_profile_default, callback_group_);

    if (clnt_velocity_set_) {
      has_velocity_ =
          clnt_velocity_set_->wait_for_service(std::chrono::milliseconds(250));
    } else {
      has_velocity_ = false;
    }
#endif
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

#ifdef REMOTE_ACTUATOR_USES_TOPICS
  std_msgs::msg::Float64 msg;
  msg.data = position;
  publisher_position_set_->publish(msg);
#else
  auto req = std::make_shared<remote_actuator::srv::PositionSet::Request>();
  auto resp = std::make_shared<remote_actuator::srv::PositionSet::Response>();

  req->position = position;

  auto f = clnt_position_set_->async_send_request(req);
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::position_set(): response received");
#endif
}

void RemoteInterface::velocity_set(double velocity) {
  if (!has_velocity()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "RemoteInterface::velocity_set(): not connected");
    return;
  }

#ifdef REMOTE_ACTUATOR_USES_TOPICS
  std_msgs::msg::Float64 msg;
  msg.data = velocity;
  publisher_velocity_set_->publish(msg);
#else
  auto req = std::make_shared<remote_actuator::srv::VelocitySet::Request>();
  auto resp = std::make_shared<remote_actuator::srv::VelocitySet::Response>();

  req->velocity = velocity;

  auto f = clnt_velocity_set_->async_send_request(req);
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::velocity_set(): response received");
#endif
}

}  // namespace remote_actuator
