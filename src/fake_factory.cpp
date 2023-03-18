/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/fake_factory.hpp"

#include <exception>

#include "remote_actuator/fake_implementation.hpp"
#include "remote_actuator/interface_remote.hpp"

namespace remote_actuator {

std::shared_ptr<Interface> FakeFactory::New(
    rclcpp::Node *node, const std::string &default_actuator_prefix) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  if (!node->has_parameter("actuator_is_remote")) {
    node->declare_parameter("actuator_is_remote", use_remote.as_bool());
  }
  node->get_parameter("actuator_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node, default_actuator_prefix);
  } else {
    return std::make_shared<FakeImplementation>(node);
  }
}

}  // namespace remote_actuator
