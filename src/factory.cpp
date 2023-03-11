/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/factory.hpp"

#include <exception>

#include "remote_actuator/interface_remote.hpp"

namespace remote_actuator {

std::shared_ptr<Interface> Factory::New(
    rclcpp::Node *node, const std::string &default_actuator_prefix) {
  rclcpp::Parameter is_remote;
  if (!node->has_parameter("actuator_is_remote")) {
    node->declare_parameter("actuator_is_remote", true);
  }
  node->get_parameter("actuator_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node, default_actuator_prefix);
  } else {
    throw std::invalid_argument(
        "Link with the actual driver or set actuator_is_remote");
  }
}

}  // namespace remote_actuator
