/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/interface.hpp"

#include <functional>

namespace remote_actuator {

Interface::Interface(rclcpp::Node *node,
                     const std::string &default_actuator_prefix)
    : node_{node} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto actuator_prefix = default_actuator_prefix;
  if (actuator_prefix == "") {
    actuator_prefix = "/actuator/" + std::string(node_->get_name());
  }

  int index = 0;
  std::string parameter_name;
  do {
    parameter_name = "actuator_prefix";
    if (index++ != 0) {
      parameter_name += "_" + std::to_string(index);
    }
  } while (node->has_parameter(parameter_name));

  node->declare_parameter(parameter_name, actuator_prefix);
  node->get_parameter(parameter_name, interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace remote_actuator
