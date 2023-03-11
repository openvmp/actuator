/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/fake_node.hpp"

namespace remote_actuator {

FakeNode::FakeNode() : rclcpp::Node("actuator_fake") {
  intf_ = std::make_shared<FakeImplementation>(this);
}

}  // namespace remote_actuator
