/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ACTUATOR_FAKE_NODE_H
#define OPENVMP_ACTUATOR_FAKE_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/fake_implementation.hpp"

namespace remote_actuator {

class FakeNode : public rclcpp::Node {
 public:
  FakeNode();

  std::shared_ptr<FakeImplementation> intf_;
};

}  // namespace remote_actuator

#endif  // OPENVMP_ACTUATOR_FAKE_NODE_H
