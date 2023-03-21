/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_actuator/fake_implementation.hpp"

#include <functional>

namespace remote_actuator {

FakeImplementation::FakeImplementation(rclcpp::Node *node)
    : Implementation(node) {
  init_actuator();
}

void FakeImplementation::position_set_real_(double position) {
  position_did_set_(position);
}

void FakeImplementation::velocity_set_real_(double velocity) {
  velocity_did_set_(velocity);
}

}  // namespace remote_actuator
