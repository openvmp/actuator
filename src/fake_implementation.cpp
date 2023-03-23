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
  position *= 100.0;
  if (position > 0) {
    position = ::floor(position);
  } else {
    position = ::ceil(position);
  }
  position /= 100.0;
  position_did_set_(position);
}

void FakeImplementation::velocity_set_real_(double velocity) {
  velocity *= 100.0;
  if (velocity > 0) {
    velocity = ::floor(velocity);
  } else {
    velocity = ::ceil(velocity);
  }
  velocity /= 100.0;
  velocity_did_set_(velocity);
}

}  // namespace remote_actuator
