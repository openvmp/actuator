/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ACTUATOR_FAKE_IMPLEMENTATION_H
#define OPENVMP_ACTUATOR_FAKE_IMPLEMENTATION_H

#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/implementation.hpp"

namespace remote_actuator {

class FakeImplementation : public Implementation {
 public:
  FakeImplementation(rclcpp::Node *node);
  virtual ~FakeImplementation() = default;

 protected:
  virtual bool has_position() override { return true; }
  virtual bool has_velocity() override { return true; }
  virtual void position_set_real_(double) override;
  virtual void velocity_set_real_(double) override;
};

}  // namespace remote_actuator

#endif  // OPENVMP_ACTUATOR_FAKE_IMPLEMENTATION_H
