/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_ACTUATOR_INTERFACE_H
#define OPENVMP_REMOTE_ACTUATOR_INTERFACE_H

#include <memory>
#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/srv/position_set.hpp"
#include "remote_actuator/srv/velocity_set.hpp"

#define REMOTE_ACTUATOR_TOPIC_POSITION "/position"
#define REMOTE_ACTUATOR_TOPIC_VELOCITY "/velocity"
#define REMOTE_ACTUATOR_TOPIC_POSITION_SET "/set_position"
#define REMOTE_ACTUATOR_TOPIC_VELOCITY_SET "/set_velocity"
#define REMOTE_ACTUATOR_SERVICE_POSITION_SET "/set_position"
#define REMOTE_ACTUATOR_SERVICE_VELOCITY_SET "/set_velocity"

#if 1
#define REMOTE_ACTUATOR_USES_TOPICS
#endif

namespace remote_actuator {

class Interface {
 public:
  Interface(rclcpp::Node *node,
            const std::string &default_actuator_prefix = "");
  virtual ~Interface() {}

  /* has_position must return true if the driver supports position control */
  virtual bool has_position() { return false; }
  /* has_velocity must return true if the driver supports velocity control */
  virtual bool has_velocity() { return false; }
  virtual void position_set(double) {}
  virtual void velocity_set(double) {}

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string get_prefix_();

 private:
  rclcpp::Parameter interface_prefix_;
};

}  // namespace remote_actuator

#endif  // OPENVMP_REMOTE_ACTUATOR_INTERFACE_H
