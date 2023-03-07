/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_ACTUATOR_IMPLEMENTATION_H
#define OPENVMP_REMOTE_ACTUATOR_IMPLEMENTATION_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/interface.hpp"
#include "remote_actuator/srv/velocity_set.hpp"
#include "std_msgs/msg/float32.hpp"

namespace remote_actuator {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node);
  virtual ~Implementation() {}

  virtual void velocity_set(double) override final;
  virtual void position_set(double) override final;

 protected:
  virtual void position_set_real_(double) = 0;
  virtual void velocity_set_real_(double) = 0;

 private:
  // topics
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr topic_position_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr topic_velocity_;

  // services
  rclcpp::Service<remote_actuator::srv::PositionSet>::SharedPtr
      srv_position_set_;
  rclcpp::Service<remote_actuator::srv::VelocitySet>::SharedPtr
      srv_velocity_set_;

  rclcpp::FutureReturnCode position_set_handler_(
      const std::shared_ptr<remote_actuator::srv::PositionSet::Request> request,
      std::shared_ptr<remote_actuator::srv::PositionSet::Response> response);
  rclcpp::FutureReturnCode velocity_set_handler_(
      const std::shared_ptr<remote_actuator::srv::VelocitySet::Request> request,
      std::shared_ptr<remote_actuator::srv::VelocitySet::Response> response);
};

}  // namespace remote_actuator

#endif  // OPENVMP_REMOTE_ACTUATOR_IMPLEMENTATION_H
