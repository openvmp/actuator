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
#include "std_msgs/msg/float64.hpp"

namespace remote_actuator {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node,
                 const std::string &default_actuator_prefix = "");
  virtual ~Implementation() {}

  void init_actuator();

  virtual void position_set(double) override final;
  virtual void velocity_set(double) override final;

 protected:
  rclcpp::Parameter param_position_min_;
  rclcpp::Parameter param_position_max_;
  rclcpp::Parameter param_velocity_min_;
  rclcpp::Parameter param_velocity_max_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> position_min_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> position_max_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> velocity_min_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> velocity_max_cb_handle_;
  void cb_position_minmax_(const rclcpp::Parameter &p);
  void cb_velocity_minmax_(const rclcpp::Parameter &p);
  double position_min_, position_max_;
  double velocity_min_, velocity_max_;
  double position_mod_, velocity_mod_;
  std::mutex param_maxmin_lock_;

  virtual void position_set_real_(double) = 0;
  virtual void velocity_set_real_(double) = 0;

  void position_did_set_(double);
  void velocity_did_set_(double);

 private:
  // topics
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr topic_position_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr topic_velocity_;

  // subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      subscription_position_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      subscription_velocity_;

  void sub_position_handler_(const std_msgs::msg::Float64::SharedPtr);
  void sub_velocity_handler_(const std_msgs::msg::Float64::SharedPtr);

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
