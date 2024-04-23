#include "robot_body_filter/utils/time_utils.hpp"

namespace robot_body_filter {

rclcpp::Duration remainingTime(const rclcpp::Time &query, const double timeout)
{
  // rclcpp::Time::waitForValid(rclcpp::WallDuration().fromSec(timeout));
  // if (!rclcpp::Time::isValid()) {
  //   ROS_ERROR("ROS time is not yet initialized");
  //   return rclcpp::Duration(0);
  // }

  // const auto passed = (rclcpp::Time::now() - query).toSec();
  // return rclcpp::Duration(std::max(0.0, timeout - passed));
}

rclcpp::Duration remainingTime(const rclcpp::Time &query,
                            const rclcpp::Duration &timeout)
{
  // rclcpp::Time::waitForValid(rclcpp::WallDuration(timeout.sec, timeout.nsec));
  // if (!rclcpp::Time::isValid()) {
  //   // ROS_ERROR("ROS time is not yet initialized");
  //   return rclcpp::Duration(0);
  // }

  // const auto passed = rclcpp::Time::now() - query;
  // const auto remaining = timeout - passed;
  // return (remaining.sec >= 0) ? remaining : rclcpp::Duration(0);
}

};