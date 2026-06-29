#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_ros2_control {

class SafeCartesianVelocityController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void target_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  bool twist_to_velocity(const geometry_msgs::msg::TwistStamped& msg,
                         Eigen::Vector3d& linear,
                         Eigen::Vector3d& angular) const;
  Eigen::Vector3d clamp_step(const Eigen::Vector3d& desired,
                             const Eigen::Vector3d& current,
                             double max_step) const;

  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_sub_;

  mutable std::mutex target_mutex_;
  Eigen::Vector3d target_linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_angular_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d command_linear_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d command_angular_velocity_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_target_time_;
  rclcpp::Time activation_time_;
  bool has_target_{false};
  std::atomic_bool target_available_{false};
  bool initialized_{false};

  std::string target_topic_{"~/target_twist"};
  std::string frame_id_{"fr3_link0"};
  double startup_hold_time_s_{0.4};
  double watchdog_timeout_sec_{0.25};
  double max_linear_speed_mps_{0.01};
  double max_linear_acceleration_mps2_{0.03};
  double max_angular_speed_radps_{0.5};
  double max_angular_acceleration_radps2_{1.0};
};

}  // namespace serl_franka_ros2_control
