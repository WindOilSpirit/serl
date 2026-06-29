#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_ros2_control {

class JointSineTorqueTestController : public controller_interface::ControllerInterface {
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
  std::vector<std::string> joint_names() const;
  bool resolve_state_interface_indices();
  void publish_status(const rclcpp::Time& time,
                      double elapsed_s,
                      double tau_command,
                      const rclcpp::Duration& period);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::string arm_id_{"fr3"};
  std::vector<std::string> configured_joint_names_;
  int joint_number_{4};
  double amplitude_nm_{0.05};
  double frequency_hz_{0.2};
  double start_delay_s_{2.0};
  double duration_s_{3.0};
  double debug_publish_rate_{100.0};

  rclcpp::Time start_time_;
  rclcpp::Time last_debug_publish_time_;
  bool active_{false};
  bool state_interface_indices_initialized_{false};
  std::array<size_t, 7> q_state_interface_indices_{};
  std::array<size_t, 7> dq_state_interface_indices_{};
};

}  // namespace serl_franka_ros2_control
