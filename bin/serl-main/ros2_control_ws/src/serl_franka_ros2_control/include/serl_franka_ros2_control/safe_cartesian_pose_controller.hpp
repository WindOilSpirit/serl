#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_ros2_control {

class SafeCartesianPoseController : public controller_interface::ControllerInterface {
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
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void hold_current_service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void clear_target_service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void enable_targets_service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool pose_to_eigen(const geometry_msgs::msg::PoseStamped& msg,
                     Eigen::Vector3d& position,
                     Eigen::Quaterniond& orientation) const;
  bool read_measured_current_pose(Eigen::Quaterniond& orientation,
                                  Eigen::Vector3d& position,
                                  const char* context) const;
  bool read_command_interface_pose(Eigen::Quaterniond& orientation,
                                   Eigen::Vector3d& position,
                                   const char* context) const;
  void clear_all_targets();
  bool clear_all_targets_and_hold_current(const char* context);
  bool hold_continuous_command_pose(const char* context);
  void publish_debug_pose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
                          const rclcpp::Time& time,
                          const Eigen::Vector3d& position,
                          const Eigen::Quaterniond& orientation);
  void maybe_publish_debug_poses(const rclcpp::Time& time);
  void reject_target(const char* reason);
  void publish_target_status(const rclcpp::Time& time);
  void update_tracking_debug(const Eigen::Vector3d& desired_position,
                             const Eigen::Vector3d& command_position);
  Eigen::Vector3d limit_translation_step(const Eigen::Vector3d& desired,
                                         const Eigen::Vector3d& current,
                                         double dt);
  Eigen::Quaterniond limit_rotation_step(const Eigen::Quaterniond& desired,
                                         const Eigen::Quaterniond& current) const;

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface>
      franka_cartesian_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_received_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_accepted_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_rt_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_internal_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_target_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_current_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_target_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_targets_service_;

  mutable std::mutex target_mutex_;
  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d rt_target_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rt_target_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d command_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond command_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d command_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d command_acceleration_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_target_time_;
  rclcpp::Time rt_last_target_time_;
  bool has_target_{false};
  bool rt_has_target_{false};
  std::atomic_bool target_available_{false};
  bool target_stream_primed_{false};
  bool initialized_{false};
  std::atomic_bool accept_targets_{false};
  bool first_target_after_enable_{true};
  uint64_t target_accepted_count_{0};
  uint64_t target_rejected_count_{0};
  std::string last_target_reject_reason_{"NONE"};
  double initial_robot_time_{0.0};
  rclcpp::Time activation_time_;

  std::string arm_id_{"fr3"};
  std::string target_topic_{"~/target_pose"};
  bool use_state_interfaces_{true};
  double startup_hold_time_s_{1.0};
  double watchdog_timeout_sec_{0.25};
  double max_translation_step_m_{0.001};
  double max_translation_speed_mps_{0.02};
  double max_translation_acceleration_mps2_{0.02};
  double max_translation_jerk_mps3_{0.08};
  double translation_tracking_time_s_{0.4};
  double max_rotation_step_rad_{0.01};
  double max_target_distance_m_{0.05};
  double activation_hold_tolerance_m_{0.0001};
  double first_target_tolerance_m_{0.0001};
  double command_measured_tracking_tolerance_m_{0.0005};
  double debug_publish_period_s_{0.10};
  double activation_command_to_measured_norm_m_{std::numeric_limits<double>::quiet_NaN()};
  double seeded_command_to_measured_norm_m_{std::numeric_limits<double>::quiet_NaN()};
  std::string command_seed_source_{"NONE"};
  double last_target_to_command_error_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_target_to_measured_error_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_command_to_measured_error_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_update_period_s_{std::numeric_limits<double>::quiet_NaN()};
  uint64_t update_overrun_count_{0};
  rclcpp::Time last_debug_publish_time_;

  Eigen::Vector3d raw_target_to_command_error_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double raw_target_to_command_error_norm_{std::numeric_limits<double>::quiet_NaN()};
  bool target_distance_clamped_{false};
  Eigen::Vector3d desired_position_before_guard_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d desired_position_after_guard_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};

  Eigen::Vector3d debug_desired_velocity_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double debug_desired_velocity_norm_{std::numeric_limits<double>::quiet_NaN()};
  bool debug_desired_speed_limited_{false};
  Eigen::Vector3d debug_desired_acceleration_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double debug_desired_acceleration_norm_{std::numeric_limits<double>::quiet_NaN()};
  bool debug_desired_acceleration_limited_{false};
  double debug_acceleration_delta_norm_{std::numeric_limits<double>::quiet_NaN()};
  bool debug_jerk_limited_{false};
  Eigen::Vector3d debug_command_velocity_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double debug_command_velocity_norm_{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d debug_command_acceleration_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double debug_command_acceleration_norm_{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d debug_step_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double debug_step_norm_{std::numeric_limits<double>::quiet_NaN()};
  bool debug_step_limited_{false};

  Eigen::Vector3d debug_command_measured_error_{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double debug_command_measured_error_norm_{std::numeric_limits<double>::quiet_NaN()};
  double debug_target_measured_error_norm_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace serl_franka_ros2_control
