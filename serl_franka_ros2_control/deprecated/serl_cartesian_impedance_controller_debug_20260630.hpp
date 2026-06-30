#pragma once

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka/robot_state.h>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_ros2_control {

class SerlCartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  struct PoseReference {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool pose_msg_to_reference(const geometry_msgs::msg::PoseStamped& msg, PoseReference& pose) const;
  bool resolve_state_interface_indices();
  void update_joint_state_from_robot_state(const franka::RobotState& robot_state);
  void update_joint_state_from_interfaces();
  franka::RobotState* get_robot_state_ptr() const;
  bool read_measured_pose(PoseReference& pose) const;
  PoseReference limit_reference(const PoseReference& smoothed_target,
                                const PoseReference& measured);
  Eigen::Matrix<double, 6, 1> compute_cartesian_error(const PoseReference& measured,
                                                       const PoseReference& reference) const;
  void clip_cartesian_error(Eigen::Matrix<double, 6, 1>& error) const;
  Vector7d saturate_torque_rate(const Vector7d& tau_d_calculated,
                                const Vector7d& tau_reference);
  void publish_pose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
                    const rclcpp::Time& time,
                    const PoseReference& pose);
  void maybe_publish_debug(const rclcpp::Time& time);
  void publish_status(const rclcpp::Time& time);
  std::vector<std::string> joint_names() const;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr smoothed_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr limited_reference_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr measured_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  mutable std::mutex target_mutex_;
  PoseReference raw_target_;
  rclcpp::Time last_target_time_;
  bool target_received_{false};
  uint64_t target_update_count_{0};

  PoseReference smoothed_target_;
  PoseReference limited_reference_;
  PoseReference measured_pose_;
  bool smoothed_target_initialized_{false};
  bool active_{false};
  rclcpp::Time last_debug_publish_time_;

  std::string arm_id_{"fr3"};
  std::string target_topic_{"/serl_cartesian_impedance_controller/target_pose"};
  std::string reference_limit_mode_{"per_axis_error_clip"};
  std::vector<std::string> configured_joint_names_;
  bool use_robot_state_q_dq_{true};
  bool enable_nullspace_torque_{false};
  double translational_stiffness_{600.0};
  std::atomic<double> runtime_translational_stiffness_{600.0};
  double rotational_stiffness_{40.0};
  double translational_damping_{40.0};
  double rotational_damping_{8.0};
  double translational_ki_{0.0};
  double rotational_ki_{0.0};
  double max_pos_error_{0.002};
  double max_ori_error_{0.10};
  double filter_coeff_{0.005};
  double watchdog_timeout_sec_{0.25};
  double nullspace_stiffness_{10.0};
  double joint1_nullspace_stiffness_{10.0};
  double torque_rate_limit_{1.0};
  double debug_publish_rate_{50.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_damping_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_ki_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Vector3d translational_clip_min_{Eigen::Vector3d::Constant(-0.01)};
  Eigen::Vector3d translational_clip_max_{Eigen::Vector3d::Constant(0.01)};
  Eigen::Vector3d rotational_clip_min_{Eigen::Vector3d::Constant(-0.05)};
  Eigen::Vector3d rotational_clip_max_{Eigen::Vector3d::Constant(0.05)};
  Eigen::Matrix<double, 6, 1> error_i_{Eigen::Matrix<double, 6, 1>::Zero()};
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d q_d_nullspace_{Vector7d::Zero()};
  Vector7d last_tau_command_{Vector7d::Zero()};
  Eigen::Matrix<double, 6, 1> debug_cartesian_error_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> debug_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> debug_jacobian_velocity_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Vector3d debug_pose_diff_velocity_{Eigen::Vector3d::Zero()};
  Vector7d debug_tau_task_{Vector7d::Zero()};
  Vector7d debug_tau_nullspace_{Vector7d::Zero()};
  Vector7d debug_coriolis_{Vector7d::Zero()};
  Vector7d debug_tau_before_saturation_{Vector7d::Zero()};
  Vector7d debug_tau_after_saturation_{Vector7d::Zero()};
  Eigen::Matrix<double, 6, 1> debug_wrench_est_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> debug_wrench_est_error_{Eigen::Matrix<double, 6, 1>::Zero()};
  std::array<double, 16> debug_o_t_ee_{};
  std::array<double, 42> debug_zero_jacobian_{};
  Eigen::Vector3d previous_measured_position_{Eigen::Vector3d::Zero()};
  bool previous_measured_position_initialized_{false};
  std::array<size_t, 7> q_state_interface_indices_{};
  std::array<size_t, 7> dq_state_interface_indices_{};
  bool state_interface_indices_initialized_{false};

  double target_age_s_{std::numeric_limits<double>::quiet_NaN()};
  bool reference_was_clipped_{false};
  double position_error_before_clip_{std::numeric_limits<double>::quiet_NaN()};
  double position_error_after_clip_{std::numeric_limits<double>::quiet_NaN()};
  double orientation_error_before_clip_{std::numeric_limits<double>::quiet_NaN()};
  double orientation_error_after_clip_{std::numeric_limits<double>::quiet_NaN()};
  double tau_norm_{std::numeric_limits<double>::quiet_NaN()};
  bool tau_rate_limited_{false};
  double update_period_s_{std::numeric_limits<double>::quiet_NaN()};
  double debug_velocity_direction_cosine_{std::numeric_limits<double>::quiet_NaN()};
  double debug_velocity_norm_ratio_{std::numeric_limits<double>::quiet_NaN()};
  double debug_velocity_diff_norm_{std::numeric_limits<double>::quiet_NaN()};
  double debug_tau_task_nullspace_dot_{std::numeric_limits<double>::quiet_NaN()};
  double debug_wrench_est_error_norm_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace serl_franka_ros2_control
