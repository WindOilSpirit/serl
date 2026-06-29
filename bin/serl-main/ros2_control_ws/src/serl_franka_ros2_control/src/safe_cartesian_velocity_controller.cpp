#include "serl_franka_ros2_control/safe_cartesian_velocity_controller.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace serl_franka_ros2_control {

controller_interface::InterfaceConfiguration
SafeCartesianVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
SafeCartesianVelocityController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type SafeCartesianVelocityController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const double dt = std::clamp(period.seconds(), 0.0001, 0.01);

  if (!initialized_) {
    command_linear_velocity_.setZero();
    command_angular_velocity_.setZero();
    activation_time_ = time;
    initialized_ = true;
    return controller_interface::return_type::OK;
  }

  if ((time - activation_time_).seconds() < startup_hold_time_s_) {
    command_linear_velocity_.setZero();
    command_angular_velocity_.setZero();
    if (!franka_cartesian_velocity_->setCommand(command_linear_velocity_,
                                                command_angular_velocity_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to hold current Cartesian velocity.");
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  Eigen::Vector3d desired_linear = command_linear_velocity_;
  Eigen::Vector3d desired_angular = command_angular_velocity_;
  bool use_target = false;
  rclcpp::Time last_target_time;

  if (target_available_.load(std::memory_order_acquire)) {
    std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      last_target_time = last_target_time_;
      if (has_target_ && (time - last_target_time_).seconds() <= watchdog_timeout_sec_) {
        desired_linear = target_linear_velocity_;
        desired_angular = target_angular_velocity_;
        use_target = true;
      } else if (has_target_) {
        has_target_ = false;
        target_available_.store(false, std::memory_order_release);
      }
    }
  }

  if (!use_target) {
    command_linear_velocity_.setZero();
    command_angular_velocity_.setZero();
    if (!franka_cartesian_velocity_->setCommand(command_linear_velocity_,
                                                command_angular_velocity_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to hold current Cartesian velocity.");
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  command_linear_velocity_ =
      clamp_step(desired_linear, command_linear_velocity_, max_linear_acceleration_mps2_ * dt);
  command_angular_velocity_ =
      clamp_step(desired_angular, command_angular_velocity_, max_angular_acceleration_radps2_ * dt);

  const double linear_speed = command_linear_velocity_.norm();
  if (linear_speed > max_linear_speed_mps_ && linear_speed > 1e-12) {
    command_linear_velocity_ *= max_linear_speed_mps_ / linear_speed;
  }
  const double angular_speed = command_angular_velocity_.norm();
  if (angular_speed > max_angular_speed_radps_ && angular_speed > 1e-12) {
    command_angular_velocity_ *= max_angular_speed_radps_ / angular_speed;
  }

  if (!franka_cartesian_velocity_->setCommand(command_linear_velocity_,
                                              command_angular_velocity_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set Cartesian velocity command.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SafeCartesianVelocityController::on_init() {
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(false);

  auto_declare<std::string>("target_topic", target_topic_);
  auto_declare<std::string>("frame_id", frame_id_);
  auto_declare<double>("startup_hold_time_s", startup_hold_time_s_);
  auto_declare<double>("watchdog_timeout_sec", watchdog_timeout_sec_);
  auto_declare<double>("max_linear_speed_mps", max_linear_speed_mps_);
  auto_declare<double>("max_linear_acceleration_mps2", max_linear_acceleration_mps2_);
  auto_declare<double>("max_angular_speed_radps", max_angular_speed_radps_);
  auto_declare<double>("max_angular_acceleration_radps2", max_angular_acceleration_radps2_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafeCartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  startup_hold_time_s_ = get_node()->get_parameter("startup_hold_time_s").as_double();
  watchdog_timeout_sec_ = get_node()->get_parameter("watchdog_timeout_sec").as_double();
  max_linear_speed_mps_ = get_node()->get_parameter("max_linear_speed_mps").as_double();
  max_linear_acceleration_mps2_ =
      get_node()->get_parameter("max_linear_acceleration_mps2").as_double();
  max_angular_speed_radps_ = get_node()->get_parameter("max_angular_speed_radps").as_double();
  max_angular_acceleration_radps2_ =
      get_node()->get_parameter("max_angular_acceleration_radps2").as_double();

  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      target_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&SafeCartesianVelocityController::target_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafeCartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  command_linear_velocity_.setZero();
  command_angular_velocity_.setZero();
  std::lock_guard<std::mutex> lock(target_mutex_);
  has_target_ = false;
  target_available_.store(false, std::memory_order_release);
  initialized_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SafeCartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  has_target_ = false;
  target_available_.store(false, std::memory_order_release);
  initialized_ = false;
  return CallbackReturn::SUCCESS;
}

void SafeCartesianVelocityController::target_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  if (!twist_to_velocity(*msg, linear, angular)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Rejecting non-finite target twist.");
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  target_linear_velocity_ = linear;
  target_angular_velocity_ = angular;
  last_target_time_ = get_node()->now();
  has_target_ = true;
  target_available_.store(true, std::memory_order_release);
}

bool SafeCartesianVelocityController::twist_to_velocity(const geometry_msgs::msg::TwistStamped& msg,
                                                         Eigen::Vector3d& linear,
                                                         Eigen::Vector3d& angular) const {
  (void)msg.header.frame_id;
  linear = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
  angular = Eigen::Vector3d(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
  if (!std::isfinite(linear.x()) || !std::isfinite(linear.y()) || !std::isfinite(linear.z()) ||
      !std::isfinite(angular.x()) || !std::isfinite(angular.y()) || !std::isfinite(angular.z())) {
    return false;
  }
  return true;
}

Eigen::Vector3d SafeCartesianVelocityController::clamp_step(const Eigen::Vector3d& desired,
                                                            const Eigen::Vector3d& current,
                                                            double max_step) const {
  Eigen::Vector3d delta = desired - current;
  const double norm = delta.norm();
  if (norm > max_step && norm > 1e-12) {
    delta *= max_step / norm;
  }
  return current + delta;
}

}  // namespace serl_franka_ros2_control

PLUGINLIB_EXPORT_CLASS(serl_franka_ros2_control::SafeCartesianVelocityController,
                       controller_interface::ControllerInterface)
