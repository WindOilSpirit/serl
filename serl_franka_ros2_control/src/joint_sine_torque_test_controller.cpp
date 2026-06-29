#include "serl_franka_ros2_control/joint_sine_torque_test_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

double quiet_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace serl_franka_ros2_control {

controller_interface::InterfaceConfiguration
JointSineTorqueTestController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names()) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointSineTorqueTestController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names()) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::return_type JointSineTorqueTestController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (!active_) {
    return controller_interface::return_type::OK;
  }

  const double elapsed_s = (time - start_time_).seconds();
  const int joint_index = std::clamp(joint_number_, 1, 7) - 1;
  const double command_elapsed_s = elapsed_s - start_delay_s_;
  const bool command_enabled = command_elapsed_s >= 0.0 && command_elapsed_s <= duration_s_;
  const double tau_command =
      command_enabled ? amplitude_nm_ * std::sin(2.0 * kPi * frequency_hz_ * command_elapsed_s)
                      : 0.0;

  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_.at(i).set_value(static_cast<int>(i) == joint_index ? tau_command : 0.0);
  }
  publish_status(time, elapsed_s, tau_command, period);
  return controller_interface::return_type::OK;
}

CallbackReturn JointSineTorqueTestController::on_init() {
  try {
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<int>("joint_number", joint_number_);
    auto_declare<double>("amplitude_nm", amplitude_nm_);
    auto_declare<double>("frequency_hz", frequency_hz_);
    auto_declare<double>("start_delay_s", start_delay_s_);
    auto_declare<double>("duration_s", duration_s_);
    auto_declare<double>("debug_publish_rate", debug_publish_rate_);
  } catch (const std::exception& ex) {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", ex.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSineTorqueTestController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  configured_joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  joint_number_ = static_cast<int>(get_node()->get_parameter("joint_number").as_int());
  amplitude_nm_ = get_node()->get_parameter("amplitude_nm").as_double();
  frequency_hz_ = get_node()->get_parameter("frequency_hz").as_double();
  start_delay_s_ = get_node()->get_parameter("start_delay_s").as_double();
  duration_s_ = get_node()->get_parameter("duration_s").as_double();
  debug_publish_rate_ = get_node()->get_parameter("debug_publish_rate").as_double();

  if (!configured_joint_names_.empty() && configured_joint_names_.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names must contain exactly 7 names.");
    return CallbackReturn::FAILURE;
  }
  if (joint_number_ < 1 || joint_number_ > 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_number must be in [1, 7].");
    return CallbackReturn::FAILURE;
  }
  if (amplitude_nm_ < 0.0 || amplitude_nm_ > 0.2) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "amplitude_nm must be in [0, 0.2] for this guarded debug controller.");
    return CallbackReturn::FAILURE;
  }
  if (frequency_hz_ <= 0.0 || frequency_hz_ > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "frequency_hz must be in (0, 1.0] for this guarded debug controller.");
    return CallbackReturn::FAILURE;
  }
  if (duration_s_ <= 0.0 || duration_s_ > 5.0) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "duration_s must be in (0, 5.0] for this guarded debug controller.");
    return CallbackReturn::FAILURE;
  }
  if (start_delay_s_ < 0.0 || start_delay_s_ > 10.0) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "start_delay_s must be in [0, 10.0] for this debug controller.");
    return CallbackReturn::FAILURE;
  }

  status_pub_ = get_node()->create_publisher<std_msgs::msg::String>("~/debug/status", 10);
  RCLCPP_INFO(get_node()->get_logger(),
              "Configured joint sine torque test: joint_number=%d amplitude=%.6f Nm "
              "frequency=%.6f Hz start_delay=%.6f s duration=%.6f s",
              joint_number_, amplitude_nm_, frequency_hz_, start_delay_s_, duration_s_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSineTorqueTestController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (command_interfaces_.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "Expected 7 effort command interfaces, got %zu.",
                 command_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  if (!resolve_state_interface_indices()) {
    return CallbackReturn::ERROR;
  }

  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  start_time_ = get_node()->now();
  last_debug_publish_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  active_ = true;

  std::ostringstream command_log;
  command_log << "joint sine claimed command interfaces:";
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_log << "\n  [" << i << "] name=" << command_interfaces_.at(i).get_name()
                << " interface=" << command_interfaces_.at(i).get_interface_name();
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", command_log.str().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSineTorqueTestController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  state_interface_indices_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

bool JointSineTorqueTestController::resolve_state_interface_indices() {
  const auto resolved_joint_names = joint_names();

  auto find_interface_index = [&](const std::string& joint_name,
                                  const std::string& interface_name) -> size_t {
    const std::string full_name = joint_name + "/" + interface_name;
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      const auto& interface = state_interfaces_.at(i);
      if (interface.get_name() == full_name ||
          (interface.get_name() == joint_name &&
           interface.get_interface_name() == interface_name)) {
        return i;
      }
    }
    throw std::runtime_error("missing state interface " + full_name);
  };

  try {
    for (size_t i = 0; i < resolved_joint_names.size(); ++i) {
      q_state_interface_indices_.at(i) =
          find_interface_index(resolved_joint_names.at(i), hardware_interface::HW_IF_POSITION);
      dq_state_interface_indices_.at(i) =
          find_interface_index(resolved_joint_names.at(i), hardware_interface::HW_IF_VELOCITY);
    }
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(get_node()->get_logger(), "Cannot resolve joint state interface order: %s",
                 ex.what());
    state_interface_indices_initialized_ = false;
    return false;
  }
  state_interface_indices_initialized_ = true;
  return true;
}

void JointSineTorqueTestController::publish_status(const rclcpp::Time& time,
                                                   double elapsed_s,
                                                   double tau_command,
                                                   const rclcpp::Duration& period) {
  if (!status_pub_ || debug_publish_rate_ <= 0.0) {
    return;
  }
  const double publish_period_s = 1.0 / debug_publish_rate_;
  if (last_debug_publish_time_.nanoseconds() > 0 &&
      (time - last_debug_publish_time_).seconds() < publish_period_s) {
    return;
  }
  last_debug_publish_time_ = time;

  const int joint_index = std::clamp(joint_number_, 1, 7) - 1;
  const double q = state_interface_indices_initialized_
                       ? state_interfaces_.at(q_state_interface_indices_.at(joint_index)).get_value()
                       : quiet_nan();
  const double dq =
      state_interface_indices_initialized_
          ? state_interfaces_.at(dq_state_interface_indices_.at(joint_index)).get_value()
          : quiet_nan();

  std_msgs::msg::String msg;
  std::ostringstream out;
  out << "controller_active=" << static_cast<int>(active_)
      << " t=" << time.seconds()
      << " elapsed_s=" << elapsed_s
      << " dt=" << period.seconds()
      << " joint_number=" << joint_number_
      << " amplitude_nm=" << amplitude_nm_
      << " frequency_hz=" << frequency_hz_
      << " start_delay_s=" << start_delay_s_
      << " duration_s=" << duration_s_
      << " q_" << joint_number_ << "=" << q
      << " dq_" << joint_number_ << "=" << dq
      << " tau_command_" << joint_number_ << "=" << tau_command
      << " command_finished=" << static_cast<int>(elapsed_s > start_delay_s_ + duration_s_);
  msg.data = out.str();
  status_pub_->publish(msg);
}

std::vector<std::string> JointSineTorqueTestController::joint_names() const {
  if (get_node()->has_parameter("joint_names")) {
    const auto names = get_node()->get_parameter("joint_names").as_string_array();
    if (names.size() == 7) {
      return names;
    }
  }
  std::vector<std::string> names;
  names.reserve(7);
  const std::string arm_id =
      get_node()->has_parameter("arm_id") ? get_node()->get_parameter("arm_id").as_string()
                                          : arm_id_;
  for (int i = 1; i <= 7; ++i) {
    names.push_back(arm_id + "_joint" + std::to_string(i));
  }
  return names;
}

}  // namespace serl_franka_ros2_control

PLUGINLIB_EXPORT_CLASS(serl_franka_ros2_control::JointSineTorqueTestController,
                       controller_interface::ControllerInterface)
