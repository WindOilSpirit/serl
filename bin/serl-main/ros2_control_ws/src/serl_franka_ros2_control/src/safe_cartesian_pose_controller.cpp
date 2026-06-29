#include "serl_franka_ros2_control/safe_cartesian_pose_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include <pluginlib/class_list_macros.hpp>

namespace serl_franka_ros2_control {

controller_interface::InterfaceConfiguration
SafeCartesianPoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
SafeCartesianPoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  bool use_state_interfaces = use_state_interfaces_;
  if (get_node()->has_parameter("use_state_interfaces")) {
    use_state_interfaces = get_node()->get_parameter("use_state_interfaces").as_bool();
  }
  if (use_state_interfaces) {
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = franka_cartesian_pose_->get_state_interface_names();
    config.names.push_back(arm_id_ + "/robot_time");
  } else {
    config.type = controller_interface::interface_configuration_type::NONE;
  }
  return config;
}

controller_interface::return_type SafeCartesianPoseController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const double dt = std::clamp(period.seconds(), 0.0001, 0.01);
  last_update_period_s_ = period.seconds();
  if (last_update_period_s_ > 0.003) {
    update_overrun_count_++;
  }
  if (!initialized_) {
    Eigen::Quaterniond measured_orientation;
    Eigen::Vector3d measured_position;
    if (!read_measured_current_pose(measured_orientation, measured_position, "first update")) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Cannot initialize Cartesian pose command without measured current pose.");
      return controller_interface::return_type::ERROR;
    }
    Eigen::Quaterniond seeded_command_orientation;
    Eigen::Vector3d seeded_command_position;
    if (!read_command_interface_pose(seeded_command_orientation, seeded_command_position,
                                     "first update")) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Cannot initialize Cartesian pose command without seeded command pose.");
      return controller_interface::return_type::ERROR;
    }
    const double seed_error = (seeded_command_position - measured_position).norm();
    RCLCPP_INFO(get_node()->get_logger(),
                "First Cartesian hold seed check: measured=(%.9f, %.9f, %.9f), "
                "seeded_command=(%.9f, %.9f, %.9f), error=%.9f m.",
                measured_position.x(), measured_position.y(), measured_position.z(),
                seeded_command_position.x(), seeded_command_position.y(),
                seeded_command_position.z(), seed_error);
    if (seed_error > activation_hold_tolerance_m_) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Refusing first Cartesian hold: seeded command-measured distance %.9f m "
                   "exceeds %.9f m.",
                   seed_error, activation_hold_tolerance_m_);
      return controller_interface::return_type::ERROR;
    }
    (void)seeded_command_orientation;
    command_orientation_ = measured_orientation;
    command_position_ = measured_position;
    command_velocity_.setZero();
    command_acceleration_.setZero();
    initial_robot_time_ = state_interfaces_.back().get_value();
    activation_time_ = time;
    initialized_ = true;
    seeded_command_to_measured_norm_m_ = seed_error;
    activation_command_to_measured_norm_m_ = (command_position_ - measured_position).norm();
    command_seed_source_ = "measured_pose_after_seed_sanity_check_first_update";
    if (!franka_cartesian_pose_->setCommand(command_orientation_, command_position_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to initialize Cartesian pose hold command.");
      return controller_interface::return_type::ERROR;
    }
    Eigen::Quaterniond verify_orientation;
    Eigen::Vector3d verify_position;
    if (!read_measured_current_pose(verify_orientation, verify_position, "first update verify") ||
        (command_position_ - verify_position).norm() > activation_hold_tolerance_m_) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Initial Cartesian command is not aligned with measured current pose.");
      return controller_interface::return_type::ERROR;
    }
    maybe_publish_debug_poses(time);
    return controller_interface::return_type::OK;
  }

  if ((time - activation_time_).seconds() < startup_hold_time_s_) {
    target_stream_primed_ = false;
    if (!hold_continuous_command_pose("startup hold")) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Failed to hold continuous internal Cartesian command pose.");
      return controller_interface::return_type::ERROR;
    }
    maybe_publish_debug_poses(time);
    return controller_interface::return_type::OK;
  }

  Eigen::Vector3d desired_position = command_position_;
  bool use_target = false;

  if (target_available_.load(std::memory_order_acquire)) {
    std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      if (has_target_) {
        rt_target_position_ = target_position_;
        rt_target_orientation_ = target_orientation_;
        rt_last_target_time_ = last_target_time_;
        rt_has_target_ = true;
      }
    }
  }

  const bool target_fresh =
      rt_has_target_ && watchdog_timeout_sec_ > 0.0 &&
      rt_last_target_time_.nanoseconds() > 0 &&
      (time - rt_last_target_time_).seconds() <= watchdog_timeout_sec_;

  if (target_fresh) {
    desired_position = rt_target_position_;
    use_target = true;
  }

  if (!use_target && !target_stream_primed_) {
    if (!clear_all_targets_and_hold_current("no target before priming")) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Failed to hold continuous internal Cartesian command pose.");
      return controller_interface::return_type::ERROR;
    }
    maybe_publish_debug_poses(time);
    return controller_interface::return_type::OK;
  }

  if (!use_target) {
    accept_targets_.store(false, std::memory_order_release);
    if (!clear_all_targets_and_hold_current("target stale")) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Failed to hold continuous internal Cartesian command pose.");
      return controller_interface::return_type::ERROR;
    }
    target_stream_primed_ = false;
    maybe_publish_debug_poses(time);
    return controller_interface::return_type::OK;
  }

  if (use_target && !target_stream_primed_) {
    target_stream_primed_ = true;
    command_velocity_.setZero();
    command_acceleration_.setZero();
    if (!franka_cartesian_pose_->setCommand(command_orientation_, command_position_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to prime Cartesian pose command.");
      return controller_interface::return_type::ERROR;
    }
    maybe_publish_debug_poses(time);
    return controller_interface::return_type::OK;
  }

  desired_position_before_guard_ = desired_position;
  raw_target_to_command_error_ = desired_position - command_position_;
  raw_target_to_command_error_norm_ = raw_target_to_command_error_.norm();
  target_distance_clamped_ =
      use_target && raw_target_to_command_error_norm_ > max_target_distance_m_;

  if (target_distance_clamped_) {
    desired_position = command_position_;
  }
  desired_position_after_guard_ = desired_position;

  const auto command_position = limit_translation_step(desired_position, command_position_, dt);
  const auto command_orientation = command_orientation_;
  update_tracking_debug(desired_position, command_position);

  if (!franka_cartesian_pose_->setCommand(command_orientation, command_position)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set Cartesian pose command.");
    return controller_interface::return_type::ERROR;
  }

  command_position_ = command_position;
  command_orientation_ = command_orientation;
  maybe_publish_debug_poses(time);
  return controller_interface::return_type::OK;
}

CallbackReturn SafeCartesianPoseController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);

  auto_declare<std::string>("target_topic", target_topic_);
  auto_declare<std::string>("arm_id", arm_id_);
  auto_declare<bool>("use_state_interfaces", use_state_interfaces_);
  auto_declare<double>("startup_hold_time_s", startup_hold_time_s_);
  auto_declare<double>("watchdog_timeout_sec", watchdog_timeout_sec_);
  auto_declare<double>("max_translation_step_m", max_translation_step_m_);
  auto_declare<double>("max_translation_speed_mps", max_translation_speed_mps_);
  auto_declare<double>("max_translation_acceleration_mps2", max_translation_acceleration_mps2_);
  auto_declare<double>("max_translation_jerk_mps3", max_translation_jerk_mps3_);
  auto_declare<double>("translation_tracking_time_s", translation_tracking_time_s_);
  auto_declare<double>("max_rotation_step_rad", max_rotation_step_rad_);
  auto_declare<double>("max_target_distance_m", max_target_distance_m_);
  auto_declare<double>("activation_hold_tolerance_m", activation_hold_tolerance_m_);
  auto_declare<double>("first_target_tolerance_m", first_target_tolerance_m_);
  auto_declare<double>("command_measured_tracking_tolerance_m",
                       command_measured_tracking_tolerance_m_);
  auto_declare<double>("debug_publish_period_s", debug_publish_period_s_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafeCartesianPoseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  use_state_interfaces_ = get_node()->get_parameter("use_state_interfaces").as_bool();
  startup_hold_time_s_ = get_node()->get_parameter("startup_hold_time_s").as_double();
  watchdog_timeout_sec_ = get_node()->get_parameter("watchdog_timeout_sec").as_double();
  max_translation_step_m_ = get_node()->get_parameter("max_translation_step_m").as_double();
  max_translation_speed_mps_ = get_node()->get_parameter("max_translation_speed_mps").as_double();
  max_translation_acceleration_mps2_ =
      get_node()->get_parameter("max_translation_acceleration_mps2").as_double();
  max_translation_jerk_mps3_ = get_node()->get_parameter("max_translation_jerk_mps3").as_double();
  translation_tracking_time_s_ =
      get_node()->get_parameter("translation_tracking_time_s").as_double();
  max_rotation_step_rad_ = get_node()->get_parameter("max_rotation_step_rad").as_double();
  max_target_distance_m_ = get_node()->get_parameter("max_target_distance_m").as_double();
  activation_hold_tolerance_m_ =
      get_node()->get_parameter("activation_hold_tolerance_m").as_double();
  first_target_tolerance_m_ = get_node()->get_parameter("first_target_tolerance_m").as_double();
  command_measured_tracking_tolerance_m_ =
      get_node()->get_parameter("command_measured_tracking_tolerance_m").as_double();
  debug_publish_period_s_ = get_node()->get_parameter("debug_publish_period_s").as_double();

  auto target_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, target_qos,
      std::bind(&SafeCartesianPoseController::target_callback, this, std::placeholders::_1));
  debug_received_target_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/received_target_pose",
                                                                    10);
  debug_accepted_target_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/accepted_target_pose",
                                                                    10);
  debug_rt_target_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/rt_target_pose", 10);
  debug_internal_command_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          "~/debug/internal_command_pose", 10);
  debug_target_status_pub_ =
      get_node()->create_publisher<std_msgs::msg::String>("~/debug/target_status", 10);
  hold_current_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/hold_current",
      std::bind(&SafeCartesianPoseController::hold_current_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  clear_target_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/clear_target",
      std::bind(&SafeCartesianPoseController::clear_target_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  enable_targets_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/enable_targets",
      std::bind(&SafeCartesianPoseController::enable_targets_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

CallbackReturn SafeCartesianPoseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  if (!use_state_interfaces_) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Refusing to activate SafeCartesianPoseController with "
                 "use_state_interfaces=false. Real measured Cartesian pose is required.");
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  if (state_interfaces_.size() < franka_cartesian_pose_->get_state_interface_names().size() + 1) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Refusing to activate: missing Cartesian pose state interfaces.");
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  Eigen::Quaterniond measured_orientation;
  Eigen::Vector3d measured_position;
  if (!read_measured_current_pose(measured_orientation, measured_position, "on_activate")) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Refusing to activate: measured Cartesian pose is unavailable or invalid.");
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  Eigen::Quaterniond seeded_command_orientation;
  Eigen::Vector3d seeded_command_position;
  if (!read_command_interface_pose(seeded_command_orientation, seeded_command_position,
                                   "on_activate")) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Refusing to activate: seeded Cartesian command interface pose is invalid.");
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  const double seed_error = (seeded_command_position - measured_position).norm();
  RCLCPP_INFO(get_node()->get_logger(),
              "Activation Cartesian seed check: measured=(%.9f, %.9f, %.9f), "
              "seeded_command=(%.9f, %.9f, %.9f), error=%.9f m.",
              measured_position.x(), measured_position.y(), measured_position.z(),
              seeded_command_position.x(), seeded_command_position.y(),
              seeded_command_position.z(), seed_error);
  if (seed_error > activation_hold_tolerance_m_) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Refusing to activate: seeded command-measured distance %.9f m exceeds %.9f m.",
                 seed_error, activation_hold_tolerance_m_);
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  (void)seeded_command_orientation;
  command_position_ = measured_position;
  command_orientation_ = measured_orientation;
  command_velocity_.setZero();
  command_acceleration_.setZero();
  initial_robot_time_ = state_interfaces_.back().get_value();
  activation_time_ = get_node()->now();
  last_debug_publish_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  clear_all_targets();
  accept_targets_.store(false, std::memory_order_release);
  first_target_after_enable_ = true;
  target_accepted_count_ = 0;
  target_rejected_count_ = 0;
  last_target_reject_reason_ = "NONE";
  last_target_to_command_error_m_ = std::numeric_limits<double>::quiet_NaN();
  last_target_to_measured_error_m_ = std::numeric_limits<double>::quiet_NaN();
  last_command_to_measured_error_m_ = std::numeric_limits<double>::quiet_NaN();
  seeded_command_to_measured_norm_m_ = seed_error;
  activation_command_to_measured_norm_m_ = (command_position_ - measured_position).norm();
  command_seed_source_ = "measured_pose_after_seed_sanity_check";
  last_update_period_s_ = std::numeric_limits<double>::quiet_NaN();
  update_overrun_count_ = 0;
  initialized_ = true;
  if (!franka_cartesian_pose_->setCommand(command_orientation_, command_position_)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Failed to initialize Cartesian command from measured current pose.");
    franka_cartesian_pose_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SafeCartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  has_target_ = false;
  rt_has_target_ = false;
  target_stream_primed_ = false;
  target_available_.store(false, std::memory_order_release);
  accept_targets_.store(false, std::memory_order_release);
  first_target_after_enable_ = true;
  activation_command_to_measured_norm_m_ = std::numeric_limits<double>::quiet_NaN();
  seeded_command_to_measured_norm_m_ = std::numeric_limits<double>::quiet_NaN();
  command_seed_source_ = "NONE";
  initialized_ = false;
  return CallbackReturn::SUCCESS;
}

void SafeCartesianPoseController::target_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto now = get_node()->now();
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  const bool pose_valid = pose_to_eigen(*msg, position, orientation);
  if (pose_valid) {
    publish_debug_pose(debug_received_target_pub_, now, position, orientation);
  }
  if (!accept_targets_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Rejecting target: target acceptance is disabled.");
    reject_target("accept_targets=false");
    publish_target_status(now);
    return;
  }
  const rclcpp::Time msg_stamp(msg->header.stamp, get_node()->get_clock()->get_clock_type());
  if (activation_time_.nanoseconds() > 0 &&
      (msg_stamp.nanoseconds() <= 0 || msg_stamp < activation_time_)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Rejecting target with missing/old stamp before controller activation time.");
    reject_target("old_or_missing_stamp");
    publish_target_status(now);
    return;
  }

  if (!pose_valid) {
    RCLCPP_ERROR(get_node()->get_logger(), "Rejecting non-finite target pose.");
    reject_target("invalid_pose");
    publish_target_status(now);
    return;
  }

  Eigen::Quaterniond measured_orientation;
  Eigen::Vector3d measured_position;
  if (!read_measured_current_pose(measured_orientation, measured_position, "target callback")) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Rejecting target: measured current pose is unavailable.");
    accept_targets_.store(false, std::memory_order_release);
    clear_all_targets_and_hold_current("target callback measured unavailable");
    reject_target("measured_unavailable");
    publish_target_status(now);
    return;
  }
  last_target_to_command_error_m_ = (position - command_position_).norm();
  last_target_to_measured_error_m_ = (position - measured_position).norm();
  last_command_to_measured_error_m_ = (command_position_ - measured_position).norm();
  if (first_target_after_enable_) {
    if (last_target_to_command_error_m_ > first_target_tolerance_m_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Rejecting first target: target-command distance %.9f m exceeds %.9f m. "
                   "First target must be continuous with internal command pose.",
                   last_target_to_command_error_m_, first_target_tolerance_m_);
      accept_targets_.store(false, std::memory_order_release);
      clear_all_targets_and_hold_current("first target too far from internal command");
      reject_target("first_target_too_far_from_command");
      publish_target_status(now);
      return;
    }
    if (last_command_to_measured_error_m_ > command_measured_tracking_tolerance_m_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Rejecting first target: command-measured distance %.9f m exceeds %.9f m. "
                   "Measured pose is the safety reference; internal command pose is too far from it.",
                   last_command_to_measured_error_m_,
                   command_measured_tracking_tolerance_m_);
      accept_targets_.store(false, std::memory_order_release);
      clear_all_targets_and_hold_current("internal command too far from measured");
      reject_target("command_too_far_from_measured");
      publish_target_status(now);
      return;
    }
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  target_position_ = position;
  target_orientation_ = orientation;
  last_target_time_ = now;
  has_target_ = true;
  first_target_after_enable_ = false;
  target_available_.store(true, std::memory_order_release);
  target_accepted_count_++;
  last_target_reject_reason_ = "NONE";
  publish_debug_pose(debug_accepted_target_pub_, now, position, orientation);
  publish_target_status(now);
}

void SafeCartesianPoseController::hold_current_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  accept_targets_.store(false, std::memory_order_release);
  response->success = clear_all_targets_and_hold_current("hold_current service");
  response->message =
      response->success
          ? "holding continuous internal command pose; measured pose is only checked for warning"
          : "failed to hold continuous internal command pose";
}

void SafeCartesianPoseController::clear_target_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  accept_targets_.store(false, std::memory_order_release);
  response->success = clear_all_targets_and_hold_current("clear_target service");
  response->message =
      response->success
          ? "cleared target and holding continuous internal command pose"
          : "failed to clear target and hold continuous internal command pose";
}

void SafeCartesianPoseController::enable_targets_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (!initialized_) {
    response->success = false;
    response->message = "controller is not initialized";
    return;
  }
  if (!clear_all_targets_and_hold_current("enable_targets service")) {
    response->success = false;
    response->message =
        "failed to hold continuous internal command pose before enabling targets";
    return;
  }
  first_target_after_enable_ = true;
  accept_targets_.store(true, std::memory_order_release);
  response->success = true;
  response->message =
      "target acceptance enabled; first target must be close to internal command pose";
}

bool SafeCartesianPoseController::pose_to_eigen(const geometry_msgs::msg::PoseStamped& msg,
                                                Eigen::Vector3d& position,
                                                Eigen::Quaterniond& orientation) const {
  position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                                   msg.pose.orientation.y, msg.pose.orientation.z);
  if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
      !std::isfinite(position.z()) || !std::isfinite(orientation.w()) ||
      !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z())) {
    return false;
  }
  if (orientation.norm() < 1e-6) {
    return false;
  }
  orientation.normalize();
  return true;
}

void SafeCartesianPoseController::clear_all_targets() {
  std::lock_guard<std::mutex> lock(target_mutex_);
  has_target_ = false;
  rt_has_target_ = false;
  target_stream_primed_ = false;
  target_position_.setConstant(std::numeric_limits<double>::quiet_NaN());
  rt_target_position_.setConstant(std::numeric_limits<double>::quiet_NaN());
  target_orientation_ = Eigen::Quaterniond(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
  rt_target_orientation_ = target_orientation_;
  last_target_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  rt_last_target_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  target_available_.store(false, std::memory_order_release);
  first_target_after_enable_ = true;
  command_velocity_.setZero();
  command_acceleration_.setZero();
}

bool SafeCartesianPoseController::read_measured_current_pose(Eigen::Quaterniond& orientation,
                                                             Eigen::Vector3d& position,
                                                             const char* context) const {
  if (!use_state_interfaces_) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Cannot read measured Cartesian pose in %s: state interfaces disabled.",
                 context);
    return false;
  }
  try {
    std::tie(orientation, position) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Failed to read measured Cartesian pose in %s: %s.", context, exc.what());
    return false;
  }
  if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
      !std::isfinite(position.z()) || !std::isfinite(orientation.w()) ||
      !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z()) || orientation.norm() < 1e-6) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid measured Cartesian pose in %s.", context);
    return false;
  }
  orientation.normalize();
  return true;
}

bool SafeCartesianPoseController::read_command_interface_pose(Eigen::Quaterniond& orientation,
                                                              Eigen::Vector3d& position,
                                                              const char* context) const {
  try {
    std::tie(orientation, position) =
        franka_cartesian_pose_->getCommandedOrientationAndTranslation();
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Failed to read seeded Cartesian command pose in %s: %s.", context,
                 exc.what());
    return false;
  }
  if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
      !std::isfinite(position.z()) || !std::isfinite(orientation.w()) ||
      !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
      !std::isfinite(orientation.z()) || orientation.norm() < 1e-6) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid seeded Cartesian command pose in %s.",
                 context);
    return false;
  }
  orientation.normalize();
  return true;
}

bool SafeCartesianPoseController::clear_all_targets_and_hold_current(const char* context) {
  if (!hold_continuous_command_pose(context)) {
    clear_all_targets();
    accept_targets_.store(false, std::memory_order_release);
    first_target_after_enable_ = true;
    return false;
  }
  return true;
}

bool SafeCartesianPoseController::hold_continuous_command_pose(const char* context) {
  Eigen::Quaterniond measured_orientation;
  Eigen::Vector3d measured_position;
  if (!read_measured_current_pose(measured_orientation, measured_position, context)) {
    return false;
  }
  const double tracking_error = (command_position_ - measured_position).norm();
  if (tracking_error > activation_hold_tolerance_m_) {
    const Eigen::Vector3d error = command_position_ - measured_position;
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Holding continuous internal Cartesian command pose in %s. This does not force command "
        "to measured pose. Command-measured distance is %.9f m "
        "(dx=%.9f, dy=%.9f, dz=%.9f, activation tolerance %.9f m). "
        "Target stream remains cleared.",
        context, tracking_error, error.x(), error.y(), error.z(), activation_hold_tolerance_m_);
  }
  clear_all_targets();
  const bool ok = franka_cartesian_pose_->setCommand(command_orientation_, command_position_);
  if (!ok) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set continuous hold command in %s.",
                 context);
  }
  return ok;
}

void SafeCartesianPoseController::publish_debug_pose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
    const rclcpp::Time& time,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) {
  if (!pub || pub->get_subscription_count() == 0) {
    return;
  }
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = arm_id_ + "_link0";
  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();
  const auto normalized = orientation.normalized();
  msg.pose.orientation.x = normalized.x();
  msg.pose.orientation.y = normalized.y();
  msg.pose.orientation.z = normalized.z();
  msg.pose.orientation.w = normalized.w();
  pub->publish(msg);
}

void SafeCartesianPoseController::maybe_publish_debug_poses(const rclcpp::Time& time) {
  if (debug_publish_period_s_ <= 0.0) {
    return;
  }
  if (last_debug_publish_time_.nanoseconds() > 0 &&
      (time - last_debug_publish_time_).seconds() < debug_publish_period_s_) {
    return;
  }
  last_debug_publish_time_ = time;
  publish_debug_pose(debug_internal_command_pub_, time, command_position_, command_orientation_);
  if (rt_has_target_) {
    publish_debug_pose(debug_rt_target_pub_, time, rt_target_position_, rt_target_orientation_);
  }
  publish_target_status(time);
}

void SafeCartesianPoseController::reject_target(const char* reason) {
  target_rejected_count_++;
  last_target_reject_reason_ = reason;
}

void SafeCartesianPoseController::publish_target_status(const rclcpp::Time& time) {
  if (!debug_target_status_pub_ || debug_target_status_pub_->get_subscription_count() == 0) {
    return;
  }
  std::ostringstream stream;
  stream << "time=" << time.seconds()
         << " accept_targets=" << (accept_targets_.load(std::memory_order_acquire) ? "True" : "False")
         << " target_accepted_count=" << target_accepted_count_
         << " target_rejected_count=" << target_rejected_count_
         << " last_target_reject_reason=" << last_target_reject_reason_
         << " target_to_command_error=" << last_target_to_command_error_m_
         << " target_to_measured_error=" << last_target_to_measured_error_m_
         << " command_to_measured_error=" << last_command_to_measured_error_m_
         << " activation_command_to_measured_norm=" << activation_command_to_measured_norm_m_
         << " seeded_command_to_measured_norm=" << seeded_command_to_measured_norm_m_
         << " command_seed_source=" << command_seed_source_
         << " controller_update_period_s=" << last_update_period_s_
         << " controller_update_overrun_count=" << update_overrun_count_
         << " translation_tracking_time_s=" << translation_tracking_time_s_
         << " max_translation_speed_mps=" << max_translation_speed_mps_
         << " max_translation_acceleration_mps2=" << max_translation_acceleration_mps2_
         << " max_translation_jerk_mps3=" << max_translation_jerk_mps3_
         << " max_translation_step_m=" << max_translation_step_m_
         << " max_target_distance_m=" << max_target_distance_m_
         << " watchdog_timeout_sec=" << watchdog_timeout_sec_
         << " raw_target_to_command_error_norm=" << raw_target_to_command_error_norm_
         << " raw_target_to_command_error_x=" << raw_target_to_command_error_.x()
         << " raw_target_to_command_error_y=" << raw_target_to_command_error_.y()
         << " raw_target_to_command_error_z=" << raw_target_to_command_error_.z()
         << " target_distance_clamped=" << (target_distance_clamped_ ? "True" : "False")
         << " desired_position_before_guard_x=" << desired_position_before_guard_.x()
         << " desired_position_before_guard_y=" << desired_position_before_guard_.y()
         << " desired_position_before_guard_z=" << desired_position_before_guard_.z()
         << " desired_position_after_guard_x=" << desired_position_after_guard_.x()
         << " desired_position_after_guard_y=" << desired_position_after_guard_.y()
         << " desired_position_after_guard_z=" << desired_position_after_guard_.z()
         << " desired_velocity_x=" << debug_desired_velocity_.x()
         << " desired_velocity_y=" << debug_desired_velocity_.y()
         << " desired_velocity_z=" << debug_desired_velocity_.z()
         << " desired_velocity_norm=" << debug_desired_velocity_norm_
         << " desired_speed_limited=" << (debug_desired_speed_limited_ ? "True" : "False")
         << " desired_acceleration_x=" << debug_desired_acceleration_.x()
         << " desired_acceleration_y=" << debug_desired_acceleration_.y()
         << " desired_acceleration_z=" << debug_desired_acceleration_.z()
         << " desired_acceleration_norm=" << debug_desired_acceleration_norm_
         << " desired_acceleration_limited="
         << (debug_desired_acceleration_limited_ ? "True" : "False")
         << " acceleration_delta_norm=" << debug_acceleration_delta_norm_
         << " jerk_limited=" << (debug_jerk_limited_ ? "True" : "False")
         << " command_velocity_x=" << debug_command_velocity_.x()
         << " command_velocity_y=" << debug_command_velocity_.y()
         << " command_velocity_z=" << debug_command_velocity_.z()
         << " command_velocity_norm=" << debug_command_velocity_norm_
         << " command_acceleration_x=" << debug_command_acceleration_.x()
         << " command_acceleration_y=" << debug_command_acceleration_.y()
         << " command_acceleration_z=" << debug_command_acceleration_.z()
         << " command_acceleration_norm=" << debug_command_acceleration_norm_
         << " step_x=" << debug_step_.x()
         << " step_y=" << debug_step_.y()
         << " step_z=" << debug_step_.z()
         << " step_norm=" << debug_step_norm_
         << " step_limited=" << (debug_step_limited_ ? "True" : "False")
         << " command_measured_error_x=" << debug_command_measured_error_.x()
         << " command_measured_error_y=" << debug_command_measured_error_.y()
         << " command_measured_error_z=" << debug_command_measured_error_.z()
         << " command_measured_error_norm=" << debug_command_measured_error_norm_
         << " target_measured_error_norm=" << debug_target_measured_error_norm_
         << " has_target=" << (has_target_ ? "True" : "False")
         << " rt_has_target=" << (rt_has_target_ ? "True" : "False")
         << " target_stream_primed=" << (target_stream_primed_ ? "True" : "False");
  std_msgs::msg::String msg;
  msg.data = stream.str();
  debug_target_status_pub_->publish(msg);
}

void SafeCartesianPoseController::update_tracking_debug(
    const Eigen::Vector3d& desired_position, const Eigen::Vector3d& command_position) {
  try {
    Eigen::Quaterniond measured_orientation;
    Eigen::Vector3d measured_position;
    std::tie(measured_orientation, measured_position) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    (void)measured_orientation;
    if (!std::isfinite(measured_position.x()) || !std::isfinite(measured_position.y()) ||
        !std::isfinite(measured_position.z())) {
      throw std::runtime_error("non-finite measured position");
    }
    debug_command_measured_error_ = command_position - measured_position;
    debug_command_measured_error_norm_ = debug_command_measured_error_.norm();
    debug_target_measured_error_norm_ = (desired_position - measured_position).norm();
  } catch (const std::exception&) {
    debug_command_measured_error_.setConstant(std::numeric_limits<double>::quiet_NaN());
    debug_command_measured_error_norm_ = std::numeric_limits<double>::quiet_NaN();
    debug_target_measured_error_norm_ = std::numeric_limits<double>::quiet_NaN();
  }
}

Eigen::Vector3d SafeCartesianPoseController::limit_translation_step(
    const Eigen::Vector3d& desired, const Eigen::Vector3d& current, double dt) {
  const Eigen::Vector3d error = desired - current;
  const double tracking_time = std::max(0.05, translation_tracking_time_s_);
  Eigen::Vector3d desired_velocity = error / tracking_time;

  const double desired_speed = desired_velocity.norm();
  debug_desired_velocity_ = desired_velocity;
  debug_desired_velocity_norm_ = desired_speed;
  debug_desired_speed_limited_ =
      desired_speed > max_translation_speed_mps_ && desired_speed > 1e-12;
  if (desired_speed > max_translation_speed_mps_ && desired_speed > 1e-12) {
    desired_velocity *= max_translation_speed_mps_ / desired_speed;
  }

  Eigen::Vector3d desired_acceleration = (desired_velocity - command_velocity_) / dt;
  const double desired_acceleration_norm = desired_acceleration.norm();
  debug_desired_acceleration_ = desired_acceleration;
  debug_desired_acceleration_norm_ = desired_acceleration_norm;
  debug_desired_acceleration_limited_ =
      desired_acceleration_norm > max_translation_acceleration_mps2_ &&
      desired_acceleration_norm > 1e-12;
  if (desired_acceleration_norm > max_translation_acceleration_mps2_ &&
      desired_acceleration_norm > 1e-12) {
    desired_acceleration *= max_translation_acceleration_mps2_ / desired_acceleration_norm;
  }

  Eigen::Vector3d acceleration_delta = desired_acceleration - command_acceleration_;
  const double acceleration_delta_norm = acceleration_delta.norm();
  const double max_acceleration_delta = max_translation_jerk_mps3_ * dt;
  debug_acceleration_delta_norm_ = acceleration_delta_norm;
  debug_jerk_limited_ =
      acceleration_delta_norm > max_acceleration_delta && acceleration_delta_norm > 1e-12;
  if (acceleration_delta_norm > max_acceleration_delta && acceleration_delta_norm > 1e-12) {
    acceleration_delta *= max_acceleration_delta / acceleration_delta_norm;
  }

  command_acceleration_ += acceleration_delta;
  const double acceleration_norm = command_acceleration_.norm();
  if (acceleration_norm > max_translation_acceleration_mps2_ && acceleration_norm > 1e-12) {
    command_acceleration_ *= max_translation_acceleration_mps2_ / acceleration_norm;
  }
  debug_command_acceleration_ = command_acceleration_;
  debug_command_acceleration_norm_ = command_acceleration_.norm();

  command_velocity_ += command_acceleration_ * dt;
  const double speed_norm = command_velocity_.norm();
  if (speed_norm > max_translation_speed_mps_ && speed_norm > 1e-12) {
    command_velocity_ *= max_translation_speed_mps_ / speed_norm;
  }
  debug_command_velocity_ = command_velocity_;
  debug_command_velocity_norm_ = command_velocity_.norm();

  Eigen::Vector3d step = command_velocity_ * dt;
  const double step_norm = step.norm();
  debug_step_ = step;
  debug_step_norm_ = step_norm;
  debug_step_limited_ = step_norm > max_translation_step_m_ && step_norm > 1e-12;
  if (step_norm > max_translation_step_m_ && step_norm > 1e-12) {
    step *= max_translation_step_m_ / step_norm;
    command_velocity_ = step / dt;
  }
  debug_step_ = step;
  debug_step_norm_ = step.norm();
  debug_command_velocity_ = command_velocity_;
  debug_command_velocity_norm_ = command_velocity_.norm();

  if (step.dot(error) > 0.0 && step.norm() > error.norm()) {
    command_velocity_.setZero();
    command_acceleration_.setZero();
    debug_step_ = error;
    debug_step_norm_ = error.norm();
    debug_command_velocity_ = command_velocity_;
    debug_command_velocity_norm_ = command_velocity_.norm();
    debug_command_acceleration_ = command_acceleration_;
    debug_command_acceleration_norm_ = command_acceleration_.norm();
    return desired;
  }

  return current + step;
}

Eigen::Quaterniond SafeCartesianPoseController::limit_rotation_step(
    const Eigen::Quaterniond& desired, const Eigen::Quaterniond& current) const {
  Eigen::Quaterniond normalized_desired = desired.normalized();
  Eigen::Quaterniond normalized_current = current.normalized();
  if (normalized_current.dot(normalized_desired) < 0.0) {
    normalized_desired.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd delta(normalized_current.inverse() * normalized_desired);
  const double angle = std::abs(delta.angle());
  if (angle <= max_rotation_step_rad_ || angle < 1e-12) {
    return normalized_desired;
  }
  const double t = std::clamp(max_rotation_step_rad_ / angle, 0.0, 1.0);
  return normalized_current.slerp(t, normalized_desired).normalized();
}

}  // namespace serl_franka_ros2_control

PLUGINLIB_EXPORT_CLASS(serl_franka_ros2_control::SafeCartesianPoseController,
                       controller_interface::ControllerInterface)
