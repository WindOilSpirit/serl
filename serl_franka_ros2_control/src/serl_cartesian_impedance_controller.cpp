#include "serl_franka_ros2_control/serl_cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace {

template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd& matrix, double tolerance = 1.0e-6) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& singular_values = svd.singularValues();
  Eigen::MatrixXd singular_values_inv =
      Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > tolerance) {
      singular_values_inv(i, i) = 1.0 / singular_values(i);
    }
  }
  return svd.matrixV() * singular_values_inv * svd.matrixU().transpose();
}

double quaternion_angle(const Eigen::Quaterniond& q_in) {
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return 2.0 * std::atan2(q.vec().norm(), std::abs(q.w()));
}

geometry_msgs::msg::PoseStamped make_pose_msg(const rclcpp::Time& time,
                                              const std::string& frame_id,
                                              const Eigen::Vector3d& position,
                                              const Eigen::Quaterniond& orientation) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id;
  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();
  msg.pose.orientation.w = orientation.w();
  return msg;
}

double quiet_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace serl_franka_ros2_control {

controller_interface::InterfaceConfiguration
SerlCartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names()) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
SerlCartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names()) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  const std::string arm_id =
      get_node()->has_parameter("arm_id") ? get_node()->get_parameter("arm_id").as_string()
                                          : arm_id_;
  config.names.push_back(arm_id + "/robot_model");
  config.names.push_back(arm_id + "/robot_state");
  return config;
}

controller_interface::return_type SerlCartesianImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  update_period_s_ = period.seconds();
  const double dt = std::isfinite(update_period_s_) && update_period_s_ > 0.0
                        ? update_period_s_
                        : 0.001;

  franka::RobotState* robot_state = get_robot_state_ptr();
  if (robot_state == nullptr) {
    RCLCPP_FATAL(get_node()->get_logger(), "Franka robot_state interface is unavailable.");
    return controller_interface::return_type::ERROR;
  }

  try {
    update_joint_state_from_robot_state(*robot_state);
    if (!read_measured_pose(measured_pose_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Measured Franka end-effector pose is invalid.");
      return controller_interface::return_type::ERROR;
    }

    PoseReference target;
    rclcpp::Time last_target_snapshot;
    bool has_target_snapshot = false;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target = raw_target_;
      last_target_snapshot = last_target_time_;
      has_target_snapshot = target_received_;
    }
    target_age_s_ =
        has_target_snapshot && last_target_snapshot.nanoseconds() > 0
            ? (time - last_target_snapshot).seconds()
            : std::numeric_limits<double>::quiet_NaN();

    if (!smoothed_target_initialized_) {
      smoothed_target_ = measured_pose_;
      smoothed_target_initialized_ = true;
    }
    Eigen::Quaterniond target_orientation = target.orientation;
    if (smoothed_target_.orientation.coeffs().dot(target_orientation.coeffs()) < 0.0) {
      target_orientation.coeffs() *= -1.0;
    }
    smoothed_target_.position =
        filter_coeff_ * target.position + (1.0 - filter_coeff_) * smoothed_target_.position;
    smoothed_target_.orientation =
        smoothed_target_.orientation.slerp(filter_coeff_, target_orientation).normalized();
    limited_reference_ = smoothed_target_;
    constexpr double x_command_detection_threshold = 1.0e-5;
    bool positive_x_compensation_active = false;
    if (previous_raw_target_x_initialized_) {
      positive_x_compensation_active =
          (target.position.x() - previous_raw_target_x_) > x_command_detection_threshold;
    }
    previous_raw_target_x_ = target.position.x();
    previous_raw_target_x_initialized_ = true;

    constexpr double z_command_detection_threshold = 1.0e-5;
    bool z_gravity_compensation_active = false;
    if (previous_raw_target_z_initialized_) {
      z_gravity_compensation_active =
          std::abs(target.position.z() - previous_raw_target_z_) > z_command_detection_threshold;
    }
    previous_raw_target_z_ = target.position.z();
    previous_raw_target_z_initialized_ = true;

    Eigen::Matrix<double, 6, 1> error =
        compute_cartesian_error(measured_pose_, limited_reference_);
    position_error_before_clip_ = error.head(3).norm();
    position_error_after_clip_ = error.head(3).norm();
    orientation_error_before_clip_ = error.tail(3).norm();
    orientation_error_after_clip_ = error.tail(3).norm();
    reference_was_clipped_ = false;

    error_i_.head(3) += error.head(3) * dt;
    error_i_.tail(3) += error.tail(3) * dt;
    error_i_.head(3) = error_i_.head(3).cwiseMax(-0.1).cwiseMin(0.1);
    error_i_.tail(3) = error_i_.tail(3).cwiseMax(-0.3).cwiseMin(0.3);

    const auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
    const auto jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    const Eigen::Matrix<double, 6, 1> cartesian_velocity = jacobian * dq_;
    debug_jacobian_velocity_ = cartesian_velocity;
    if (dt > 0.0 && previous_measured_position_initialized_) {
      debug_pose_diff_velocity_ = (measured_pose_.position - previous_measured_position_) / dt;
      const Eigen::Vector3d jacobian_linear_velocity = cartesian_velocity.head(3);
      const double jacobian_norm = jacobian_linear_velocity.norm();
      const double pose_diff_norm = debug_pose_diff_velocity_.norm();
      debug_velocity_diff_norm_ = (jacobian_linear_velocity - debug_pose_diff_velocity_).norm();
      if (jacobian_norm > 1.0e-12 && pose_diff_norm > 1.0e-12) {
        debug_velocity_direction_cosine_ =
            jacobian_linear_velocity.dot(debug_pose_diff_velocity_) /
            (jacobian_norm * pose_diff_norm);
        debug_velocity_norm_ratio_ = jacobian_norm / pose_diff_norm;
      } else {
        debug_velocity_direction_cosine_ = quiet_nan();
        debug_velocity_norm_ratio_ = quiet_nan();
      }
    } else {
      debug_pose_diff_velocity_.setConstant(quiet_nan());
      debug_velocity_direction_cosine_ = quiet_nan();
      debug_velocity_norm_ratio_ = quiet_nan();
      debug_velocity_diff_norm_ = quiet_nan();
    }
    previous_measured_position_ = measured_pose_.position;
    previous_measured_position_initialized_ = true;

    const double translational_stiffness =
        runtime_translational_stiffness_.load(std::memory_order_relaxed);
    const auto axis_stiffness = [translational_stiffness](double override_value) {
      return override_value > 0.0 ? override_value : translational_stiffness;
    };
    const double translational_stiffness_x =
        axis_stiffness(runtime_translational_stiffness_x_.load(std::memory_order_relaxed));
    const double translational_stiffness_y =
        axis_stiffness(runtime_translational_stiffness_y_.load(std::memory_order_relaxed));
    const double translational_stiffness_z =
        axis_stiffness(runtime_translational_stiffness_z_.load(std::memory_order_relaxed));
    const double translational_damping_x =
        2.0 * std::sqrt(std::max(translational_stiffness_x, 1.0e-6));
    const double translational_damping_y =
        2.0 * std::sqrt(std::max(translational_stiffness_y, 1.0e-6));
    const double translational_damping_z =
        2.0 * std::sqrt(std::max(translational_stiffness_z, 1.0e-6));
    const double rotational_damping =
        2.0 * std::sqrt(std::max(rotational_stiffness_, 1.0e-6));

    Eigen::Matrix<double, 6, 6> cartesian_stiffness = Eigen::Matrix<double, 6, 6>::Zero();
    cartesian_stiffness(0, 0) = translational_stiffness_x;
    cartesian_stiffness(1, 1) = translational_stiffness_y;
    cartesian_stiffness(2, 2) = translational_stiffness_z;
    cartesian_stiffness.bottomRightCorner(3, 3) =
        rotational_stiffness_ * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> cartesian_damping = Eigen::Matrix<double, 6, 6>::Zero();
    cartesian_damping(0, 0) = translational_damping_x;
    cartesian_damping(1, 1) = translational_damping_y;
    cartesian_damping(2, 2) = translational_damping_z;
    cartesian_damping.bottomRightCorner(3, 3) =
        rotational_damping * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 1> wrench =
        -cartesian_stiffness * error - cartesian_damping * cartesian_velocity -
        cartesian_ki_ * error_i_;
    if (positive_x_compensation_active) {
      wrench(0) += runtime_positive_x_compensation_force_.load(std::memory_order_relaxed);
    }
    if (z_gravity_compensation_active) {
      wrench(2) += runtime_z_gravity_compensation_force_.load(std::memory_order_relaxed);
    }
    constexpr double linear_wrench_norm_limit = 80.0;
    constexpr double angular_wrench_norm_limit = 8.0;
    const double linear_wrench_norm = wrench.head(3).norm();
    if (linear_wrench_norm > linear_wrench_norm_limit) {
      wrench.head(3) *= linear_wrench_norm_limit / linear_wrench_norm;
    }
    const double angular_wrench_norm = wrench.tail(3).norm();
    if (angular_wrench_norm > angular_wrench_norm_limit) {
      wrench.tail(3) *= angular_wrench_norm_limit / angular_wrench_norm;
    }
    // Keep the physically consistent torque mapping tau = J^T F.
    // Do not premultiply wrench by (J J^T)^-1 here; that changes the wrench semantics.
    const Vector7d tau_task = jacobian.transpose() * wrench;

    const Eigen::MatrixXd jacobian_pinv = pseudo_inverse(jacobian);
    Vector7d tau_nullspace =
        (Eigen::Matrix<double, 7, 7>::Identity() -
         jacobian.transpose() * jacobian_pinv.transpose()) *
        (nullspace_stiffness_ * (q_d_nullspace_ - q_) -
         2.0 * std::sqrt(std::max(nullspace_stiffness_, 0.0)) * dq_);
    if (!enable_nullspace_torque_) {
      tau_nullspace.setZero();
    }

    const Eigen::Matrix<double, 6, 1> wrench_est =
        pseudo_inverse(jacobian.transpose()) * tau_task;
    const Vector7d tau_d_calculated = tau_task + tau_nullspace + coriolis;
    const Vector7d tau_d_saturated = saturate_torque_rate(tau_d_calculated, last_tau_command_);

    debug_cartesian_error_ = error;
    debug_wrench_ = wrench;
    debug_tau_task_ = tau_task;
    debug_tau_nullspace_ = tau_nullspace;
    debug_coriolis_ = coriolis;
    debug_tau_before_saturation_ = tau_d_calculated;
    debug_tau_after_saturation_ = tau_d_saturated;
    debug_tau_j_ = Eigen::Map<const Vector7d>(robot_state->tau_J.data());
    debug_tau_j_d_ = Eigen::Map<const Vector7d>(robot_state->tau_J_d.data());
    debug_tau_ext_hat_filtered_ =
        Eigen::Map<const Vector7d>(robot_state->tau_ext_hat_filtered.data());
    debug_o_f_ext_hat_k_ =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(robot_state->O_F_ext_hat_K.data());
    debug_k_f_ext_hat_k_ =
        Eigen::Map<const Eigen::Matrix<double, 6, 1>>(robot_state->K_F_ext_hat_K.data());
    debug_wrench_est_ = wrench_est;
    debug_wrench_est_error_ = wrench_est - wrench;
    debug_wrench_est_error_norm_ = debug_wrench_est_error_.norm();
    debug_tau_task_nullspace_dot_ = tau_task.dot(tau_nullspace);
    debug_positive_x_compensation_active_ = positive_x_compensation_active;
    debug_z_gravity_compensation_active_ = z_gravity_compensation_active;
    debug_o_t_ee_ = robot_state->O_T_EE;
    debug_zero_jacobian_ = jacobian_array;
    last_tau_command_ = tau_d_saturated;
    tau_norm_ = tau_d_saturated.norm();
    translational_damping_ =
        (translational_damping_x + translational_damping_y + translational_damping_z) / 3.0;
    rotational_damping_ = rotational_damping;

    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_.at(i).set_value(tau_d_saturated(i));
    }

    maybe_publish_debug(time);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(get_node()->get_logger(), "SERL Cartesian impedance update failed: %s", ex.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SerlCartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<std::string>("target_topic", target_topic_);
    auto_declare<std::string>("reference_limit_mode", reference_limit_mode_);
    auto_declare<bool>("use_robot_state_q_dq", use_robot_state_q_dq_);
    auto_declare<bool>("enable_nullspace_torque", enable_nullspace_torque_);
    auto_declare<double>("translational_stiffness", translational_stiffness_);
    auto_declare<double>("translational_stiffness_x", -1.0);
    auto_declare<double>("translational_stiffness_y", -1.0);
    auto_declare<double>("translational_stiffness_z", -1.0);
    auto_declare<double>("z_gravity_compensation_force", 0.0);
    auto_declare<double>("positive_x_compensation_force", 0.0);
    auto_declare<double>("rotational_stiffness", rotational_stiffness_);
    auto_declare<double>("translational_damping", translational_damping_);
    auto_declare<double>("rotational_damping", rotational_damping_);
    auto_declare<double>("translational_Ki", translational_ki_);
    auto_declare<double>("rotational_Ki", rotational_ki_);
    auto_declare<double>("max_pos_error", max_pos_error_);
    auto_declare<double>("max_ori_error", max_ori_error_);
    auto_declare<double>("translational_clip_neg_x", -translational_clip_min_.x());
    auto_declare<double>("translational_clip_neg_y", -translational_clip_min_.y());
    auto_declare<double>("translational_clip_neg_z", -translational_clip_min_.z());
    auto_declare<double>("translational_clip_x", translational_clip_max_.x());
    auto_declare<double>("translational_clip_y", translational_clip_max_.y());
    auto_declare<double>("translational_clip_z", translational_clip_max_.z());
    auto_declare<double>("rotational_clip_neg_x", -rotational_clip_min_.x());
    auto_declare<double>("rotational_clip_neg_y", -rotational_clip_min_.y());
    auto_declare<double>("rotational_clip_neg_z", -rotational_clip_min_.z());
    auto_declare<double>("rotational_clip_x", rotational_clip_max_.x());
    auto_declare<double>("rotational_clip_y", rotational_clip_max_.y());
    auto_declare<double>("rotational_clip_z", rotational_clip_max_.z());
    auto_declare<double>("filter_coeff", filter_coeff_);
    auto_declare<double>("watchdog_timeout_sec", watchdog_timeout_sec_);
    auto_declare<double>("nullspace_stiffness", nullspace_stiffness_);
    auto_declare<double>("joint1_nullspace_stiffness", joint1_nullspace_stiffness_);
    auto_declare<double>("torque_rate_limit", torque_rate_limit_);
    auto_declare<double>("debug_publish_rate", debug_publish_rate_);
  } catch (const std::exception& ex) {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", ex.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SerlCartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  configured_joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  reference_limit_mode_ = get_node()->get_parameter("reference_limit_mode").as_string();
  use_robot_state_q_dq_ = get_node()->get_parameter("use_robot_state_q_dq").as_bool();
  enable_nullspace_torque_ = get_node()->get_parameter("enable_nullspace_torque").as_bool();
  translational_stiffness_ = get_node()->get_parameter("translational_stiffness").as_double();
  runtime_translational_stiffness_.store(translational_stiffness_, std::memory_order_relaxed);
  runtime_translational_stiffness_x_.store(
      get_node()->get_parameter("translational_stiffness_x").as_double(),
      std::memory_order_relaxed);
  runtime_translational_stiffness_y_.store(
      get_node()->get_parameter("translational_stiffness_y").as_double(),
      std::memory_order_relaxed);
  runtime_translational_stiffness_z_.store(
      get_node()->get_parameter("translational_stiffness_z").as_double(),
      std::memory_order_relaxed);
  runtime_z_gravity_compensation_force_.store(
      get_node()->get_parameter("z_gravity_compensation_force").as_double(),
      std::memory_order_relaxed);
  runtime_positive_x_compensation_force_.store(
      get_node()->get_parameter("positive_x_compensation_force").as_double(),
      std::memory_order_relaxed);
  rotational_stiffness_ = get_node()->get_parameter("rotational_stiffness").as_double();
  translational_damping_ = get_node()->get_parameter("translational_damping").as_double();
  rotational_damping_ = get_node()->get_parameter("rotational_damping").as_double();
  translational_ki_ = get_node()->get_parameter("translational_Ki").as_double();
  rotational_ki_ = get_node()->get_parameter("rotational_Ki").as_double();
  max_pos_error_ = get_node()->get_parameter("max_pos_error").as_double();
  max_ori_error_ = get_node()->get_parameter("max_ori_error").as_double();
  translational_clip_min_ << -get_node()->get_parameter("translational_clip_neg_x").as_double(),
      -get_node()->get_parameter("translational_clip_neg_y").as_double(),
      -get_node()->get_parameter("translational_clip_neg_z").as_double();
  translational_clip_max_ << get_node()->get_parameter("translational_clip_x").as_double(),
      get_node()->get_parameter("translational_clip_y").as_double(),
      get_node()->get_parameter("translational_clip_z").as_double();
  rotational_clip_min_ << -get_node()->get_parameter("rotational_clip_neg_x").as_double(),
      -get_node()->get_parameter("rotational_clip_neg_y").as_double(),
      -get_node()->get_parameter("rotational_clip_neg_z").as_double();
  rotational_clip_max_ << get_node()->get_parameter("rotational_clip_x").as_double(),
      get_node()->get_parameter("rotational_clip_y").as_double(),
      get_node()->get_parameter("rotational_clip_z").as_double();
  filter_coeff_ = std::clamp(get_node()->get_parameter("filter_coeff").as_double(), 0.0, 1.0);
  watchdog_timeout_sec_ = get_node()->get_parameter("watchdog_timeout_sec").as_double();
  nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
  joint1_nullspace_stiffness_ =
      get_node()->get_parameter("joint1_nullspace_stiffness").as_double();
  torque_rate_limit_ = get_node()->get_parameter("torque_rate_limit").as_double();
  debug_publish_rate_ = get_node()->get_parameter("debug_publish_rate").as_double();

  if (!configured_joint_names_.empty() && configured_joint_names_.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names must contain exactly 7 names.");
    return CallbackReturn::FAILURE;
  }
  if (reference_limit_mode_ != "per_axis_error_clip" &&
      reference_limit_mode_ != "norm_target_distance") {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "reference_limit_mode must be 'per_axis_error_clip' or 'norm_target_distance'.");
    return CallbackReturn::FAILURE;
  }

  cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3, 3) =
      translational_stiffness_ * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) =
      rotational_stiffness_ * Eigen::Matrix3d::Identity();
  cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3, 3) = translational_damping_ * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) = rotational_damping_ * Eigen::Matrix3d::Identity();
  cartesian_ki_.setZero();
  cartesian_ki_.topLeftCorner(3, 3) = translational_ki_ * Eigen::Matrix3d::Identity();
  cartesian_ki_.bottomRightCorner(3, 3) = rotational_ki_ * Eigen::Matrix3d::Identity();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");

  const auto target_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, target_qos,
      std::bind(&SerlCartesianImpedanceController::target_callback, this, std::placeholders::_1));
  raw_target_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/raw_target_pose", 10);
  smoothed_target_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/debug/smoothed_target_pose", 10);
  limited_reference_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/debug/clipped_target_pose", 10);
  measured_pose_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/measured_pose", 10);
  status_pub_ = get_node()->create_publisher<std_msgs::msg::String>("~/debug/status", 10);
  parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
          const auto& name = parameter.get_name();
          if (name != "translational_stiffness" &&
              name != "translational_stiffness_x" &&
              name != "translational_stiffness_y" &&
              name != "translational_stiffness_z" &&
              name != "z_gravity_compensation_force" &&
              name != "positive_x_compensation_force") {
            continue;
          }
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
              parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            result.successful = false;
            result.reason = name + " must be numeric";
            return result;
          }
          const double value =
              parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                  ? static_cast<double>(parameter.as_int())
                  : parameter.as_double();
          const bool is_force_compensation =
              name == "z_gravity_compensation_force" ||
              name == "positive_x_compensation_force";
          const double lower = is_force_compensation
                                   ? -50.0
                                   : (name == "translational_stiffness" ? 0.0 : -1.0);
          const double upper = is_force_compensation
                                   ? 50.0
                                   : (name == "translational_stiffness" ? 5000.0 : 10000.0);
          if (!std::isfinite(value) || value < lower || value > upper) {
            result.successful = false;
            result.reason = name + " must be finite and in [" + std::to_string(lower) + ", " +
                            std::to_string(upper) + "]";
            return result;
          }
        }
        for (const auto& parameter : parameters) {
          const auto& name = parameter.get_name();
          if (name == "translational_stiffness" ||
              name == "translational_stiffness_x" ||
              name == "translational_stiffness_y" ||
              name == "translational_stiffness_z" ||
              name == "z_gravity_compensation_force" ||
              name == "positive_x_compensation_force") {
            const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                                     ? static_cast<double>(parameter.as_int())
                                     : parameter.as_double();
            if (name == "translational_stiffness") {
              translational_stiffness_ = value;
              runtime_translational_stiffness_.store(value, std::memory_order_relaxed);
            } else if (name == "translational_stiffness_x") {
              runtime_translational_stiffness_x_.store(value, std::memory_order_relaxed);
            } else if (name == "translational_stiffness_y") {
              runtime_translational_stiffness_y_.store(value, std::memory_order_relaxed);
            } else if (name == "translational_stiffness_z") {
              runtime_translational_stiffness_z_.store(value, std::memory_order_relaxed);
            } else if (name == "z_gravity_compensation_force") {
              runtime_z_gravity_compensation_force_.store(value, std::memory_order_relaxed);
            } else if (name == "positive_x_compensation_force") {
              runtime_positive_x_compensation_force_.store(value, std::memory_order_relaxed);
            }
            RCLCPP_INFO(get_node()->get_logger(), "Runtime %s updated to %.3f",
                        name.c_str(), value);
          }
        }
        return result;
      });

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured SERL Cartesian impedance controller: arm_id=%s target_topic=%s "
              "reference_limit_mode=%s use_robot_state_q_dq=%d enable_nullspace_torque=%d "
              "filter_coeff=%.6f",
              arm_id_.c_str(), target_topic_.c_str(), reference_limit_mode_.c_str(),
              static_cast<int>(use_robot_state_q_dq_), static_cast<int>(enable_nullspace_torque_),
              filter_coeff_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SerlCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (command_interfaces_.size() != 7) {
    RCLCPP_FATAL(get_node()->get_logger(), "Expected 7 effort command interfaces, got %zu.",
                 command_interfaces_.size());
    return CallbackReturn::ERROR;
  }

  const auto resolved_joint_names = joint_names();
  std::ostringstream joint_log;
  joint_log << "joint_names used by controller:";
  for (size_t i = 0; i < resolved_joint_names.size(); ++i) {
    joint_log << "\n  [" << i << "] " << resolved_joint_names.at(i);
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", joint_log.str().c_str());

  std::ostringstream command_log;
  command_log << "claimed command interfaces:";
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_log << "\n  [" << i << "] name=" << command_interfaces_.at(i).get_name()
                << " interface=" << command_interfaces_.at(i).get_interface_name();
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", command_log.str().c_str());

  std::ostringstream state_log;
  state_log << "claimed state interfaces:";
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    state_log << "\n  [" << i << "] name=" << state_interfaces_.at(i).get_name()
              << " interface=" << state_interfaces_.at(i).get_interface_name();
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", state_log.str().c_str());

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  if (get_robot_state_ptr() == nullptr) {
    RCLCPP_FATAL(get_node()->get_logger(), "Cannot activate: robot_state interface is missing.");
    franka_robot_model_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  if (!resolve_state_interface_indices()) {
    franka_robot_model_->release_interfaces();
    return CallbackReturn::ERROR;
  }
  if (use_robot_state_q_dq_) {
    update_joint_state_from_robot_state(*get_robot_state_ptr());
  } else {
    update_joint_state_from_interfaces();
  }
  if (!read_measured_pose(measured_pose_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Cannot activate: measured pose is invalid.");
    franka_robot_model_->release_interfaces();
    return CallbackReturn::ERROR;
  }

  q_d_nullspace_ = q_;
  raw_target_ = measured_pose_;
  smoothed_target_ = measured_pose_;
  limited_reference_ = measured_pose_;
  smoothed_target_initialized_ = true;
  previous_raw_target_x_ = raw_target_.position.x();
  previous_raw_target_x_initialized_ = true;
  previous_raw_target_z_ = raw_target_.position.z();
  previous_raw_target_z_initialized_ = true;
  error_i_.setZero();
  last_tau_command_.setZero();
  debug_cartesian_error_.setZero();
  debug_wrench_.setZero();
  debug_jacobian_velocity_.setZero();
  debug_pose_diff_velocity_.setConstant(quiet_nan());
  debug_tau_task_.setZero();
  debug_tau_nullspace_.setZero();
  debug_coriolis_.setZero();
  debug_tau_before_saturation_.setZero();
  debug_tau_after_saturation_.setZero();
  debug_tau_j_.setZero();
  debug_tau_j_d_.setZero();
  debug_tau_ext_hat_filtered_.setZero();
  debug_o_f_ext_hat_k_.setZero();
  debug_k_f_ext_hat_k_.setZero();
  debug_wrench_est_.setZero();
  debug_wrench_est_error_.setZero();
  debug_o_t_ee_.fill(0.0);
  debug_zero_jacobian_.fill(0.0);
  previous_measured_position_ = measured_pose_.position;
  previous_measured_position_initialized_ = false;
  debug_velocity_direction_cosine_ = quiet_nan();
  debug_velocity_norm_ratio_ = quiet_nan();
  debug_velocity_diff_norm_ = quiet_nan();
  debug_tau_task_nullspace_dot_ = quiet_nan();
  debug_wrench_est_error_norm_ = quiet_nan();
  debug_positive_x_compensation_active_ = false;
  debug_z_gravity_compensation_active_ = false;
  active_ = true;
  target_received_ = false;
  target_update_count_ = 0;
  last_debug_publish_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(),
              "Activated SERL Cartesian impedance controller at measured pose (%.6f, %.6f, %.6f).",
              measured_pose_.position.x(), measured_pose_.position.y(), measured_pose_.position.z());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SerlCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  state_interface_indices_initialized_ = false;
  previous_measured_position_initialized_ = false;
  previous_raw_target_x_initialized_ = false;
  previous_raw_target_z_initialized_ = false;
  debug_positive_x_compensation_active_ = false;
  debug_z_gravity_compensation_active_ = false;
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

void SerlCartesianImpedanceController::target_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  PoseReference pose;
  if (!pose_msg_to_reference(*msg, pose)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Ignoring invalid target pose.");
    return;
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  raw_target_ = pose;
  last_target_time_ = msg->header.stamp.nanosec == 0 && msg->header.stamp.sec == 0
                          ? get_node()->now()
                          : rclcpp::Time(msg->header.stamp);
  target_received_ = true;
  target_update_count_++;
}

bool SerlCartesianImpedanceController::pose_msg_to_reference(
    const geometry_msgs::msg::PoseStamped& msg, PoseReference& pose) const {
  pose.position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  pose.orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                                        msg.pose.orientation.y, msg.pose.orientation.z);
  if (!pose.position.allFinite() || !std::isfinite(pose.orientation.norm()) ||
      pose.orientation.norm() < 1.0e-9) {
    return false;
  }
  pose.orientation.normalize();
  return true;
}

bool SerlCartesianImpedanceController::resolve_state_interface_indices() {
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

  std::ostringstream q_log;
  q_log << "state_interface_q_order:";
  for (size_t i = 0; i < resolved_joint_names.size(); ++i) {
    const size_t index = q_state_interface_indices_.at(i);
    q_log << "\n  [" << i << "] " << resolved_joint_names.at(i) << "/"
          << hardware_interface::HW_IF_POSITION << " -> state_interfaces_[" << index
          << "] name=" << state_interfaces_.at(index).get_name()
          << " interface=" << state_interfaces_.at(index).get_interface_name();
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", q_log.str().c_str());

  std::ostringstream dq_log;
  dq_log << "state_interface_dq_order:";
  for (size_t i = 0; i < resolved_joint_names.size(); ++i) {
    const size_t index = dq_state_interface_indices_.at(i);
    dq_log << "\n  [" << i << "] " << resolved_joint_names.at(i) << "/"
           << hardware_interface::HW_IF_VELOCITY << " -> state_interfaces_[" << index
           << "] name=" << state_interfaces_.at(index).get_name()
           << " interface=" << state_interfaces_.at(index).get_interface_name();
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", dq_log.str().c_str());

  std::ostringstream effort_log;
  effort_log << "command_interface_effort_order:";
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    effort_log << "\n  [" << i << "] " << command_interfaces_.at(i).get_name()
               << " interface=" << command_interfaces_.at(i).get_interface_name();
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", effort_log.str().c_str());

  std::ostringstream jacobian_log;
  jacobian_log << "jacobian_columns_joint_order:";
  for (size_t i = 0; i < resolved_joint_names.size(); ++i) {
    jacobian_log << "\n  [" << i << "] " << resolved_joint_names.at(i);
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", jacobian_log.str().c_str());

  return true;
}

void SerlCartesianImpedanceController::update_joint_state_from_robot_state(
    const franka::RobotState& robot_state) {
  Eigen::Map<const Vector7d> q(robot_state.q.data());
  Eigen::Map<const Vector7d> dq(robot_state.dq.data());
  q_ = q;
  dq_ = dq;
}

void SerlCartesianImpedanceController::update_joint_state_from_interfaces() {
  if (!state_interface_indices_initialized_) {
    throw std::runtime_error("joint state interface indices were not resolved before update");
  }
  for (size_t i = 0; i < 7; ++i) {
    q_(i) = state_interfaces_.at(q_state_interface_indices_.at(i)).get_value();
    dq_(i) = state_interfaces_.at(dq_state_interface_indices_.at(i)).get_value();
  }
}

franka::RobotState* SerlCartesianImpedanceController::get_robot_state_ptr() const {
  const std::string state_interface_name = arm_id_ + "/robot_state";
  auto state_interface = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&](const auto& interface) {
        return interface.get_name() == state_interface_name;
      });
  if (state_interface == state_interfaces_.cend()) {
    return nullptr;
  }
  return bit_cast<franka::RobotState*>(state_interface->get_value());
}

bool SerlCartesianImpedanceController::read_measured_pose(PoseReference& pose) const {
  const franka::RobotState* robot_state = get_robot_state_ptr();
  if (robot_state == nullptr) {
    return false;
  }
  const Eigen::Map<const Eigen::Matrix4d> transform_matrix(robot_state->O_T_EE.data());
  const Eigen::Affine3d transform(transform_matrix);
  pose.position = transform.translation();
  pose.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return pose.position.allFinite() && std::isfinite(pose.orientation.norm());
}

SerlCartesianImpedanceController::PoseReference
SerlCartesianImpedanceController::limit_reference(const PoseReference& smoothed_target,
                                                  const PoseReference& measured) {
  PoseReference limited = smoothed_target;
  reference_was_clipped_ = false;

  if (reference_limit_mode_ == "per_axis_error_clip") {
    const Eigen::Matrix<double, 6, 1> raw_error =
        compute_cartesian_error(measured, smoothed_target);
    Eigen::Matrix<double, 6, 1> clipped_error = raw_error;
    clip_cartesian_error(clipped_error);

    position_error_before_clip_ = raw_error.head(3).norm();
    position_error_after_clip_ = clipped_error.head(3).norm();
    orientation_error_before_clip_ = raw_error.tail(3).norm();
    orientation_error_after_clip_ = clipped_error.tail(3).norm();
    reference_was_clipped_ = (raw_error - clipped_error).norm() > 1.0e-12;

    limited.position = measured.position - clipped_error.head(3);
    return limited;
  }

  const Eigen::Vector3d position_error = smoothed_target.position - measured.position;
  position_error_before_clip_ = position_error.norm();
  if (position_error_before_clip_ > max_pos_error_ && position_error_before_clip_ > 1.0e-12) {
    limited.position =
        measured.position + max_pos_error_ * position_error / position_error_before_clip_;
    reference_was_clipped_ = true;
  }
  position_error_after_clip_ = (limited.position - measured.position).norm();

  Eigen::Quaterniond target_orientation = smoothed_target.orientation;
  if (measured.orientation.coeffs().dot(target_orientation.coeffs()) < 0.0) {
    target_orientation.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond orientation_delta = measured.orientation.inverse() * target_orientation;
  orientation_error_before_clip_ = quaternion_angle(orientation_delta);
  if (orientation_error_before_clip_ > max_ori_error_ && orientation_error_before_clip_ > 1.0e-12) {
    const double ratio = max_ori_error_ / orientation_error_before_clip_;
    limited.orientation = measured.orientation.slerp(ratio, target_orientation).normalized();
    reference_was_clipped_ = true;
  } else {
    limited.orientation = target_orientation.normalized();
  }
  orientation_error_after_clip_ =
      quaternion_angle(measured.orientation.inverse() * limited.orientation);
  return limited;
}

Eigen::Matrix<double, 6, 1> SerlCartesianImpedanceController::compute_cartesian_error(
    const PoseReference& measured, const PoseReference& reference) const {
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) = measured.position - reference.position;

  Eigen::Quaterniond measured_orientation = measured.orientation;
  Eigen::Quaterniond reference_orientation = reference.orientation;
  if (reference_orientation.coeffs().dot(measured_orientation.coeffs()) < 0.0) {
    measured_orientation.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond error_quaternion =
      measured_orientation.inverse() * reference_orientation;
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) = -measured_orientation.toRotationMatrix() * error.tail(3);
  return error;
}

void SerlCartesianImpedanceController::clip_cartesian_error(
    Eigen::Matrix<double, 6, 1>& error) const {
  for (int i = 0; i < 3; ++i) {
    error(i) = std::clamp(error(i), translational_clip_min_(i), translational_clip_max_(i));
    error(i + 3) = std::clamp(error(i + 3), rotational_clip_min_(i), rotational_clip_max_(i));
  }
}

SerlCartesianImpedanceController::Vector7d
SerlCartesianImpedanceController::saturate_torque_rate(const Vector7d& tau_d_calculated,
                                                       const Vector7d& tau_reference) {
  Vector7d tau_d_saturated;
  tau_rate_limited_ = false;
  for (size_t i = 0; i < 7; ++i) {
    const double difference = tau_d_calculated(i) - tau_reference(i);
    const double limited_difference =
        std::clamp(difference, -torque_rate_limit_, torque_rate_limit_);
    tau_d_saturated(i) = tau_reference(i) + limited_difference;
    tau_rate_limited_ = tau_rate_limited_ || std::abs(difference - limited_difference) > 1.0e-12;
  }
  return tau_d_saturated;
}

void SerlCartesianImpedanceController::publish_pose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
    const rclcpp::Time& time,
    const PoseReference& pose) {
  if (pub) {
    pub->publish(make_pose_msg(time, "base", pose.position, pose.orientation));
  }
}

void SerlCartesianImpedanceController::maybe_publish_debug(const rclcpp::Time& time) {
  if (debug_publish_rate_ <= 0.0) {
    return;
  }
  const double period_s = 1.0 / debug_publish_rate_;
  if (last_debug_publish_time_.nanoseconds() > 0 &&
      (time - last_debug_publish_time_).seconds() < period_s) {
    return;
  }
  last_debug_publish_time_ = time;
  publish_pose(raw_target_pub_, time, raw_target_);
  publish_pose(smoothed_target_pub_, time, smoothed_target_);
  publish_pose(limited_reference_pub_, time, limited_reference_);
  publish_pose(measured_pose_pub_, time, measured_pose_);
  publish_status(time);
}

void SerlCartesianImpedanceController::publish_status(const rclcpp::Time& time) {
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  std::ostringstream out;
  out << "controller_active=" << static_cast<int>(active_)
      << " target_received=" << static_cast<int>(target_received_)
      << " target_update_count=" << target_update_count_ << " target_age_s=" << target_age_s_
      << " reference_was_clipped=" << static_cast<int>(reference_was_clipped_)
      << " reference_clipped=" << static_cast<int>(reference_was_clipped_)
      << " target_distance_clamped=0"
      << " use_robot_state_q_dq=" << static_cast<int>(use_robot_state_q_dq_)
      << " enable_nullspace_torque=" << static_cast<int>(enable_nullspace_torque_)
      << " control_law_mode=raw_target_no_filter_no_cartesian_clip"
      << " reference_limit_mode=" << reference_limit_mode_
      << " desired_speed_limited=0"
      << " desired_acceleration_limited=0"
      << " jerk_limited=0"
      << " step_limited=0"
      << " position_error_before_clip=" << position_error_before_clip_
      << " position_error_after_clip=" << position_error_after_clip_
      << " max_pos_error=" << max_pos_error_
      << " orientation_error_before_clip=" << orientation_error_before_clip_
      << " orientation_error_after_clip=" << orientation_error_after_clip_
      << " max_ori_error=" << max_ori_error_
      << " translational_clip_min_x=" << translational_clip_min_.x()
      << " translational_clip_min_y=" << translational_clip_min_.y()
      << " translational_clip_min_z=" << translational_clip_min_.z()
      << " translational_clip_max_x=" << translational_clip_max_.x()
      << " translational_clip_max_y=" << translational_clip_max_.y()
      << " translational_clip_max_z=" << translational_clip_max_.z()
      << " rotational_clip_min_x=" << rotational_clip_min_.x()
      << " rotational_clip_min_y=" << rotational_clip_min_.y()
      << " rotational_clip_min_z=" << rotational_clip_min_.z()
      << " rotational_clip_max_x=" << rotational_clip_max_.x()
      << " rotational_clip_max_y=" << rotational_clip_max_.y()
      << " rotational_clip_max_z=" << rotational_clip_max_.z()
      << " tau_norm=" << tau_norm_
      << " commanded_torque_norm=" << tau_norm_
      << " tau_rate_limited=" << static_cast<int>(tau_rate_limited_)
      << " torque_rate_limited=" << static_cast<int>(tau_rate_limited_)
      << " t=" << time.seconds()
      << " dt=" << update_period_s_
      << " update_period=" << update_period_s_
      << " update_period_s=" << update_period_s_
      << " translational_stiffness="
      << runtime_translational_stiffness_.load(std::memory_order_relaxed)
      << " translational_stiffness_x="
      << (runtime_translational_stiffness_x_.load(std::memory_order_relaxed) > 0.0
              ? runtime_translational_stiffness_x_.load(std::memory_order_relaxed)
              : runtime_translational_stiffness_.load(std::memory_order_relaxed))
      << " translational_stiffness_y="
      << (runtime_translational_stiffness_y_.load(std::memory_order_relaxed) > 0.0
              ? runtime_translational_stiffness_y_.load(std::memory_order_relaxed)
              : runtime_translational_stiffness_.load(std::memory_order_relaxed))
      << " translational_stiffness_z="
      << (runtime_translational_stiffness_z_.load(std::memory_order_relaxed) > 0.0
              ? runtime_translational_stiffness_z_.load(std::memory_order_relaxed)
              : runtime_translational_stiffness_.load(std::memory_order_relaxed))
      << " z_gravity_compensation_force="
      << runtime_z_gravity_compensation_force_.load(std::memory_order_relaxed)
      << " z_gravity_compensation_active="
      << static_cast<int>(debug_z_gravity_compensation_active_)
      << " positive_x_compensation_force="
      << runtime_positive_x_compensation_force_.load(std::memory_order_relaxed)
      << " positive_x_compensation_active="
      << static_cast<int>(debug_positive_x_compensation_active_)
      << " translational_damping=" << translational_damping_
      << " rotational_stiffness=" << rotational_stiffness_
      << " rotational_damping=" << rotational_damping_
      << " filter_coeff=" << filter_coeff_
      << " measured_x=" << measured_pose_.position.x()
      << " measured_y=" << measured_pose_.position.y()
      << " measured_z=" << measured_pose_.position.z()
      << " limited_x=" << limited_reference_.position.x()
      << " limited_y=" << limited_reference_.position.y()
      << " limited_z=" << limited_reference_.position.z()
      << " position_error_x=" << debug_cartesian_error_.x()
      << " position_error_y=" << debug_cartesian_error_.y()
      << " position_error_z=" << debug_cartesian_error_.z()
      << " cartesian_force_x=" << debug_wrench_.x()
      << " cartesian_force_y=" << debug_wrench_.y()
      << " cartesian_force_z=" << debug_wrench_.z()
      << " cartesian_torque_x=" << debug_wrench_(3)
      << " cartesian_torque_y=" << debug_wrench_(4)
      << " cartesian_torque_z=" << debug_wrench_(5)
      << " desired_wrench_x=" << debug_wrench_(0)
      << " desired_wrench_y=" << debug_wrench_(1)
      << " desired_wrench_z=" << debug_wrench_(2)
      << " desired_wrench_torque_x=" << debug_wrench_(3)
      << " desired_wrench_torque_y=" << debug_wrench_(4)
      << " desired_wrench_torque_z=" << debug_wrench_(5)
      << " wrench_est_x=" << debug_wrench_est_(0)
      << " wrench_est_y=" << debug_wrench_est_(1)
      << " wrench_est_z=" << debug_wrench_est_(2)
      << " wrench_est_torque_x=" << debug_wrench_est_(3)
      << " wrench_est_torque_y=" << debug_wrench_est_(4)
      << " wrench_est_torque_z=" << debug_wrench_est_(5)
      << " wrench_est_error_x=" << debug_wrench_est_error_(0)
      << " wrench_est_error_y=" << debug_wrench_est_error_(1)
      << " wrench_est_error_z=" << debug_wrench_est_error_(2)
      << " wrench_est_error_torque_x=" << debug_wrench_est_error_(3)
      << " wrench_est_error_torque_y=" << debug_wrench_est_error_(4)
      << " wrench_est_error_torque_z=" << debug_wrench_est_error_(5)
      << " wrench_est_error_norm=" << debug_wrench_est_error_norm_
      << " dot_tau_task_tau_nullspace=" << debug_tau_task_nullspace_dot_
      << " jacobian_velocity_x=" << debug_jacobian_velocity_.x()
      << " jacobian_velocity_y=" << debug_jacobian_velocity_.y()
      << " jacobian_velocity_z=" << debug_jacobian_velocity_.z()
      << " jacobian_velocity_angular_x=" << debug_jacobian_velocity_(3)
      << " jacobian_velocity_angular_y=" << debug_jacobian_velocity_(4)
      << " jacobian_velocity_angular_z=" << debug_jacobian_velocity_(5)
      << " cartesian_velocity_from_jacobian_x=" << debug_jacobian_velocity_.x()
      << " cartesian_velocity_from_jacobian_y=" << debug_jacobian_velocity_.y()
      << " cartesian_velocity_from_jacobian_z=" << debug_jacobian_velocity_.z()
      << " pose_diff_velocity_x=" << debug_pose_diff_velocity_.x()
      << " pose_diff_velocity_y=" << debug_pose_diff_velocity_.y()
      << " pose_diff_velocity_z=" << debug_pose_diff_velocity_.z()
      << " measured_velocity_from_pose_diff_x=" << debug_pose_diff_velocity_.x()
      << " measured_velocity_from_pose_diff_y=" << debug_pose_diff_velocity_.y()
      << " measured_velocity_from_pose_diff_z=" << debug_pose_diff_velocity_.z()
      << " velocity_direction_cosine=" << debug_velocity_direction_cosine_
      << " velocity_norm_ratio=" << debug_velocity_norm_ratio_
      << " velocity_diff_norm=" << debug_velocity_diff_norm_;
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_task_" << (i + 1) << "=" << debug_tau_task_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_nullspace_" << (i + 1) << "=" << debug_tau_nullspace_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " torque_task_" << (i + 1) << "=" << debug_tau_task_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " coriolis_" << (i + 1) << "=" << debug_coriolis_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_before_saturation_" << (i + 1) << "=" << debug_tau_before_saturation_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_after_saturation_" << (i + 1) << "=" << debug_tau_after_saturation_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_command_" << (i + 1) << "=" << last_tau_command_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_J_" << (i + 1) << "=" << debug_tau_j_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_J_d_" << (i + 1) << "=" << debug_tau_j_d_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " tau_ext_hat_filtered_" << (i + 1) << "=" << debug_tau_ext_hat_filtered_(i);
  }
  for (size_t i = 0; i < 6; ++i) {
    out << " O_F_ext_hat_K_" << i << "=" << debug_o_f_ext_hat_k_(i);
  }
  for (size_t i = 0; i < 6; ++i) {
    out << " K_F_ext_hat_K_" << i << "=" << debug_k_f_ext_hat_k_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " q_" << (i + 1) << "=" << q_(i);
  }
  for (size_t i = 0; i < 7; ++i) {
    out << " dq_" << (i + 1) << "=" << dq_(i);
  }
  for (size_t i = 0; i < debug_o_t_ee_.size(); ++i) {
    out << " O_T_EE_" << i << "=" << debug_o_t_ee_.at(i);
  }
  for (size_t i = 0; i < debug_zero_jacobian_.size(); ++i) {
    out << " zero_jacobian_" << i << "=" << debug_zero_jacobian_.at(i);
  }
  out << " stamp_sec=" << time.seconds();
  msg.data = out.str();
  status_pub_->publish(msg);
}

std::vector<std::string> SerlCartesianImpedanceController::joint_names() const {
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

PLUGINLIB_EXPORT_CLASS(serl_franka_ros2_control::SerlCartesianImpedanceController,
                       controller_interface::ControllerInterface)
