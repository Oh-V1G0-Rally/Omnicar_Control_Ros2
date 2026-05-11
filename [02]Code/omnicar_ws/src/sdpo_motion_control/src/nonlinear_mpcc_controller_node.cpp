#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdpo_motion_control/msg/path_follower_debug.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace sdpo_motion_control
{

class NonlinearMpccControllerNode : public rclcpp::Node
{
public:
  NonlinearMpccControllerNode()
  : Node("nonlinear_mpcc_controller")
  {
    control_frame_id_ = declare_parameter<std::string>("control_frame_id", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "pose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    error_topic_ = declare_parameter<std::string>("error_topic", "mpcc_error");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "mpcc_debug");
    path_file_ = declare_parameter<std::string>("path_file", "");
    path_is_closed_ = declare_parameter<bool>("path_is_closed", true);
    publish_debug_topics_ = declare_parameter<bool>("publish_debug_topics", true);
    publish_zero_when_inactive_ = declare_parameter<bool>("publish_zero_when_inactive", true);
    stop_at_end_ = declare_parameter<bool>("stop_at_end", false);
    heading_mode_ = declare_parameter<std::string>("heading_mode", "fixed");

    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 30.0);
    debug_publish_rate_hz_ = declare_parameter<double>("debug_publish_rate_hz", 20.0);
    prediction_horizon_ = declare_parameter<int>("prediction_horizon", 12);
    prediction_dt_ = declare_parameter<double>("prediction_dt", 0.08);
    fixed_heading_ = declare_parameter<double>("fixed_heading", 0.0);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.10);

    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.25);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.25);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.50);
    max_gamma_dot_ = declare_parameter<double>("max_gamma_dot", 0.30);
    min_gamma_dot_ = declare_parameter<double>("min_gamma_dot", 0.0);
    max_accel_x_ = declare_parameter<double>("max_accel_x", 0.60);
    max_accel_y_ = declare_parameter<double>("max_accel_y", 0.60);
    max_accel_w_ = declare_parameter<double>("max_accel_w", 1.20);
    max_gamma_accel_ = declare_parameter<double>("max_gamma_accel", 0.80);

    q_contour_ = declare_parameter<double>("q_contour", 20.0);
    q_lag_ = declare_parameter<double>("q_lag", 2.0);
    q_yaw_ = declare_parameter<double>("q_yaw", 1.0);
    q_terminal_contour_ = declare_parameter<double>("q_terminal_contour", 30.0);
    q_terminal_lag_ = declare_parameter<double>("q_terminal_lag", 4.0);
    q_progress_ = declare_parameter<double>("q_progress", 0.40);
    r_vx_ = declare_parameter<double>("r_vx", 0.15);
    r_vy_ = declare_parameter<double>("r_vy", 0.15);
    r_w_ = declare_parameter<double>("r_w", 0.10);
    r_gamma_dot_ = declare_parameter<double>("r_gamma_dot", 0.05);
    r_delta_vx_ = declare_parameter<double>("r_delta_vx", 0.50);
    r_delta_vy_ = declare_parameter<double>("r_delta_vy", 0.50);
    r_delta_w_ = declare_parameter<double>("r_delta_w", 0.20);
    r_delta_gamma_dot_ = declare_parameter<double>("r_delta_gamma_dot", 0.10);

    vx_samples_ = declare_parameter<int>("vx_samples", 5);
    vy_samples_ = declare_parameter<int>("vy_samples", 5);
    w_samples_ = declare_parameter<int>("w_samples", 5);
    gamma_dot_samples_ = declare_parameter<int>("gamma_dot_samples", 5);

    if (prediction_horizon_ < 1) {
      throw std::runtime_error("prediction_horizon must be >= 1");
    }
    if (prediction_dt_ <= 0.0) {
      throw std::runtime_error("prediction_dt must be > 0");
    }
    if (heading_mode_ != "fixed" && heading_mode_ != "tangent" && heading_mode_ != "path") {
      throw std::runtime_error("heading_mode must be 'fixed', 'tangent', or 'path'");
    }

    loadPath(path_file_);

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    if (publish_debug_topics_) {
      pub_error_ = create_publisher<geometry_msgs::msg::Twist>(error_topic_, 10);
      pub_debug_ = create_publisher<sdpo_motion_control::msg::PathFollowerDebug>(debug_topic_, 10);
    }

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&NonlinearMpccControllerNode::onPose, this, std::placeholders::_1));

    last_update_time_ = now();
    last_debug_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0))),
      std::bind(&NonlinearMpccControllerNode::onTimer, this));
  }

private:
  struct PathPoint
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool has_yaw = false;
  };

  struct PathSample
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool has_yaw = false;
    double tangent_x = 1.0;
    double tangent_y = 0.0;
    double normal_x = 0.0;
    double normal_y = 1.0;
  };

  struct Projection
  {
    double gamma = 0.0;
    double distance_sq = std::numeric_limits<double>::infinity();
  };

  struct State
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double gamma = 0.0;
  };

  struct Control
  {
    double vx = 0.0;
    double vy = 0.0;
    double w = 0.0;
    double gamma_dot = 0.0;
  };

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_frame_id_ = msg->header.frame_id;
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_yaw_ = normalizeAngle(tf2::getYaw(msg->pose.orientation));
    pose_received_ = true;
  }

  void onTimer()
  {
    const auto stamp = now();
    double dt = (stamp - last_update_time_).seconds();
    last_update_time_ = stamp;
    if (dt <= 0.0) {
      dt = 1e-3;
    }

    if (!pose_received_) {
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(stamp, "waiting_for_pose", "waiting_for_pose", Control(), PathSample());
      return;
    }

    if (!control_frame_id_.empty() && !current_frame_id_.empty() &&
      current_frame_id_ != control_frame_id_)
    {
      const std::string stop_reason =
        "current_pose_frame_mismatch: current_frame_id='" + current_frame_id_ +
        "', expected='" + control_frame_id_ + "'";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stopping MPCC: current pose frame '%s' does not match control frame '%s'.",
        current_frame_id_.c_str(), control_frame_id_.c_str());
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(stamp, "stopped", stop_reason, Control(), PathSample());
      return;
    }

    if (!initialized_) {
      gamma_ = nearestGamma(current_x_, current_y_);
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "Nonlinear MPCC initialized at gamma %.3f m.", gamma_);
    }

    if (!path_is_closed_ && stop_at_end_ && gamma_ >= total_length_) {
      publishZero();
      publishDebug(stamp, "finished", "", Control(), sampleAt(total_length_));
      return;
    }

    const Control best = optimizeControl(dt);
    publishCommand(stamp, dt, best);
  }

  Control optimizeControl(double control_dt) const
  {
    const double dt = std::max(control_dt, 1e-3);
    const std::vector<double> vx_values =
      sampleRangeLimited(last_cmd_.vx, -max_linear_x_, max_linear_x_, max_accel_x_ * dt, vx_samples_);
    const std::vector<double> vy_values =
      sampleRangeLimited(last_cmd_.vy, -max_linear_y_, max_linear_y_, max_accel_y_ * dt, vy_samples_);
    const std::vector<double> w_values =
      sampleRangeLimited(last_cmd_.w, -max_angular_z_, max_angular_z_, max_accel_w_ * dt, w_samples_);
    const std::vector<double> gamma_dot_values = sampleRangeLimited(
      last_cmd_.gamma_dot, min_gamma_dot_, max_gamma_dot_, max_gamma_accel_ * dt,
      gamma_dot_samples_);

    Control best;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const double vx : vx_values) {
      for (const double vy : vy_values) {
        for (const double w : w_values) {
          for (const double gamma_dot : gamma_dot_values) {
            Control candidate;
            candidate.vx = vx;
            candidate.vy = vy;
            candidate.w = w;
            candidate.gamma_dot = gamma_dot;
            const double cost = rolloutCost(candidate);
            if (cost < best_cost) {
              best_cost = cost;
              best = candidate;
            }
          }
        }
      }
    }
    return best;
  }

  double rolloutCost(const Control & u) const
  {
    State state;
    state.x = current_x_;
    state.y = current_y_;
    state.yaw = current_yaw_;
    state.gamma = gamma_;

    double cost = commandCost(u, last_cmd_);
    for (int i = 0; i < prediction_horizon_; ++i) {
      state = integrate(state, u, prediction_dt_);
      const PathSample ref = sampleAt(state.gamma);
      const double dx = state.x - ref.x;
      const double dy = state.y - ref.y;
      const double contour_error = dx * ref.normal_x + dy * ref.normal_y;
      const double lag_error = dx * ref.tangent_x + dy * ref.tangent_y;
      const double yaw_error = normalizeAngle(desiredYaw(state.gamma, ref) - state.yaw);
      cost += q_contour_ * contour_error * contour_error;
      cost += q_lag_ * lag_error * lag_error;
      cost += q_yaw_ * yaw_error * yaw_error;
      cost -= q_progress_ * u.gamma_dot * prediction_dt_;
    }

    const PathSample terminal = sampleAt(state.gamma);
    const double terminal_dx = state.x - terminal.x;
    const double terminal_dy = state.y - terminal.y;
    const double terminal_contour = terminal_dx * terminal.normal_x + terminal_dy * terminal.normal_y;
    const double terminal_lag = terminal_dx * terminal.tangent_x + terminal_dy * terminal.tangent_y;
    cost += q_terminal_contour_ * terminal_contour * terminal_contour;
    cost += q_terminal_lag_ * terminal_lag * terminal_lag;
    return cost;
  }

  double commandCost(const Control & u, const Control & previous) const
  {
    const double dvx = u.vx - previous.vx;
    const double dvy = u.vy - previous.vy;
    const double dw = u.w - previous.w;
    const double dgamma_dot = u.gamma_dot - previous.gamma_dot;
    return r_vx_ * u.vx * u.vx +
           r_vy_ * u.vy * u.vy +
           r_w_ * u.w * u.w +
           r_gamma_dot_ * u.gamma_dot * u.gamma_dot +
           r_delta_vx_ * dvx * dvx +
           r_delta_vy_ * dvy * dvy +
           r_delta_w_ * dw * dw +
           r_delta_gamma_dot_ * dgamma_dot * dgamma_dot;
  }

  State integrate(const State & state, const Control & u, double dt) const
  {
    State next = state;
    const double cos_yaw = std::cos(state.yaw);
    const double sin_yaw = std::sin(state.yaw);
    next.x += (u.vx * cos_yaw - u.vy * sin_yaw) * dt;
    next.y += (u.vx * sin_yaw + u.vy * cos_yaw) * dt;
    next.yaw = normalizeAngle(next.yaw + u.w * dt);
    next.gamma = normalizePathGamma(next.gamma + u.gamma_dot * dt);
    return next;
  }

  void publishCommand(const rclcpp::Time & stamp, double dt, const Control & command)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = clamp(command.vx, -max_linear_x_, max_linear_x_);
    msg.linear.y = clamp(command.vy, -max_linear_y_, max_linear_y_);
    msg.angular.z = clamp(command.w, -max_angular_z_, max_angular_z_);
    pub_cmd_vel_->publish(msg);

    gamma_ = normalizePathGamma(gamma_ + command.gamma_dot * std::max(dt, 1e-3));
    last_cmd_ = command;
    const PathSample target = sampleAt(gamma_);
    updateLastErrors(target);
    publishDebug(stamp, "mpcc_tracking", "", command, target);
  }

  void updateLastErrors(const PathSample & target)
  {
    last_target_ = target;
    last_target_yaw_ = desiredYaw(gamma_, target);
    last_ex_world_ = target.x - current_x_;
    last_ey_world_ = target.y - current_y_;
    const double dx = current_x_ - target.x;
    const double dy = current_y_ - target.y;
    last_contour_error_ = dx * target.normal_x + dy * target.normal_y;
    last_lag_error_ = dx * target.tangent_x + dy * target.tangent_y;
    last_yaw_error_ = normalizeAngle(last_target_yaw_ - current_yaw_);
    worldToBody(last_ex_world_, last_ey_world_, last_ex_body_, last_ey_body_);
  }

  void publishDebug(
    const rclcpp::Time & stamp, const std::string & state_name, const std::string & stop_reason,
    const Control & command, const PathSample & target)
  {
    if (!publish_debug_topics_) {
      return;
    }

    geometry_msgs::msg::Twist error_msg;
    error_msg.linear.x = last_contour_error_;
    error_msg.linear.y = last_lag_error_;
    error_msg.angular.z = last_yaw_error_;
    pub_error_->publish(error_msg);

    if (!shouldPublishStructuredDebug(stamp)) {
      return;
    }

    sdpo_motion_control::msg::PathFollowerDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = control_frame_id_;
    msg.active = pose_received_ && stop_reason.empty();
    msg.state = state_name;
    msg.control_frame_id = control_frame_id_;
    msg.current_frame_id = current_frame_id_;
    msg.stop_reason = stop_reason;
    msg.current_x = current_x_;
    msg.current_y = current_y_;
    msg.current_yaw = current_yaw_;
    msg.target_x = target.x;
    msg.target_y = target.y;
    msg.target_yaw = desiredYaw(gamma_, target);
    msg.tangent_x = target.tangent_x;
    msg.tangent_y = target.tangent_y;
    msg.gamma = gamma_;
    msg.gamma_dot = command.gamma_dot;
    msg.error_x_world = last_ex_world_;
    msg.error_y_world = last_ey_world_;
    msg.error_x_body = last_ex_body_;
    msg.error_y_body = last_ey_body_;
    msg.error_along = last_lag_error_;
    msg.error_yaw = last_yaw_error_;
    msg.feedforward_vx = command.vx;
    msg.feedforward_vy = command.vy;
    msg.cmd_vx_raw = command.vx;
    msg.cmd_vy_raw = command.vy;
    msg.cmd_w_raw = command.w;
    msg.cmd_vx = clamp(command.vx, -max_linear_x_, max_linear_x_);
    msg.cmd_vy = clamp(command.vy, -max_linear_y_, max_linear_y_);
    msg.cmd_w = clamp(command.w, -max_angular_z_, max_angular_z_);
    msg.saturated_vx = isSaturated(command.vx, msg.cmd_vx);
    msg.saturated_vy = isSaturated(command.vy, msg.cmd_vy);
    msg.saturated_w = isSaturated(command.w, msg.cmd_w);
    pub_debug_->publish(msg);
    last_debug_publish_time_ = stamp;
  }

  void loadPath(const std::string & path_file)
  {
    if (path_file.empty()) {
      throw std::runtime_error("path_file parameter must not be empty");
    }

    std::ifstream file(path_file);
    if (!file.is_open()) {
      throw std::runtime_error("failed to open path_file: " + path_file);
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::replace(line.begin(), line.end(), ';', ',');
      std::stringstream ss(line);
      std::string token;
      std::vector<double> values;
      bool numeric_row = true;
      while (std::getline(ss, token, ',')) {
        try {
          size_t parsed = 0;
          const double value = std::stod(token, &parsed);
          if (parsed == 0) {
            numeric_row = false;
            break;
          }
          values.push_back(value);
        } catch (const std::exception &) {
          numeric_row = false;
          break;
        }
      }

      if (!numeric_row || values.size() < 2) {
        continue;
      }

      PathPoint point;
      point.x = values[0];
      point.y = values[1];
      if (values.size() >= 3) {
        point.yaw = values[2];
        point.has_yaw = true;
      }
      points_.push_back(point);
    }

    if (points_.size() < 2) {
      throw std::runtime_error("path_file must contain at least two numeric x,y points");
    }

    cumulative_s_.clear();
    cumulative_s_.push_back(0.0);
    for (size_t i = 1; i < points_.size(); ++i) {
      const double ds = std::hypot(points_[i].x - points_[i - 1].x, points_[i].y - points_[i - 1].y);
      if (ds <= 1e-9) {
        throw std::runtime_error("path_file contains repeated consecutive points");
      }
      cumulative_s_.push_back(cumulative_s_.back() + ds);
    }
    total_length_ = cumulative_s_.back();
    RCLCPP_INFO(
      get_logger(), "Loaded %zu path points from '%s' with total length %.3f m.",
      points_.size(), path_file.c_str(), total_length_);
  }

  PathSample sampleAt(double gamma) const
  {
    if (total_length_ <= 0.0) {
      return PathSample();
    }

    gamma = normalizePathGamma(gamma);
    auto upper = std::upper_bound(cumulative_s_.begin(), cumulative_s_.end(), gamma);
    size_t idx = 0;
    if (upper == cumulative_s_.begin()) {
      idx = 0;
    } else if (upper == cumulative_s_.end()) {
      idx = cumulative_s_.size() - 2;
    } else {
      idx = static_cast<size_t>(std::distance(cumulative_s_.begin(), upper) - 1);
    }

    const PathPoint & a = points_[idx];
    const PathPoint & b = points_[idx + 1];
    const double segment_length = cumulative_s_[idx + 1] - cumulative_s_[idx];
    const double ratio = segment_length > 1e-9 ? (gamma - cumulative_s_[idx]) / segment_length : 0.0;

    PathSample sample;
    sample.x = a.x + ratio * (b.x - a.x);
    sample.y = a.y + ratio * (b.y - a.y);
    sample.tangent_x = (b.x - a.x) / segment_length;
    sample.tangent_y = (b.y - a.y) / segment_length;
    sample.normal_x = -sample.tangent_y;
    sample.normal_y = sample.tangent_x;
    sample.has_yaw = a.has_yaw && b.has_yaw;
    if (sample.has_yaw) {
      sample.yaw = normalizeAngle(a.yaw + ratio * normalizeAngle(b.yaw - a.yaw));
    } else {
      sample.yaw = std::atan2(sample.tangent_y, sample.tangent_x);
    }
    return sample;
  }

  double desiredYaw(double gamma, const PathSample & sample) const
  {
    if (heading_mode_ == "fixed") {
      return fixed_heading_;
    }
    if (heading_mode_ == "path" && sample.has_yaw) {
      return sample.yaw;
    }
    const PathSample ahead = sampleAt(gamma + std::max(lookahead_distance_, 1e-6));
    return std::atan2(ahead.y - sample.y, ahead.x - sample.x);
  }

  double nearestGamma(double x, double y) const
  {
    Projection best;
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const PathPoint & a = points_[i];
      const PathPoint & b = points_[i + 1];
      const double ab_x = b.x - a.x;
      const double ab_y = b.y - a.y;
      const double ab_len_sq = ab_x * ab_x + ab_y * ab_y;
      if (ab_len_sq <= 1e-12) {
        continue;
      }
      const double ap_x = x - a.x;
      const double ap_y = y - a.y;
      const double u = clamp((ap_x * ab_x + ap_y * ab_y) / ab_len_sq, 0.0, 1.0);
      const double proj_x = a.x + u * ab_x;
      const double proj_y = a.y + u * ab_y;
      const double dx = x - proj_x;
      const double dy = y - proj_y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < best.distance_sq) {
        best.distance_sq = distance_sq;
        best.gamma = cumulative_s_[i] + u * std::sqrt(ab_len_sq);
      }
    }
    return normalizePathGamma(best.gamma);
  }

  std::vector<double> sampleRangeLimited(
    double center, double min_value, double max_value, double max_step, int sample_count) const
  {
    sample_count = std::max(sample_count, 1);
    const double low = clamp(center - max_step, min_value, max_value);
    const double high = clamp(center + max_step, min_value, max_value);
    std::vector<double> values;
    values.reserve(static_cast<size_t>(sample_count));
    if (sample_count == 1 || std::abs(high - low) <= 1e-12) {
      values.push_back(clamp(center, min_value, max_value));
      return values;
    }
    for (int i = 0; i < sample_count; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(sample_count - 1);
      values.push_back(low + ratio * (high - low));
    }
    return values;
  }

  double normalizePathGamma(double gamma) const
  {
    if (path_is_closed_ && total_length_ > 0.0) {
      while (gamma < 0.0) {
        gamma += total_length_;
      }
      while (gamma >= total_length_) {
        gamma -= total_length_;
      }
      return gamma;
    }
    return clamp(gamma, 0.0, total_length_);
  }

  void worldToBody(double world_x, double world_y, double & body_x, double & body_y) const
  {
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    body_x = cos_yaw * world_x + sin_yaw * world_y;
    body_y = -sin_yaw * world_x + cos_yaw * world_y;
  }

  bool shouldPublishStructuredDebug(const rclcpp::Time & stamp) const
  {
    if (debug_publish_rate_hz_ <= 0.0) {
      return true;
    }
    if (last_debug_publish_time_.nanoseconds() == 0) {
      return true;
    }
    const double elapsed = (stamp - last_debug_publish_time_).seconds();
    return elapsed >= (1.0 / debug_publish_rate_hz_);
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist msg;
    pub_cmd_vel_->publish(msg);
  }

  static bool isSaturated(double raw_value, double clamped_value)
  {
    return std::abs(raw_value - clamped_value) > 1e-9;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  std::string control_frame_id_;
  std::string current_frame_id_;
  std::string pose_topic_;
  std::string cmd_vel_topic_;
  std::string error_topic_;
  std::string debug_topic_;
  std::string path_file_;
  std::string heading_mode_;

  bool path_is_closed_ = true;
  bool publish_debug_topics_ = true;
  bool publish_zero_when_inactive_ = true;
  bool stop_at_end_ = false;
  bool pose_received_ = false;
  bool initialized_ = false;

  int prediction_horizon_ = 12;
  int vx_samples_ = 5;
  int vy_samples_ = 5;
  int w_samples_ = 5;
  int gamma_dot_samples_ = 5;

  double prediction_dt_ = 0.08;
  double fixed_heading_ = 0.0;
  double lookahead_distance_ = 0.10;
  double max_linear_x_ = 0.0;
  double max_linear_y_ = 0.0;
  double max_angular_z_ = 0.0;
  double max_gamma_dot_ = 0.0;
  double min_gamma_dot_ = 0.0;
  double max_accel_x_ = 0.0;
  double max_accel_y_ = 0.0;
  double max_accel_w_ = 0.0;
  double max_gamma_accel_ = 0.0;
  double q_contour_ = 0.0;
  double q_lag_ = 0.0;
  double q_yaw_ = 0.0;
  double q_terminal_contour_ = 0.0;
  double q_terminal_lag_ = 0.0;
  double q_progress_ = 0.0;
  double r_vx_ = 0.0;
  double r_vy_ = 0.0;
  double r_w_ = 0.0;
  double r_gamma_dot_ = 0.0;
  double r_delta_vx_ = 0.0;
  double r_delta_vy_ = 0.0;
  double r_delta_w_ = 0.0;
  double r_delta_gamma_dot_ = 0.0;
  double debug_publish_rate_hz_ = 0.0;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  double gamma_ = 0.0;
  double total_length_ = 0.0;
  double last_target_yaw_ = 0.0;
  double last_ex_world_ = 0.0;
  double last_ey_world_ = 0.0;
  double last_ex_body_ = 0.0;
  double last_ey_body_ = 0.0;
  double last_contour_error_ = 0.0;
  double last_lag_error_ = 0.0;
  double last_yaw_error_ = 0.0;

  Control last_cmd_;
  PathSample last_target_;
  std::vector<PathPoint> points_;
  std::vector<double> cumulative_s_;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_error_;
  rclcpp::Publisher<sdpo_motion_control::msg::PathFollowerDebug>::SharedPtr pub_debug_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
};

}  // namespace sdpo_motion_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_motion_control::NonlinearMpccControllerNode>());
  rclcpp::shutdown();
  return 0;
}
