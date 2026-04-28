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

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode()
  : Node("path_follower")
  {
    control_frame_id_ = declare_parameter<std::string>("control_frame_id", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "pose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    error_topic_ = declare_parameter<std::string>("error_topic", "path_follower_error");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "path_follower_debug");
    controller_mode_ = declare_parameter<std::string>("controller_mode", "moving_reference_p");
    path_file_ = declare_parameter<std::string>("path_file", "");
    path_is_closed_ = declare_parameter<bool>("path_is_closed", true);
    publish_debug_topics_ = declare_parameter<bool>("publish_debug_topics", true);
    publish_zero_when_inactive_ = declare_parameter<bool>("publish_zero_when_inactive", true);
    stop_at_end_ = declare_parameter<bool>("stop_at_end", false);
    startup_mode_ = declare_parameter<std::string>("startup_mode", "nearest_then_follow");
    heading_mode_ = declare_parameter<std::string>("heading_mode", "fixed");

    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 50.0);
    debug_publish_rate_hz_ = declare_parameter<double>("debug_publish_rate_hz", 20.0);
    entry_tolerance_xy_ = declare_parameter<double>("entry_tolerance_xy", 0.03);
    desired_speed_ = declare_parameter<double>("desired_speed", 0.08);
    k_gamma_ = declare_parameter<double>("k_gamma", 0.5);
    min_gamma_dot_ = declare_parameter<double>("min_gamma_dot", 0.0);
    max_gamma_dot_ = declare_parameter<double>("max_gamma_dot", 0.12);
    linear_k_along_ = declare_parameter<double>("linear_k_along", 0.0);
    linear_k_lateral_ = declare_parameter<double>("linear_k_lateral", 0.8);
    linear_projection_lookahead_ = declare_parameter<double>("linear_projection_lookahead", 0.0);
    linear_projection_search_behind_ = declare_parameter<double>("linear_projection_search_behind", 0.10);
    linear_projection_search_ahead_ = declare_parameter<double>("linear_projection_search_ahead", 0.60);
    fixed_heading_ = declare_parameter<double>("fixed_heading", 0.0);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.10);
    beacon_x_ = declare_parameter<double>("beacon_x", 0.0);
    beacon_y_ = declare_parameter<double>("beacon_y", 0.0);

    kp_x_ = declare_parameter<double>("kp_x", 0.8);
    ki_x_ = declare_parameter<double>("ki_x", 0.0);
    kd_x_ = declare_parameter<double>("kd_x", 0.0);
    kp_y_ = declare_parameter<double>("kp_y", 0.9);
    ki_y_ = declare_parameter<double>("ki_y", 0.0);
    kd_y_ = declare_parameter<double>("kd_y", 0.0);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 0.5);
    ki_yaw_ = declare_parameter<double>("ki_yaw", 0.0);
    kd_yaw_ = declare_parameter<double>("kd_yaw", 0.0);

    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.12);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.12);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.35);
    max_integral_x_ = declare_parameter<double>("max_integral_x", 0.15);
    max_integral_y_ = declare_parameter<double>("max_integral_y", 0.15);
    max_integral_yaw_ = declare_parameter<double>("max_integral_yaw", 0.2);

    loadPath(path_file_);

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    if (publish_debug_topics_) {
      pub_error_ = create_publisher<geometry_msgs::msg::Twist>(error_topic_, 10);
      pub_debug_ = create_publisher<sdpo_motion_control::msg::PathFollowerDebug>(debug_topic_, 10);
    }

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      std::bind(&PathFollowerNode::onPose, this, std::placeholders::_1));

    if (startup_mode_ != "nearest_then_follow" && startup_mode_ != "follow_immediately") {
      throw std::runtime_error("startup_mode must be 'nearest_then_follow' or 'follow_immediately'");
    }

    if (controller_mode_ != "moving_reference_p" && controller_mode_ != "linear_segment") {
      throw std::runtime_error("controller_mode must be 'moving_reference_p' or 'linear_segment'");
    }

    if (
      heading_mode_ != "fixed" && heading_mode_ != "tangent" && heading_mode_ != "path" &&
      heading_mode_ != "beacon")
    {
      throw std::runtime_error("heading_mode must be 'fixed', 'tangent', 'path', or 'beacon'");
    }

    state_ = startup_mode_ == "follow_immediately" ? State::FOLLOW_PATH : State::WAITING_FOR_POSE;
    last_update_time_ = now();
    last_debug_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0))),
      std::bind(&PathFollowerNode::onTimer, this));
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
  };

  struct SegmentProjection
  {
    double gamma = 0.0;
    double distance_sq = std::numeric_limits<double>::infinity();
  };

  struct ControlCommand
  {
    PathSample target;
    double target_yaw = 0.0;
    double gamma_dot = 0.0;
    double feedforward_body_x = 0.0;
    double feedforward_body_y = 0.0;
    double position_feedback_body_x = 0.0;
    double position_feedback_body_y = 0.0;
    std::string state_name;
    std::string stop_reason;
  };

  enum class State
  {
    WAITING_FOR_POSE,
    APPROACH_PATH,
    FOLLOW_PATH,
    FINISHED
  };

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_frame_id_ = msg->header.frame_id;
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_yaw_ = tf2::getYaw(msg->pose.orientation);
    pose_received_ = true;
  }

  void onTimer()
  {
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    if (dt <= 0.0) {
      dt = 1e-3;
    }

    if (!pose_received_) {
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(current_time, "waiting_for_pose", "waiting_for_pose", 0.0, 0.0, 0.0);
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
        "Stopping path follower: current pose frame '%s' does not match control frame '%s'.",
        current_frame_id_.c_str(), control_frame_id_.c_str());
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(current_time, stateName(), stop_reason, 0.0, 0.0, 0.0);
      return;
    }

    if (state_ == State::WAITING_FOR_POSE) {
      gamma_ = nearestGamma(current_x_, current_y_);
      target_gamma_ = gamma_;
      resetControllerState();
      state_ = startup_mode_ == "nearest_then_follow" ? State::APPROACH_PATH : State::FOLLOW_PATH;
      RCLCPP_INFO(
        get_logger(), "Path follower initialized at gamma %.3f m using nearest path point.",
        gamma_);
    }

    if (state_ == State::FINISHED) {
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(current_time, stateName(), "", 0.0, 0.0, 0.0);
      return;
    }

    if (state_ == State::APPROACH_PATH) {
      runApproach(current_time, dt);
      return;
    }

    runFollow(current_time, dt);
  }

  void runApproach(const rclcpp::Time & stamp, double dt)
  {
    const PathSample target = sampleAt(target_gamma_);
    const double target_yaw = desiredYaw(target_gamma_, target);
    const double dx_world = target.x - current_x_;
    const double dy_world = target.y - current_y_;
    const double distance = std::hypot(dx_world, dy_world);

    if (distance <= entry_tolerance_xy_) {
      gamma_ = target_gamma_;
      resetControllerState();
      state_ = State::FOLLOW_PATH;
      RCLCPP_INFO(get_logger(), "Path entry reached. Starting gamma tracking.");
      runFollow(stamp, dt);
      return;
    }

    ControlCommand command;
    command.target = target;
    command.target_yaw = target_yaw;
    command.state_name = "approach_path";
    computePositionFeedback(
      target, dt, command.position_feedback_body_x, command.position_feedback_body_y);
    publishCommand(stamp, dt, command);
  }

  void runFollow(const rclcpp::Time & stamp, double dt)
  {
    ControlCommand command = controller_mode_ == "linear_segment" ?
      computeLinearSegmentCommand() :
      computeMovingReferenceCommand(dt);
    publishCommand(stamp, dt, command);
  }

  ControlCommand computeMovingReferenceCommand(double dt)
  {
    ControlCommand command;
    command.target = sampleAt(gamma_);
    command.target_yaw = desiredYaw(gamma_, command.target);
    command.state_name = "follow_path";

    const double error_along =
      (current_x_ - command.target.x) * command.target.tangent_x +
      (current_y_ - command.target.y) * command.target.tangent_y;
    command.gamma_dot = clamp(
      desired_speed_ + k_gamma_ * error_along, min_gamma_dot_, max_gamma_dot_);

    const double ff_world_x = command.target.tangent_x * command.gamma_dot;
    const double ff_world_y = command.target.tangent_y * command.gamma_dot;
    worldToBody(ff_world_x, ff_world_y, command.feedforward_body_x, command.feedforward_body_y);

    computePositionFeedback(
      command.target, dt, command.position_feedback_body_x, command.position_feedback_body_y);

    gamma_ += command.gamma_dot * dt;
    if (path_is_closed_ && total_length_ > 0.0) {
      while (gamma_ >= total_length_) {
        gamma_ -= total_length_;
      }
    } else if (gamma_ >= total_length_) {
      gamma_ = total_length_;
      if (stop_at_end_) {
        state_ = State::FINISHED;
        resetControllerState();
      }
    }

    return command;
  }

  ControlCommand computeLinearSegmentCommand()
  {
    ControlCommand command;
    const SegmentProjection projection = projectOnPathNearGamma(
      current_x_, current_y_, gamma_, linear_projection_search_behind_,
      linear_projection_search_ahead_, linear_projection_lookahead_);
    gamma_ = projection.gamma;

    command.target = sampleAt(gamma_);
    command.target_yaw = desiredYaw(gamma_, command.target);
    command.gamma_dot = desired_speed_;
    command.state_name = "linear_segment";

    const double dx_world = command.target.x - current_x_;
    const double dy_world = command.target.y - current_y_;
    const double normal_x = -command.target.tangent_y;
    const double normal_y = command.target.tangent_x;
    const double error_along =
      dx_world * command.target.tangent_x + dy_world * command.target.tangent_y;
    const double error_lateral = dx_world * normal_x + dy_world * normal_y;

    const double cmd_world_x =
      desired_speed_ * command.target.tangent_x +
      linear_k_along_ * error_along * command.target.tangent_x +
      linear_k_lateral_ * error_lateral * normal_x;
    const double cmd_world_y =
      desired_speed_ * command.target.tangent_y +
      linear_k_along_ * error_along * command.target.tangent_y +
      linear_k_lateral_ * error_lateral * normal_y;
    worldToBody(cmd_world_x, cmd_world_y, command.feedforward_body_x, command.feedforward_body_y);

    if (!path_is_closed_ && projection.gamma >= total_length_ && stop_at_end_) {
      state_ = State::FINISHED;
      resetControllerState();
    }

    return command;
  }

  void publishCommand(const rclcpp::Time & stamp, double dt, const ControlCommand & command)
  {
    const double dx_world = command.target.x - current_x_;
    const double dy_world = command.target.y - current_y_;
    const double yaw_error = normalizeAngle(command.target_yaw - current_yaw_);

    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    const double ex_body = cos_yaw * dx_world + sin_yaw * dy_world;
    const double ey_body = -sin_yaw * dx_world + cos_yaw * dy_world;

    integral_yaw_ = updateIntegral(integral_yaw_, yaw_error, dt, ki_yaw_, max_integral_yaw_);

    derivative_yaw_ = normalizeAngle(yaw_error - prev_error_yaw_) / dt;

    prev_error_yaw_ = yaw_error;

    const double cmd_vx_raw =
      command.feedforward_body_x + command.position_feedback_body_x;
    const double cmd_vy_raw =
      command.feedforward_body_y + command.position_feedback_body_y;
    const double cmd_w_raw =
      kp_yaw_ * yaw_error + ki_yaw_ * integral_yaw_ + kd_yaw_ * derivative_yaw_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(cmd_vx_raw, -max_linear_x_, max_linear_x_);
    cmd.linear.y = clamp(cmd_vy_raw, -max_linear_y_, max_linear_y_);
    cmd.angular.z = clamp(cmd_w_raw, -max_angular_z_, max_angular_z_);
    pub_cmd_vel_->publish(cmd);

    last_target_ = command.target;
    last_target_yaw_ = command.target_yaw;
    last_gamma_dot_ = command.gamma_dot;
    last_error_along_ =
      (current_x_ - command.target.x) * command.target.tangent_x +
      (current_y_ - command.target.y) * command.target.tangent_y;
    last_ff_body_x_ = command.feedforward_body_x;
    last_ff_body_y_ = command.feedforward_body_y;
    last_cmd_vx_raw_ = cmd_vx_raw;
    last_cmd_vy_raw_ = cmd_vy_raw;
    last_cmd_w_raw_ = cmd_w_raw;
    last_cmd_vx_ = cmd.linear.x;
    last_cmd_vy_ = cmd.linear.y;
    last_cmd_w_ = cmd.angular.z;
    last_ex_world_ = dx_world;
    last_ey_world_ = dy_world;
    last_ex_body_ = ex_body;
    last_ey_body_ = ey_body;
    last_yaw_error_ = yaw_error;

    publishDebug(
      stamp, command.state_name, command.stop_reason, cmd.linear.x, cmd.linear.y, cmd.angular.z);
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

      if (!numeric_row) {
        continue;
      }
      if (values.size() < 2) {
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

    if (path_is_closed_) {
      while (gamma < 0.0) {
        gamma += total_length_;
      }
      while (gamma >= total_length_) {
        gamma -= total_length_;
      }
    } else {
      gamma = clamp(gamma, 0.0, total_length_);
    }

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
    sample.has_yaw = a.has_yaw && b.has_yaw;
    if (sample.has_yaw) {
      sample.yaw = normalizeAngle(a.yaw + ratio * normalizeAngle(b.yaw - a.yaw));
    } else {
      sample.yaw = std::atan2(sample.tangent_y, sample.tangent_x);
    }
    return sample;
  }

  double nearestGamma(double x, double y) const
  {
    return projectOnPath(x, y, 0.0).gamma;
  }

  SegmentProjection projectOnPath(double x, double y, double lookahead) const
  {
    SegmentProjection best;
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
    best.gamma = normalizePathGamma(best.gamma + lookahead);
    return best;
  }

  SegmentProjection projectOnPathNearGamma(
    double x, double y, double center_gamma, double search_behind, double search_ahead,
    double lookahead) const
  {
    if (total_length_ <= 0.0) {
      return SegmentProjection();
    }

    search_behind = std::max(0.0, search_behind);
    search_ahead = std::max(0.0, search_ahead);

    const double window_start = center_gamma - search_behind;
    const double window_end = center_gamma + search_ahead;
    SegmentProjection best;

    const int min_wrap = path_is_closed_ ? -1 : 0;
    const int max_wrap = path_is_closed_ ? 1 : 0;
    for (int wrap = min_wrap; wrap <= max_wrap; ++wrap) {
      const double offset = static_cast<double>(wrap) * total_length_;
      for (size_t i = 0; i + 1 < points_.size(); ++i) {
        const double segment_start = cumulative_s_[i] + offset;
        const double segment_end = cumulative_s_[i + 1] + offset;
        if (segment_end < window_start || segment_start > window_end) {
          continue;
        }

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
        double u = clamp((ap_x * ab_x + ap_y * ab_y) / ab_len_sq, 0.0, 1.0);
        const double segment_length = std::sqrt(ab_len_sq);
        const double gamma_candidate = segment_start + u * segment_length;
        const double clamped_gamma_candidate =
          clamp(gamma_candidate, window_start, window_end);
        u = (clamped_gamma_candidate - segment_start) / segment_length;

        const double proj_x = a.x + u * ab_x;
        const double proj_y = a.y + u * ab_y;
        const double dx = x - proj_x;
        const double dy = y - proj_y;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best.distance_sq) {
          best.distance_sq = distance_sq;
          best.gamma = clamped_gamma_candidate;
        }
      }
    }

    if (!std::isfinite(best.distance_sq)) {
      best.gamma = center_gamma;
    }

    best.gamma = normalizePathGamma(best.gamma + lookahead);
    return best;
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

  double desiredYaw(double gamma, const PathSample & sample) const
  {
    if (heading_mode_ == "fixed") {
      return fixed_heading_;
    }
    if (heading_mode_ == "beacon") {
      return std::atan2(beacon_y_ - current_y_, beacon_x_ - current_x_);
    }
    if (heading_mode_ == "path" && sample.has_yaw) {
      return sample.yaw;
    }

    const PathSample ahead = sampleAt(gamma + std::max(lookahead_distance_, 1e-6));
    return std::atan2(ahead.y - sample.y, ahead.x - sample.x);
  }

  void worldToBody(double world_x, double world_y, double & body_x, double & body_y) const
  {
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    body_x = cos_yaw * world_x + sin_yaw * world_y;
    body_y = -sin_yaw * world_x + cos_yaw * world_y;
  }

  void computePositionFeedback(
    const PathSample & target, double dt, double & feedback_x, double & feedback_y)
  {
    const double dx_world = target.x - current_x_;
    const double dy_world = target.y - current_y_;
    double ex_body = 0.0;
    double ey_body = 0.0;
    worldToBody(dx_world, dy_world, ex_body, ey_body);

    integral_x_ = updateIntegral(integral_x_, ex_body, dt, ki_x_, max_integral_x_);
    integral_y_ = updateIntegral(integral_y_, ey_body, dt, ki_y_, max_integral_y_);
    derivative_x_ = (ex_body - prev_error_x_) / dt;
    derivative_y_ = (ey_body - prev_error_y_) / dt;
    prev_error_x_ = ex_body;
    prev_error_y_ = ey_body;

    feedback_x = kp_x_ * ex_body + ki_x_ * integral_x_ + kd_x_ * derivative_x_;
    feedback_y = kp_y_ * ey_body + ki_y_ * integral_y_ + kd_y_ * derivative_y_;
  }

  void publishDebug(
    const rclcpp::Time & stamp, const std::string & state_name, const std::string & stop_reason,
    double cmd_vx, double cmd_vy, double cmd_w)
  {
    if (!publish_debug_topics_) {
      return;
    }

    geometry_msgs::msg::Twist error_msg;
    error_msg.linear.x = last_ex_body_;
    error_msg.linear.y = last_ey_body_;
    error_msg.angular.z = last_yaw_error_;
    pub_error_->publish(error_msg);

    if (!shouldPublishStructuredDebug(stamp)) {
      return;
    }

    sdpo_motion_control::msg::PathFollowerDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = control_frame_id_;
    msg.active = state_ == State::APPROACH_PATH || state_ == State::FOLLOW_PATH;
    msg.state = state_name;
    msg.control_frame_id = control_frame_id_;
    msg.current_frame_id = current_frame_id_;
    msg.stop_reason = stop_reason;
    msg.current_x = current_x_;
    msg.current_y = current_y_;
    msg.current_yaw = current_yaw_;
    msg.target_x = last_target_.x;
    msg.target_y = last_target_.y;
    msg.target_yaw = last_target_yaw_;
    msg.tangent_x = last_target_.tangent_x;
    msg.tangent_y = last_target_.tangent_y;
    msg.gamma = gamma_;
    msg.gamma_dot = last_gamma_dot_;
    msg.error_x_world = last_ex_world_;
    msg.error_y_world = last_ey_world_;
    msg.error_x_body = last_ex_body_;
    msg.error_y_body = last_ey_body_;
    msg.error_along = last_error_along_;
    msg.error_yaw = last_yaw_error_;
    msg.integral_x = integral_x_;
    msg.integral_y = integral_y_;
    msg.integral_yaw = integral_yaw_;
    msg.derivative_x = derivative_x_;
    msg.derivative_y = derivative_y_;
    msg.derivative_yaw = derivative_yaw_;
    msg.feedforward_vx = last_ff_body_x_;
    msg.feedforward_vy = last_ff_body_y_;
    msg.cmd_vx_raw = last_cmd_vx_raw_;
    msg.cmd_vy_raw = last_cmd_vy_raw_;
    msg.cmd_w_raw = last_cmd_w_raw_;
    msg.cmd_vx = cmd_vx;
    msg.cmd_vy = cmd_vy;
    msg.cmd_w = cmd_w;
    msg.saturated_vx = isSaturated(last_cmd_vx_raw_, last_cmd_vx_);
    msg.saturated_vy = isSaturated(last_cmd_vy_raw_, last_cmd_vy_);
    msg.saturated_w = isSaturated(last_cmd_w_raw_, last_cmd_w_);
    pub_debug_->publish(msg);
    last_debug_publish_time_ = stamp;
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

  std::string stateName() const
  {
    switch (state_) {
      case State::WAITING_FOR_POSE:
        return "waiting_for_pose";
      case State::APPROACH_PATH:
        return "approach_path";
      case State::FOLLOW_PATH:
        return "follow_path";
      case State::FINISHED:
        return "finished";
    }
    return "unknown";
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist msg;
    pub_cmd_vel_->publish(msg);
  }

  void resetControllerState()
  {
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    integral_yaw_ = 0.0;
    prev_error_x_ = 0.0;
    prev_error_y_ = 0.0;
    prev_error_yaw_ = 0.0;
    derivative_x_ = 0.0;
    derivative_y_ = 0.0;
    derivative_yaw_ = 0.0;
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

  static double updateIntegral(
    double integral, double error, double dt, double ki, double max_integral)
  {
    if (ki == 0.0) {
      return 0.0;
    }
    return clamp(integral + error * dt, -max_integral, max_integral);
  }

  std::string control_frame_id_;
  std::string current_frame_id_;
  std::string pose_topic_;
  std::string cmd_vel_topic_;
  std::string error_topic_;
  std::string debug_topic_;
  std::string controller_mode_;
  std::string path_file_;
  std::string startup_mode_;
  std::string heading_mode_;

  bool path_is_closed_ = true;
  bool publish_debug_topics_ = true;
  bool publish_zero_when_inactive_ = true;
  bool stop_at_end_ = false;
  bool pose_received_ = false;

  double entry_tolerance_xy_ = 0.0;
  double desired_speed_ = 0.0;
  double k_gamma_ = 0.0;
  double min_gamma_dot_ = 0.0;
  double max_gamma_dot_ = 0.0;
  double linear_k_along_ = 0.0;
  double linear_k_lateral_ = 0.0;
  double linear_projection_lookahead_ = 0.0;
  double linear_projection_search_behind_ = 0.0;
  double linear_projection_search_ahead_ = 0.0;
  double fixed_heading_ = 0.0;
  double lookahead_distance_ = 0.0;
  double beacon_x_ = 0.0;
  double beacon_y_ = 0.0;

  double kp_x_ = 0.0;
  double ki_x_ = 0.0;
  double kd_x_ = 0.0;
  double kp_y_ = 0.0;
  double ki_y_ = 0.0;
  double kd_y_ = 0.0;
  double kp_yaw_ = 0.0;
  double ki_yaw_ = 0.0;
  double kd_yaw_ = 0.0;

  double max_linear_x_ = 0.0;
  double max_linear_y_ = 0.0;
  double max_angular_z_ = 0.0;
  double max_integral_x_ = 0.0;
  double max_integral_y_ = 0.0;
  double max_integral_yaw_ = 0.0;
  double debug_publish_rate_hz_ = 0.0;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  double gamma_ = 0.0;
  double target_gamma_ = 0.0;
  double total_length_ = 0.0;

  double integral_x_ = 0.0;
  double integral_y_ = 0.0;
  double integral_yaw_ = 0.0;
  double prev_error_x_ = 0.0;
  double prev_error_y_ = 0.0;
  double prev_error_yaw_ = 0.0;
  double derivative_x_ = 0.0;
  double derivative_y_ = 0.0;
  double derivative_yaw_ = 0.0;

  PathSample last_target_;
  double last_target_yaw_ = 0.0;
  double last_gamma_dot_ = 0.0;
  double last_error_along_ = 0.0;
  double last_ff_body_x_ = 0.0;
  double last_ff_body_y_ = 0.0;
  double last_cmd_vx_raw_ = 0.0;
  double last_cmd_vy_raw_ = 0.0;
  double last_cmd_w_raw_ = 0.0;
  double last_cmd_vx_ = 0.0;
  double last_cmd_vy_ = 0.0;
  double last_cmd_w_ = 0.0;
  double last_ex_world_ = 0.0;
  double last_ey_world_ = 0.0;
  double last_ex_body_ = 0.0;
  double last_ey_body_ = 0.0;
  double last_yaw_error_ = 0.0;

  State state_ = State::WAITING_FOR_POSE;
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
  rclcpp::spin(std::make_shared<sdpo_motion_control::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
