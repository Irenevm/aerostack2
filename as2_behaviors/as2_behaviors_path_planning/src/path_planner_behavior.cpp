// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!******************************************************************************
 *  \file       path_planner_behavior.cpp
 *  \brief      path_planner_behavior implementation file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#include <algorithm>
#include <cmath>
#include <limits>

#include "as2_behaviors_path_planning/path_planner_behavior.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

PathPlannerBehavior::PathPlannerBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::NavigateToPoint>("path_planner", options)
{
  try {
    this->declare_parameter("plugin_name", "a_star");
    this->get_parameter("plugin_name", plugin_name_);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Launch argument <plugin_name> not defined or malformed: %s",
      e.what());
    this->~PathPlannerBehavior();
  }

  this->declare_parameter("enable_visualization", false);
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();

  this->declare_parameter("enable_path_optimizer", false);
  enable_path_optimizer_ = this->get_parameter("enable_path_optimizer").as_bool();

  // TODO(pariaspe): move to action_goal
  this->declare_parameter("safety_distance", 1.0);  // aprox drone size [m]
  safety_distance_ = this->get_parameter("safety_distance").as_double();

  this->declare_parameter("drone_mask_factor", 1);
  drone_mask_factor_ = this->get_parameter("drone_mask_factor").as_int();

  this->declare_parameter("simplify_path", false);
  simplify_path_ = this->get_parameter("simplify_path").as_bool();

  this->declare_parameter("dist_to_line_threshold", 1.0);
  dist_to_line_threshold_ = this->get_parameter("dist_to_line_threshold").as_double();

  this->declare_parameter("max_replans", 15);
  max_replans_ = this->get_parameter("max_replans").as_int();

  this->declare_parameter("map_check_period", 2.0);
  double map_check_period = this->get_parameter("map_check_period").as_double();
  map_check_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(map_check_period),
    [this]() { check_map_ = true; });
  map_check_timer_->cancel();

  // ── LiDAR Reactive Safety Layer parameters ─────────────────────────────
  this->declare_parameter("enable_lidar_safety", true);
  enable_lidar_safety_ = this->get_parameter("enable_lidar_safety").as_bool();

  this->declare_parameter("lidar_check_period", 0.05);
  double lidar_check_period = this->get_parameter("lidar_check_period").as_double();

  this->declare_parameter("lidar_danger_distance", 1.5);
  lidar_danger_distance_ = this->get_parameter("lidar_danger_distance").as_double();

  this->declare_parameter("lidar_stop_distance", 0.7);
  lidar_stop_distance_ = this->get_parameter("lidar_stop_distance").as_double();

  this->declare_parameter("lidar_corridor_half_width", 0.5);
  lidar_corridor_half_width_ = this->get_parameter("lidar_corridor_half_width").as_double();

  this->declare_parameter("lidar_min_cluster_size", 3);
  lidar_min_cluster_size_ = this->get_parameter("lidar_min_cluster_size").as_int();

  this->declare_parameter("lidar_persistence_count", 2);
  lidar_persistence_count_ = this->get_parameter("lidar_persistence_count").as_int();

  this->declare_parameter("enable_lidar_decel", true);
  enable_lidar_decel_ = this->get_parameter("enable_lidar_decel").as_bool();

  this->declare_parameter("lidar_kp", 1.0);
  lidar_kp_ = this->get_parameter("lidar_kp").as_double();

  this->declare_parameter("decel_speed_epsilon", 0.1);
  decel_speed_epsilon_ = this->get_parameter("decel_speed_epsilon").as_double();

  lidar_check_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(lidar_check_period),
    [this]() { check_lidar_ = true; });
  lidar_check_timer_->cancel();

  lidar_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "sensor_measurements/lidar/scan",
    rclcpp::SensorDataQoS(),
    std::bind(&PathPlannerBehavior::lidar_scan_cbk, this, std::placeholders::_1));
  // ── end LiDAR params ───────────────────────────────────────────────────

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Loading plugin
  plugin_name_ += "::Plugin";
  RCLCPP_INFO(this->get_logger(), "Loading plugin: %s", plugin_name_.c_str());
  loader_ =
    std::make_shared<pluginlib::ClassLoader<as2_behaviors_path_planning::PluginBase>>(
    "as2_behaviors_path_planning", "as2_behaviors_path_planning::PluginBase");
  try {
    path_planner_plugin_ = loader_->createSharedInstance(plugin_name_);
    path_planner_plugin_->initialize(this, tf_buffer_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The plugin failed to load. Error: %s", ex.what());
    this->~PathPlannerBehavior();
  }

  // TODO(pariaspe): use as2_names
  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", as2_names::topics::self_localization::qos,
    std::bind(&PathPlannerBehavior::drone_pose_cbk, this, std::placeholders::_1));

  if (enable_visualization_) {
    viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
  }

  follow_path_client_ = rclcpp_action::create_client<as2_msgs::action::FollowPath>(
    this, as2_names::actions::behaviors::followpath);

  // TODO(pariaspe): modify follow_path interfaces
  follow_path_pause_client_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    std::string(as2_names::actions::behaviors::followpath) + "/_behavior/pause", this);

  follow_path_resume_client_ =
    std::make_shared<as2::SynchronousServiceClient<std_srvs::srv::Trigger>>(
    std::string(as2_names::actions::behaviors::followpath) + "/_behavior/resume", this);
}

bool PathPlannerBehavior::on_activate(
  std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(), "Activating Path Planner Behavior to point [%.2f, %.2f, %.2f]",
    goal->point.point.x, goal->point.point.y, goal->point.point.z);

  original_goal_ = *goal;
  is_intermediate_goal_ = false;
  need_replan_ = false;
  replan_count_ = 0;
  check_map_ = false;
  waiting_for_map_check_replan_ = false;
  pose_at_plan_start_ = drone_pose_;
  map_check_timer_->reset();

  // Reset LiDAR safety state
  check_lidar_ = false;
  lidar_braking_ = false;
  consecutive_detections_ = 0;
  last_sent_speed_ = 0.0;
  if (enable_lidar_safety_) {
    lidar_check_timer_->reset();
  }

  bool ret = path_planner_plugin_->on_activate(drone_pose_, *goal);
  if (!ret) {
    bool occupied = path_planner_plugin_->is_occupied(goal->point);
    if (occupied) {
      RCLCPP_WARN(
        this->get_logger(),
        "Goal point is inside an obstacle. Looking for alternative goal point.");
      geometry_msgs::msg::PointStamped drone_point = geometry_msgs::msg::PointStamped();
      drone_point.header.frame_id = "earth";
      drone_point.header.stamp = this->get_clock()->now();
      drone_point.point = drone_pose_.pose.position;
      geometry_msgs::msg::PointStamped new_goal =
        path_planner_plugin_->closest_free_point(drone_point, goal->point);
      if (
        new_goal.point.x == drone_point.point.x &&
        new_goal.point.y == drone_point.point.y &&
        new_goal.point.z == drone_point.point.z)
      {
        RCLCPP_ERROR(this->get_logger(), "No alternative goal point found. Aborting navigation.");
        return false;
      }
      RCLCPP_INFO(
        this->get_logger(), "New goal point: [%.2f, %.2f, %.2f]",
        new_goal.point.x, new_goal.point.y, new_goal.point.z);

      auto new_goal_ptr = std::make_shared<as2_msgs::action::NavigateToPoint::Goal>();
      new_goal_ptr->point = new_goal;
      new_goal_ptr->yaw = goal->yaw;
      new_goal_ptr->navigation_speed = goal->navigation_speed;
      ret = path_planner_plugin_->on_activate(drone_pose_, *new_goal_ptr);
      if (!ret) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Path planner plugin failed to activate even with alternative goal point. Aborting.");
        return false;
      }
      is_intermediate_goal_ = true;
    } else {
      return false;
    }
  }

  // Call FollowPath behavior
  follow_path_succeeded_ = false;
  if (!this->follow_path_client_->wait_for_action_server(
      std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Follow Path Action server not available after waiting. Aborting navigation.");
    return false;
  }

  path_ = path_planner_plugin_->path_;

  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath behavior");
  send_follow_path_goal(goal->navigation_speed);
  last_sent_speed_ = goal->navigation_speed;

  return true;
}

bool PathPlannerBehavior::on_modify(
  std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal)
{
  RCLCPP_WARN(this->get_logger(), "Modify not implemented");
  return false;
}

bool PathPlannerBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  map_check_timer_->cancel();
  lidar_check_timer_->cancel();
  check_lidar_ = false;
  lidar_braking_ = false;
  consecutive_detections_ = 0;
  waiting_for_map_check_replan_ = false;
  // Cancel only the goal started from navigation. Behaviors only accepts
  // one goal simultaneously, don't have to worry about
  follow_path_client_->async_cancel_all_goals();
  navigation_aborted_ = true;
  return true;
}

bool PathPlannerBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool paused = follow_path_pause_client_->sendRequest(req, res, 3);
  return paused && res.success;
}

bool PathPlannerBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool resumed = follow_path_resume_client_->sendRequest(req, res, 3);
  return resumed && res.success;
}

void PathPlannerBehavior::on_execution_end(const as2_behavior::ExecutionStatus & state)
{
  std::string state_str;
  switch (state) {
    case as2_behavior::ExecutionStatus::SUCCESS:
      state_str = "SUCCEEDED";
      break;
    case as2_behavior::ExecutionStatus::ABORTED:
      state_str = "ABORTED";
      break;
    case as2_behavior::ExecutionStatus::RUNNING:
      state_str = "RUNNING";
      break;
    case as2_behavior::ExecutionStatus::FAILURE:
      state_str = "FAILED";
      break;
    default:
      state_str = "UNKNOWN";
      break;
  }
  RCLCPP_INFO(this->get_logger(), "Execution ended with state: %s", state_str.c_str());
}

as2_behavior::ExecutionStatus PathPlannerBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> & goal,
  std::shared_ptr<as2_msgs::action::NavigateToPoint::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::NavigateToPoint::Result> & result_msg)
{
  if (navigation_aborted_) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // FollowPath server rejected the goal (busy after a cancel). Retry by
  // replanning: the server will be free by the next on_run cycle.
  if (follow_path_rejected_) {
    follow_path_rejected_ = false;
    if (replan_count_ <= max_replans_) {
      RCLCPP_WARN(
        this->get_logger(),
        "FollowPath rejected by server — retrying replan (%d/%d).", replan_count_, max_replans_);
      need_replan_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "FollowPath rejected after max replans. Aborting.");
      result_msg->success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }
  }

  if (need_replan_) {
    need_replan_ = false;
    trigger_replan();
    return as2_behavior::ExecutionStatus::RUNNING;
  }

  // ── LiDAR REACTIVE SAFETY ────────────────────────────────────────────────
  if (check_lidar_) {
    check_lidar_ = false;
    if (enable_lidar_safety_ && !navigation_aborted_ &&
        !need_replan_ && !waiting_for_map_check_replan_)
    {
      double nearest = std::numeric_limits<double>::infinity();
      bool obstacle = evaluate_lidar_corridor(nearest);

      if (obstacle) {
        if (nearest <= lidar_stop_distance_) {
          // Full stop via pause/hover
          if (!lidar_braking_) {
            RCLCPP_WARN(
              get_logger(),
              "[LIDAR_SAFETY] DETECTED — d=%.2fm ≤ stop=%.2fm → engaging brake",
              nearest, lidar_stop_distance_);
            engage_lidar_brake();
          }
        } else if (enable_lidar_decel_) {
          // Progressive braking zone: stop_distance < nearest <= danger_distance
          if (lidar_braking_) {
            // Obstacle backed off from stop zone — resume before decelerating
            std_srvs::srv::Trigger::Request req;
            std_srvs::srv::Trigger::Response res;
            follow_path_resume_client_->sendRequest(req, res, 1);
            lidar_braking_ = false;
          }
          double v_safe = compute_safe_speed(nearest);
          if (std::abs(v_safe - last_sent_speed_) > decel_speed_epsilon_) {
            RCLCPP_INFO(
              get_logger(),
              "[LIDAR_SAFETY] DETECTED — d=%.2fm → speed %.2f→%.2f m/s",
              nearest, last_sent_speed_, v_safe);
            send_follow_path_goal(v_safe);
            last_sent_speed_ = v_safe;
          }
        }
      } else {
        // Corridor clear — restore normal navigation if we were braking/decelerating
        bool was_braking = lidar_braking_;
        bool was_decelerating =
          (last_sent_speed_ > 0.0 &&
           last_sent_speed_ < original_goal_.navigation_speed - decel_speed_epsilon_);

        if (was_braking || was_decelerating) {
          if (was_braking) {
            std_srvs::srv::Trigger::Request req;
            std_srvs::srv::Trigger::Response res;
            follow_path_resume_client_->sendRequest(req, res, 1);
          }
          lidar_braking_ = false;
          send_follow_path_goal(original_goal_.navigation_speed);
          last_sent_speed_ = original_goal_.navigation_speed;
          RCLCPP_INFO(
            get_logger(), "[LIDAR_SAFETY] Corridor clear — full speed %.2f m/s restored",
            original_goal_.navigation_speed);
        }
        lidar_braking_ = false;
      }

      // Warn if braking for too long without a map_check replan
      if (lidar_braking_) {
        double brake_elapsed = (this->get_clock()->now() - lidar_brake_time_).seconds();
        if (brake_elapsed > 4.0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[LIDAR_SAFETY] Braking for %.1fs — waiting for map_check replan.", brake_elapsed);
        }
      }
    }
  }
  // ── END LiDAR REACTIVE SAFETY ────────────────────────────────────────────

  if (check_map_) {
    check_map_ = false;
    // Run A* to update the internal graph with the current map, and check
    // whether the original goal is directly reachable from here.
    RCLCPP_INFO(
      this->get_logger(),
      "[MAP_CHECK] tick — drone=[%.2f, %.2f] intermediate=%s",
      drone_pose_.pose.position.x, drone_pose_.pose.position.y,
      is_intermediate_goal_ ? "true" : "false");
    bool can_reach_goal = path_planner_plugin_->on_activate(drone_pose_, original_goal_);

    // Hito A: direct path to original goal became available while flying to frontier.
    // Only fire when drone is sufficiently close to original goal — A* spuriously finds
    // "reachable" paths from far away near map boundaries; firing too early causes
    // trigger_replan() to fail from a position inside an inflated-obstacle cell.
    double dist_to_original_goal = std::hypot(
      drone_pose_.pose.position.x - original_goal_.point.point.x,
      drone_pose_.pose.position.y - original_goal_.point.point.y);
    if (can_reach_goal && is_intermediate_goal_ && dist_to_original_goal < 8.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "[MAP_CHECK] Direct path to goal [%.2f, %.2f] now available (dist=%.1fm). Replanning.",
        original_goal_.point.point.x, original_goal_.point.point.y, dist_to_original_goal);
      is_intermediate_goal_ = false;
      follow_path_client_->async_cancel_all_goals();
      waiting_for_map_check_replan_ = true;
    } else if (!can_reach_goal && !is_intermediate_goal_) {
      // Hito B (complete blockage): A* can no longer reach the goal at all.
      RCLCPP_WARN(
        this->get_logger(),
        "[MAP_CHECK] Path to goal [%.2f, %.2f] is now blocked. Replanning.",
        original_goal_.point.point.x, original_goal_.point.point.y);
      follow_path_client_->async_cancel_all_goals();
      waiting_for_map_check_replan_ = true;
    } else if (can_reach_goal && !is_intermediate_goal_ && !waiting_for_map_check_replan_) {
      // Hito B (path segment check): we're on the direct path — sample every segment
      // between consecutive waypoints to detect obstacles that fall between RDP-reduced
      // waypoints. Sampling at safety_distance_ intervals catches dynamic obstacles that
      // appear between waypoints but would not be caught by checking waypoints alone.
      double dist_from_plan_start = std::hypot(
        drone_pose_.pose.position.x - pose_at_plan_start_.pose.position.x,
        drone_pose_.pose.position.y - pose_at_plan_start_.pose.position.y);
      if (dist_from_plan_start >= safety_distance_ * 3.0 && path_.size() >= 2) {
        geometry_msgs::msg::PointStamped pt;
        pt.header.frame_id = "earth";
        pt.header.stamp = this->get_clock()->now();
        bool occupied_found = false;
        int total_checked = 0;
        for (size_t i = 0; i + 1 < path_.size() && !occupied_found; ++i) {
          const auto & p1 = path_[i];
          const auto & p2 = path_[i + 1];
          // Skip segments entirely behind the drone.
          double p2_dist_from_start = std::hypot(
            p2.x - pose_at_plan_start_.pose.position.x,
            p2.y - pose_at_plan_start_.pose.position.y);
          if (p2_dist_from_start <= dist_from_plan_start) {
            continue;
          }
          double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
          int n_samples = std::max(1, static_cast<int>(std::ceil(seg_len / safety_distance_)));
          for (int s = 1; s <= n_samples && !occupied_found; ++s) {
            double t = static_cast<double>(s) / n_samples;
            double sx = p1.x + t * (p2.x - p1.x);
            double sy = p1.y + t * (p2.y - p1.y);
            double dist_to_drone = std::hypot(
              sx - drone_pose_.pose.position.x,
              sy - drone_pose_.pose.position.y);
            if (dist_to_drone < safety_distance_ * 1.5) {
              continue;
            }
            total_checked++;
            pt.point.x = sx;
            pt.point.y = sy;
            pt.point.z = path_[i].z;
            if (path_planner_plugin_->is_occupied(pt)) {
              RCLCPP_WARN(
                this->get_logger(),
                "[MAP_CHECK] Path segment occupied at [%.2f, %.2f] (segment %zu/%zu). Replanning.",
                sx, sy, i + 1, path_.size() - 1);
              follow_path_client_->async_cancel_all_goals();
              waiting_for_map_check_replan_ = true;
              occupied_found = true;
            }
          }
        }
        if (!occupied_found) {
          RCLCPP_INFO(
            this->get_logger(),
            "[MAP_CHECK] Path segments free (sampled %d pts, waypoints=%zu).",
            total_checked, path_.size());
        }
      }
    }
  }

  // TODO(pariaspe): current feedback is just a template
  if (!follow_path_feedback_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for feedback from FollowPath behavior");
    return as2_behavior::ExecutionStatus::RUNNING;
  }
  feedback_msg->current_pose = drone_pose_;
  feedback_msg->current_speed.twist.linear.x = follow_path_feedback_->actual_speed;
  feedback_msg->distance_remaining = follow_path_feedback_->actual_distance_to_next_waypoint;
  // feedback_msg->estimated_time_remaining = -1;
  // feedback_msg->navigation_time = -1;

  if (follow_path_succeeded_) {
    result_msg->success = true;
    return as2_behavior::ExecutionStatus::SUCCESS;
  }
  return as2_behavior::ExecutionStatus::RUNNING;
}

void PathPlannerBehavior::drone_pose_cbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  drone_pose_ = *(msg);
}

void PathPlannerBehavior::follow_path_response_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(
      this->get_logger(), "FollowPath was rejected by behavior server. Aborting navigation.");
    follow_path_rejected_ = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "FollowPath accepted, flying to point.");
  }
}

void PathPlannerBehavior::follow_path_feedback_cbk(
  rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr goal_handle,
  const std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback)
{
  if (navigation_aborted_) {
    // cancel follow path too
    follow_path_client_->async_cancel_goal(goal_handle);
    return;
  }

  follow_path_feedback_ = feedback;
}

void PathPlannerBehavior::follow_path_result_cbk(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "FollowPath was aborted. Aborting navigation.");
      // navigation_aborted_ = true;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      if (waiting_for_map_check_replan_) {
        waiting_for_map_check_replan_ = false;
        need_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "FollowPath canceled — triggering MAP_CHECK replan.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "FollowPath was canceled. Cancelling navigation");
        // navigation_aborted_ = true;
      }
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code from FollowPath. Aborting navigation.");
      navigation_aborted_ = true;
      return;
  }

  if (is_intermediate_goal_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Frontier waypoint reached. Triggering replan to original goal [%.2f, %.2f, %.2f].",
      original_goal_.point.point.x, original_goal_.point.point.y,
      original_goal_.point.point.z);
    need_replan_ = true;
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Follow Path succeeded. Goal point reached. Navigation succeeded.");
    follow_path_succeeded_ = true;
  }
}

void PathPlannerBehavior::trigger_replan()
{
  follow_path_succeeded_ = false;
  follow_path_rejected_ = false;
  follow_path_feedback_.reset();
  is_intermediate_goal_ = false;
  pose_at_plan_start_ = drone_pose_;

  // Reset LiDAR safety state so the new FollowPath goal starts unbraked
  lidar_braking_ = false;
  consecutive_detections_ = 0;
  last_sent_speed_ = 0.0;

  if (++replan_count_ > max_replans_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Replan: exceeded %d replans without reaching goal. Aborting navigation.", max_replans_);
    navigation_aborted_ = true;
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Replanning to original goal [%.2f, %.2f, %.2f]",
    original_goal_.point.point.x, original_goal_.point.point.y,
    original_goal_.point.point.z);

  bool ret = path_planner_plugin_->on_activate(drone_pose_, original_goal_);
  if (!ret) {
    bool occupied = path_planner_plugin_->is_occupied(original_goal_.point);
    if (occupied) {
      geometry_msgs::msg::PointStamped drone_point;
      drone_point.header.frame_id = "earth";
      drone_point.header.stamp = this->get_clock()->now();
      drone_point.point = drone_pose_.pose.position;

      geometry_msgs::msg::PointStamped new_frontier =
        path_planner_plugin_->closest_free_point(drone_point, original_goal_.point);

      if (
        new_frontier.point.x == drone_point.point.x &&
        new_frontier.point.y == drone_point.point.y &&
        new_frontier.point.z == drone_point.point.z)
      {
        RCLCPP_ERROR(
          this->get_logger(), "Replan: no frontier found. Aborting navigation.");
        navigation_aborted_ = true;
        return;
      }

      RCLCPP_INFO(
        this->get_logger(), "Replan: new frontier at [%.2f, %.2f, %.2f]",
        new_frontier.point.x, new_frontier.point.y, new_frontier.point.z);

      // Abort if the frontier is farther from the goal than the drone itself.
      // This means closest_free_point() found a cell on the wrong side of an
      // obstacle — continuing would drive the drone away from the goal.
      double dist_drone_to_goal = std::hypot(
        drone_pose_.pose.position.x - original_goal_.point.point.x,
        drone_pose_.pose.position.y - original_goal_.point.point.y);
      double dist_frontier_to_goal = std::hypot(
        new_frontier.point.x - original_goal_.point.point.x,
        new_frontier.point.y - original_goal_.point.point.y);
      constexpr double FRONTIER_MARGIN = 0.5;
      if (dist_frontier_to_goal > dist_drone_to_goal + FRONTIER_MARGIN) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Replan: frontier [%.2f, %.2f] is farther from goal than drone "
          "(frontier=%.2fm, drone=%.2fm, margin=%.1fm). Aborting navigation.",
          new_frontier.point.x, new_frontier.point.y,
          dist_frontier_to_goal, dist_drone_to_goal, FRONTIER_MARGIN);
        navigation_aborted_ = true;
        return;
      }

      // Abort if the frontier is too close to the drone — the drone is already
      // at the nearest reachable cell and keeps replanning to the same spot.
      double dist_drone_to_frontier = std::hypot(
        drone_pose_.pose.position.x - new_frontier.point.x,
        drone_pose_.pose.position.y - new_frontier.point.y);
      constexpr double MIN_FRONTIER_DIST = 0.5;
      if (dist_drone_to_frontier < MIN_FRONTIER_DIST) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Replan: frontier [%.2f, %.2f] is only %.3fm away — drone is stuck at "
          "the closest reachable cell. Aborting navigation.",
          new_frontier.point.x, new_frontier.point.y, dist_drone_to_frontier);
        navigation_aborted_ = true;
        return;
      }

      as2_msgs::action::NavigateToPoint::Goal frontier_goal;
      frontier_goal.point            = new_frontier;
      frontier_goal.yaw              = original_goal_.yaw;
      frontier_goal.navigation_speed = original_goal_.navigation_speed;

      ret = path_planner_plugin_->on_activate(drone_pose_, frontier_goal);
      if (ret) {
        is_intermediate_goal_ = true;
      }
    }
  }

  if (!ret) {
    RCLCPP_ERROR(this->get_logger(), "Replan failed. Aborting navigation.");
    navigation_aborted_ = true;
    return;
  }

  path_ = path_planner_plugin_->path_;

  // Remove leading waypoints that are within DEAD_ZONE of the drone's current
  // position. RDP path simplification can place the first anchor point only
  // ~0.1 m behind the drone; FollowPath then drives the drone backward before
  // continuing forward, producing the observed sharp heading reversal.
  // A DEAD_ZONE of 0.4 m catches these artifacts while preserving legitimate
  // obstacle-avoidance waypoints that are farther away.
  constexpr double DEAD_ZONE = 0.4;
  while (path_.size() > 1) {
    const auto & wp = path_.front();
    double dx = wp.x - drone_pose_.pose.position.x;
    double dy = wp.y - drone_pose_.pose.position.y;
    if (std::sqrt(dx * dx + dy * dy) < DEAD_ZONE) {
      path_.erase(path_.begin());
    } else {
      break;
    }
  }
  path_.insert(path_.begin(), drone_pose_.pose.position);

  // ── DIAGNOSTIC BLOCK ────────────────────────────────────────────────────
  RCLCPP_INFO(
    this->get_logger(),
    "[DIAG replan] Drone pos at replan : [%.3f, %.3f, %.3f]",
    drone_pose_.pose.position.x,
    drone_pose_.pose.position.y,
    drone_pose_.pose.position.z);
  RCLCPP_INFO(
    this->get_logger(),
    "[DIAG replan] Path type           : %s | waypoints: %zu",
    is_intermediate_goal_ ? "FRONTIER" : "DIRECT-TO-GOAL",
    path_.size());
  for (size_t k = 0; k < path_.size(); ++k) {
    const auto & wp = path_[k];
    double dx = wp.x - drone_pose_.pose.position.x;
    double dy = wp.y - drone_pose_.pose.position.y;
    RCLCPP_INFO(
      this->get_logger(),
      "[DIAG replan] Waypoint[%zu/%zu] : [%.3f, %.3f] | delta_drone->[%.3f, %.3f]",
      k, path_.size() - 1, wp.x, wp.y, dx, dy);
  }
  // ── END DIAGNOSTIC ──────────────────────────────────────────────────────

  send_follow_path_goal(original_goal_.navigation_speed);
  last_sent_speed_ = original_goal_.navigation_speed;
}

// ── FollowPath goal helper ─────────────────────────────────────────────────

void PathPlannerBehavior::send_follow_path_goal(double max_speed)
{
  auto goal_msg = as2_msgs::action::FollowPath::Goal();
  goal_msg.header.frame_id = "earth";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.yaw = original_goal_.yaw;
  goal_msg.max_speed = static_cast<float>(max_speed);
  int i = 0;
  for (auto & p : path_) {
    as2_msgs::msg::PoseWithID pid;
    pid.id = std::to_string(i++);
    pid.pose.position = p;
    pid.pose.position.z = original_goal_.point.point.z;
    goal_msg.path.push_back(pid);
  }

  auto send_goal_options =
    rclcpp_action::Client<as2_msgs::action::FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &PathPlannerBehavior::follow_path_response_cbk, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &PathPlannerBehavior::follow_path_feedback_cbk, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
    &PathPlannerBehavior::follow_path_result_cbk, this, std::placeholders::_1);

  follow_path_client_->async_send_goal(goal_msg, send_goal_options);
}

// ── LiDAR Reactive Safety Layer ───────────────────────────────────────────

void PathPlannerBehavior::lidar_scan_cbk(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

bool PathPlannerBehavior::compute_travel_axis(geometry_msgs::msg::Vector3 & axis_out)
{
  if (path_.size() < 2) {
    return false;
  }

  // Find the first waypoint still meaningfully ahead of the drone
  constexpr double MIN_DIST = 0.15;
  for (const auto & wp : path_) {
    double dx = wp.x - drone_pose_.pose.position.x;
    double dy = wp.y - drone_pose_.pose.position.y;
    double d = std::hypot(dx, dy);
    if (d > MIN_DIST) {
      axis_out.x = dx / d;
      axis_out.y = dy / d;
      axis_out.z = 0.0;
      return true;
    }
  }
  return false;  // near goal — all waypoints already passed
}

int PathPlannerBehavior::count_corridor_hits(
  const sensor_msgs::msg::LaserScan & scan,
  const geometry_msgs::msg::Vector3 & axis,
  double & nearest_along_out)
{
  geometry_msgs::msg::TransformStamped tf_earth_from_lidar;
  try {
    tf_earth_from_lidar = tf_buffer_->lookupTransform(
      "earth", scan.header.frame_id,
      rclcpp::Time(scan.header.stamp),
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[LIDAR_SAFETY] TF lookup failed: %s", ex.what());
    consecutive_detections_ = 0;
    return 0;
  }

  int hits = 0;
  nearest_along_out = std::numeric_limits<double>::infinity();
  float angle = scan.angle_min;

  for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
    float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > static_cast<float>(lidar_danger_distance_)) {
      continue;
    }

    geometry_msgs::msg::PointStamped pt_lidar;
    pt_lidar.header = scan.header;
    pt_lidar.point.x = r * std::cos(angle);
    pt_lidar.point.y = r * std::sin(angle);
    pt_lidar.point.z = 0.0;

    geometry_msgs::msg::PointStamped pt_earth;
    tf2::doTransform(pt_lidar, pt_earth, tf_earth_from_lidar);

    double dx = pt_earth.point.x - drone_pose_.pose.position.x;
    double dy = pt_earth.point.y - drone_pose_.pose.position.y;
    double along = dx * axis.x + dy * axis.y;
    double lat_x = dx - along * axis.x;
    double lat_y = dy - along * axis.y;
    double lateral = std::hypot(lat_x, lat_y);

    if (along > 0.0 && along <= lidar_danger_distance_ && lateral <= lidar_corridor_half_width_) {
      // Only count hits in free/unknown cells — skip known map obstacles (static walls)
      // so the LiDAR layer only reacts to new/dynamic obstacles not yet confirmed in A*.
      geometry_msgs::msg::PointStamped pt_check;
      pt_check.header.frame_id = "earth";
      pt_check.header.stamp = this->get_clock()->now();
      pt_check.point = pt_earth.point;
      if (path_planner_plugin_->is_occupied(pt_check)) {
        continue;
      }
      hits++;
      if (along < nearest_along_out) {
        nearest_along_out = along;
      }
    }
  }

  return hits;
}

bool PathPlannerBehavior::evaluate_lidar_corridor(double & nearest_along)
{
  if (!last_scan_) {
    return false;
  }

  geometry_msgs::msg::Vector3 axis;
  if (!compute_travel_axis(axis)) {
    return false;
  }

  double nearest = std::numeric_limits<double>::infinity();
  int hits = count_corridor_hits(*last_scan_, axis, nearest);

  if (hits >= lidar_min_cluster_size_) {
    consecutive_detections_++;
  } else {
    consecutive_detections_ = 0;
  }

  if (consecutive_detections_ >= lidar_persistence_count_) {
    nearest_along = nearest;
    return true;
  }
  return false;
}

void PathPlannerBehavior::engage_lidar_brake()
{
  std_srvs::srv::Trigger::Request req;
  std_srvs::srv::Trigger::Response res;
  bool ok = follow_path_pause_client_->sendRequest(req, res, 2);
  if (ok && res.success) {
    RCLCPP_INFO(get_logger(), "[LIDAR_SAFETY] FollowPath paused (hover).");
    lidar_braking_ = true;
    lidar_brake_time_ = this->get_clock()->now();
  } else {
    // Fallback: cancel goal so map_check replan takes over
    RCLCPP_WARN(
      get_logger(),
      "[LIDAR_SAFETY] Pause service failed — falling back to cancel+await replan.");
    follow_path_client_->async_cancel_all_goals();
    waiting_for_map_check_replan_ = true;
    lidar_braking_ = true;
    lidar_brake_time_ = this->get_clock()->now();
  }
}

double PathPlannerBehavior::compute_safe_speed(double d) const
{
  double nav_speed = static_cast<double>(original_goal_.navigation_speed);
  double range = lidar_danger_distance_ - lidar_stop_distance_;
  if (range <= 0.0) {
    return 0.0;
  }
  double ratio = (d - lidar_stop_distance_) / range;
  double v = lidar_kp_ * ratio * nav_speed;
  return std::clamp(v, 0.0, nav_speed);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlannerBehavior)
