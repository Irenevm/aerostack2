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

  // Frontier-arrival settle retry, see .hpp for full rationale.
  this->declare_parameter("frontier_stuck_settle_s", frontier_stuck_settle_s_);
  frontier_stuck_settle_s_ = this->get_parameter("frontier_stuck_settle_s").as_double();
  this->declare_parameter("frontier_stuck_max_retries", frontier_stuck_max_retries_);
  frontier_stuck_max_retries_ = this->get_parameter("frontier_stuck_max_retries").as_int();

  // Physics-based reactive braking corridor, see .hpp for full rationale.
  this->declare_parameter("corridor_half_angle_deg", corridor_half_angle_deg_);
  corridor_half_angle_deg_ = this->get_parameter("corridor_half_angle_deg").as_double();
  this->declare_parameter("corridor_half_width", corridor_half_width_);
  corridor_half_width_ = this->get_parameter("corridor_half_width").as_double();
  this->declare_parameter("corridor_k_brake_s", corridor_k_brake_s_);
  corridor_k_brake_s_ = this->get_parameter("corridor_k_brake_s").as_double();
  this->declare_parameter("corridor_l_detect_s", corridor_l_detect_s_);
  corridor_l_detect_s_ = this->get_parameter("corridor_l_detect_s").as_double();
  this->declare_parameter("corridor_margin_m", corridor_margin_m_);
  corridor_margin_m_ = this->get_parameter("corridor_margin_m").as_double();
  this->declare_parameter("corridor_stop_distance", corridor_stop_distance_);
  corridor_stop_distance_ = this->get_parameter("corridor_stop_distance").as_double();
  this->declare_parameter("corridor_floor_speed", corridor_floor_speed_);
  corridor_floor_speed_ = this->get_parameter("corridor_floor_speed").as_double();
  this->declare_parameter("corridor_min_cluster", corridor_min_cluster_);
  corridor_min_cluster_ = this->get_parameter("corridor_min_cluster").as_int();
  this->declare_parameter("corridor_persistence", corridor_persistence_);
  corridor_persistence_ = this->get_parameter("corridor_persistence").as_int();
  this->declare_parameter("corridor_speed_epsilon", corridor_speed_epsilon_);
  corridor_speed_epsilon_ = this->get_parameter("corridor_speed_epsilon").as_double();
  this->declare_parameter("corridor_post_replan_grace_s", corridor_post_replan_grace_s_);
  corridor_post_replan_grace_s_ =
    this->get_parameter("corridor_post_replan_grace_s").as_double();
  this->declare_parameter("corridor_min_brake_hold_s", corridor_min_brake_hold_s_);
  corridor_min_brake_hold_s_ = this->get_parameter("corridor_min_brake_hold_s").as_double();
  this->declare_parameter("corridor_min_modify_interval_s", corridor_min_modify_interval_s_);
  corridor_min_modify_interval_s_ =
    this->get_parameter("corridor_min_modify_interval_s").as_double();

  this->declare_parameter("corridor_check_period", 0.05);
  double corridor_check_period = this->get_parameter("corridor_check_period").as_double();
  corridor_check_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(corridor_check_period),
    [this]() { check_corridor_ = true; });
  corridor_check_timer_->cancel();

  lidar_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "sensor_measurements/lidar/scan",
    rclcpp::SensorDataQoS(),
    std::bind(&PathPlannerBehavior::lidar_scan_cbk, this, std::placeholders::_1));

  this->declare_parameter("map_check_period", 2.0);
  double map_check_period = this->get_parameter("map_check_period").as_double();
  map_check_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(map_check_period),
    [this]() { check_map_ = true; });
  map_check_timer_->cancel();

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

  follow_path_modify_client_ = std::make_shared<as2::SynchronousServiceClient<
      as2_msgs::action::FollowPath::Impl::SendGoalService>>(
    std::string(as2_names::actions::behaviors::followpath) + "/_behavior/modify", this);
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
  frontier_retry_pending_ = false;
  frontier_stuck_retries_ = 0;
  check_map_ = false;
  waiting_for_map_check_replan_ = false;
  follow_path_active_ = false;
  pose_at_plan_start_ = drone_pose_;
  map_check_timer_->reset();

  check_corridor_ = false;
  corridor_confirm_count_ = 0;
  corridor_braking_ = false;
  last_corridor_speed_sent_ = -1.0;
  corridor_grace_until_ = this->get_clock()->now();
  corridor_nearest_at_replan_ = std::numeric_limits<double>::infinity();
  corridor_last_modify_time_ =
    this->get_clock()->now() - rclcpp::Duration::from_seconds(corridor_min_modify_interval_s_);
  corridor_check_timer_->reset();

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
  corridor_check_timer_->cancel();
  check_corridor_ = false;
  waiting_for_map_check_replan_ = false;
  // Cancel only the goal started from navigation. Behaviors only accepts
  // one goal simultaneously, don't have to worry about
  follow_path_client_->async_cancel_all_goals();
  follow_path_active_ = false;
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

  // Frontier-arrival settle retry: waiting for M_g to consolidate before
  // trying trigger_replan() again, see .hpp for full rationale.
  if (frontier_retry_pending_ && this->get_clock()->now() >= frontier_retry_deadline_) {
    frontier_retry_pending_ = false;
    need_replan_ = true;
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

  // Physics-based reactive braking corridor: runs every tick regardless of
  // MAP_CHECK state, so it can react to an obstacle at any point in the
  // flight, not just right after a replan. See .hpp for full rationale.
  if (check_corridor_) {
    check_corridor_ = false;
    if (follow_path_active_ && !need_replan_ && !waiting_for_map_check_replan_) {
      // The grace window itself is no longer checked here — it's evaluated
      // inside check_reactive_corridor(), which only lets it suppress a new
      // brake decision when nothing has gotten worse since the replan (see
      // corridor_nearest_at_replan_ in the .hpp for why: a blanket time-based
      // block here left the corridor blind to a genuinely new, unrelated
      // obstacle for the full grace duration, confirmed 2026-07-29 both live
      // and in batteries — the obstacle was already within ~1.0-1.2m by the
      // time MAP_CHECK's own slower detection caught it).
      check_reactive_corridor();
    }
  }

  if (check_map_) {
    check_map_ = false;
    // Run A* to update the internal graph with the current map, and check
    // whether the original goal is directly reachable from here.
    bool can_reach_goal = path_planner_plugin_->on_activate(drone_pose_, original_goal_);
    RCLCPP_INFO(
      this->get_logger(),
      "[MAP_CHECK] tick — drone=[%.2f, %.2f] intermediate=%s can_reach_goal=%s",
      drone_pose_.pose.position.x, drone_pose_.pose.position.y,
      is_intermediate_goal_ ? "true" : "false",
      can_reach_goal ? "true" : "false");

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
      RCLCPP_INFO(
        this->get_logger(),
        "[LAT] FRONTIER_EXIT t=%.6f — obstacle detection re-enabled",
        this->get_clock()->now().seconds());
      // Replan in place via trigger_replan() -> modify(): no cancel, the drone
      // keeps flying its current segment until the new path takes effect.
      waiting_for_map_check_replan_ = true;
      need_replan_ = true;
    } else if (!can_reach_goal && !is_intermediate_goal_) {
      // Hito B (complete blockage): A* can no longer reach the goal at all.
      RCLCPP_WARN(
        this->get_logger(),
        "[MAP_CHECK] Path to goal [%.2f, %.2f] is now blocked. Replanning.",
        original_goal_.point.point.x, original_goal_.point.point.y);
      waiting_for_map_check_replan_ = true;
      need_replan_ = true;
    }

    // Hito B (path segment check): sample every segment of the *currently active*
    // path_ — frontier leg or direct-to-goal leg, whichever we're flying — to
    // detect obstacles that fall between RDP-reduced waypoints. This runs
    // regardless of is_intermediate_goal_: a frontier path was computed with
    // unknown_as_free=false too, so it never crosses unexplored cells, only
    // known-free ones — is_occupied() here can only flag a genuinely new
    // obstacle, not stale "unknown" cells. Gating this on !is_intermediate_goal_
    // (as before) left the drone architecturally blind to new obstacles for the
    // entire duration of a frontier leg (see [LAT] FRONTIER_ENTER/EXIT bug).
    if (!need_replan_ && !waiting_for_map_check_replan_) {
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
              RCLCPP_WARN(
                this->get_logger(),
                "[LAT] DETECT t=%.6f pos=[%.2f,%.2f]",
                this->get_clock()->now().seconds(), sx, sy);
              waiting_for_map_check_replan_ = true;
              need_replan_ = true;
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
    follow_path_active_ = true;
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
  // Any result means this goal is no longer running (modify() never produces
  // one, since it updates the same goal in place without going through the
  // action client).
  follow_path_active_ = false;

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
  waiting_for_map_check_replan_ = false;
  pose_at_plan_start_ = drone_pose_;

  if (++replan_count_ > max_replans_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Replan: exceeded %d replans without reaching goal. Aborting navigation.", max_replans_);
    if (follow_path_active_) {
      follow_path_client_->async_cancel_all_goals();
      follow_path_active_ = false;
    }
    navigation_aborted_ = true;
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Replanning to original goal [%.2f, %.2f, %.2f]",
    original_goal_.point.point.x, original_goal_.point.point.y,
    original_goal_.point.point.z);
  double lat_t_replan_start = this->get_clock()->now().seconds();
  RCLCPP_INFO(this->get_logger(), "[LAT] REPLAN_START t=%.6f", lat_t_replan_start);

  bool ret = path_planner_plugin_->on_activate(drone_pose_, original_goal_);
  RCLCPP_INFO(
    this->get_logger(), "[LAT] ASTAR_DONE t=%.6f dt=%.6f",
    this->get_clock()->now().seconds(),
    this->get_clock()->now().seconds() - lat_t_replan_start);
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
        if (follow_path_active_) {
          follow_path_client_->async_cancel_all_goals();
          follow_path_active_ = false;
        }
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
        if (follow_path_active_) {
          follow_path_client_->async_cancel_all_goals();
          follow_path_active_ = false;
        }
        navigation_aborted_ = true;
        return;
      }

      // The frontier being this close to the drone usually does NOT mean a
      // genuine dead end — it usually means M_g (the consolidated occupancy
      // map A* reads) hasn't caught up yet with what the LiDAR already saw
      // while approaching (measured lag: 0.15-8s, median ~1.7s, in the
      // Dmin(v) latency study). Retry a bounded number of times, waiting
      // between attempts, before concluding it's a genuine dead end.
      double dist_drone_to_frontier = std::hypot(
        drone_pose_.pose.position.x - new_frontier.point.x,
        drone_pose_.pose.position.y - new_frontier.point.y);
      constexpr double MIN_FRONTIER_DIST = 0.5;
      if (dist_drone_to_frontier < MIN_FRONTIER_DIST) {
        if (frontier_stuck_retries_ < frontier_stuck_max_retries_) {
          frontier_stuck_retries_++;
          RCLCPP_WARN(
            this->get_logger(),
            "Replan: frontier [%.2f, %.2f] is only %.3fm away — map may not have "
            "consolidated yet. Retrying in %.1fs (%d/%d).",
            new_frontier.point.x, new_frontier.point.y, dist_drone_to_frontier,
            frontier_stuck_settle_s_, frontier_stuck_retries_, frontier_stuck_max_retries_);
          frontier_retry_pending_ = true;
          frontier_retry_deadline_ =
            this->get_clock()->now() + rclcpp::Duration::from_seconds(frontier_stuck_settle_s_);
          return;
        }
        RCLCPP_ERROR(
          this->get_logger(),
          "Replan: frontier [%.2f, %.2f] is still only %.3fm away after %d retries — "
          "drone is genuinely stuck at the closest reachable cell. Aborting navigation.",
          new_frontier.point.x, new_frontier.point.y, dist_drone_to_frontier,
          frontier_stuck_retries_);
        if (follow_path_active_) {
          follow_path_client_->async_cancel_all_goals();
          follow_path_active_ = false;
        }
        navigation_aborted_ = true;
        return;
      }
      frontier_stuck_retries_ = 0;

      as2_msgs::action::NavigateToPoint::Goal frontier_goal;
      frontier_goal.point            = new_frontier;
      frontier_goal.yaw              = original_goal_.yaw;
      frontier_goal.navigation_speed = original_goal_.navigation_speed;

      ret = path_planner_plugin_->on_activate(drone_pose_, frontier_goal);
      if (ret) {
        is_intermediate_goal_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "[LAT] FRONTIER_ENTER t=%.6f — full-block check disabled until Hito A "
          "fires or frontier reached; segment-check stays active on the frontier leg",
          this->get_clock()->now().seconds());
      }
    }
  }

  if (!ret) {
    RCLCPP_ERROR(this->get_logger(), "Replan failed. Aborting navigation.");
    if (follow_path_active_) {
      follow_path_client_->async_cancel_all_goals();
      follow_path_active_ = false;
    }
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

  // If FollowPath is already flying a goal, update its waypoints in place via
  // the "modify" service — no cancel, no stop-and-restart, so the drone keeps
  // moving while the new path takes effect. Only fall back to a fresh
  // cancel+send when there's no running goal to modify (first activation,
  // previous goal was rejected, or it already finished) or the plugin
  // rejects the modify outright.
  // If the corridor is already braking, a replan (MAP_CHECK/frontier) only
  // changes WHERE the drone is going — it says nothing about whether the
  // obstacle the corridor is braking for is still there. Keep the reduced
  // speed instead of silently resuming cruise speed underneath the corridor;
  // this was previously a priority-inversion bug (confirmed both in plain
  // "recto" and in frontier mode, 2026-07-29): a MAP_CHECK replan firing a
  // few tens of ms after REACTIVE_DECEL would cancel the corridor's braking
  // and resend the path at full cruise speed, right as the drone was
  // closest to the obstacle.
  double replan_speed = corridor_braking_ ?
    last_corridor_speed_sent_ : original_goal_.navigation_speed;

  bool replanned_in_place = false;
  if (follow_path_active_) {
    replanned_in_place = send_follow_path_modify(replan_speed);
    RCLCPP_INFO(
      this->get_logger(), "[LAT] MODIFY_%s t=%.6f",
      replanned_in_place ? "ACCEPTED" : "REJECTED",
      this->get_clock()->now().seconds());
    if (!replanned_in_place) {
      RCLCPP_WARN(
        this->get_logger(),
        "FollowPath modify rejected — falling back to cancel + resend.");
      follow_path_client_->async_cancel_all_goals();
      follow_path_active_ = false;
    }
  }
  if (!replanned_in_place) {
    send_follow_path_goal(replan_speed);
    RCLCPP_INFO(
      this->get_logger(), "[LAT] GOAL_SENT t=%.6f",
      this->get_clock()->now().seconds());
  }

  // Trust the just-computed path for a grace period — see .hpp for why:
  // A* only guarantees safety_distance_ of clearance on a tight detour,
  // which can be closer than this corridor's own danger_distance would
  // otherwise re-trigger on. But only re-arm the grace period / clear the
  // braking state when the corridor wasn't already actively braking — if it
  // was, leave it running and let its own release logic (omni_nearest +
  // corridor_min_brake_hold_s) decide when the obstacle is actually clear.
  if (!corridor_braking_) {
    corridor_grace_until_ =
      this->get_clock()->now() + rclcpp::Duration::from_seconds(corridor_post_replan_grace_s_);
    // Baseline for the grace override (2026-07-29): capture what the
    // corridor sees right now, so a later reading that's genuinely worse
    // (or newly detects something when nothing was visible here) can still
    // break through the grace window instead of being blindly suppressed.
    corridor_nearest_at_replan_ = std::numeric_limits<double>::infinity();
    evaluate_lidar_corridor(corridor_nearest_at_replan_);
  }
  corridor_confirm_count_ = 0;
}

// ── Physics-based reactive braking corridor ─────────────────────────────────

void PathPlannerBehavior::lidar_scan_cbk(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

bool PathPlannerBehavior::evaluate_lidar_corridor(double & nearest_range)
{
  if (!last_scan_ || last_scan_->ranges.empty()) {
    return false;
  }

  // Forward direction: angle 0 in the scan message IS the drone's forward
  // direction (matches telemetry_logger.py's validated convention — the
  // scan frame is body-fixed under PATH_FACING). Deriving this instead from
  // tf2::getYaw(drone orientation) was tried in an earlier attempt (see
  // archive/reactive-corridor-attempt) and proved too noisy tick-to-tick to
  // use.
  const double target_angle_body = 0.0;

  double danger_distance = std::numeric_limits<double>::infinity();
  {
    double v = static_cast<double>(original_goal_.navigation_speed);
    danger_distance = v * (corridor_k_brake_s_ + corridor_l_detect_s_) + corridor_margin_m_;
  }
  // Generous upper bound for how far out we even bother scanning — kept
  // under the ~3.0-4.8m ground-ring LiDAR artifact documented in the
  // 2026-07-24 latency study (lowest vertical ring hitting the ground).
  double max_check_range = std::min(danger_distance + 0.5, 2.6);

  double half_angle_rad = corridor_half_angle_deg_ * M_PI / 180.0;
  int n = static_cast<int>(last_scan_->ranges.size());
  int contiguous = 0;
  int best_contiguous = 0;
  double best_min_range = std::numeric_limits<double>::infinity();
  double running_min = std::numeric_limits<double>::infinity();

  for (int i = 0; i < n; ++i) {
    double angle = last_scan_->angle_min + i * last_scan_->angle_increment;
    double delta = angle - target_angle_body;
    while (delta > M_PI) {delta -= 2 * M_PI;}
    while (delta < -M_PI) {delta += 2 * M_PI;}

    float r = last_scan_->ranges[i];
    bool qualifies = false;
    if (std::isfinite(r) && r >= last_scan_->range_min && r <= last_scan_->range_max &&
      r <= max_check_range && std::fabs(delta) <= half_angle_rad)
    {
      double lateral = r * std::sin(delta);
      if (std::fabs(lateral) <= corridor_half_width_) {
        qualifies = true;
      }
    }

    if (qualifies) {
      contiguous++;
      running_min = std::min(running_min, static_cast<double>(r));
      if (contiguous > best_contiguous) {
        best_contiguous = contiguous;
        best_min_range = running_min;
      }
    } else {
      contiguous = 0;
      running_min = std::numeric_limits<double>::infinity();
    }
  }

  if (best_contiguous >= corridor_min_cluster_) {
    nearest_range = best_min_range;
    return true;
  }
  return false;
}

double PathPlannerBehavior::nearest_omnidirectional_range(double max_range)
{
  if (!last_scan_) {return std::numeric_limits<double>::infinity();}
  double best = std::numeric_limits<double>::infinity();
  for (float r : last_scan_->ranges) {
    if (std::isfinite(r) && r >= last_scan_->range_min && r <= max_range) {
      best = std::min(best, static_cast<double>(r));
    }
  }
  return best;
}

void PathPlannerBehavior::check_reactive_corridor()
{
  double cruise_speed = static_cast<double>(original_goal_.navigation_speed);
  double danger_distance =
    cruise_speed * (corridor_k_brake_s_ + corridor_l_detect_s_) + corridor_margin_m_;

  double nearest = std::numeric_limits<double>::infinity();
  bool obstacle = evaluate_lidar_corridor(nearest);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "[LAT] CORRIDOR_DEBUG t=%.6f obstacle=%s nearest=%.2f danger=%.2f",
    this->get_clock()->now().seconds(), obstacle ? "true" : "false", nearest, danger_distance);

  if (obstacle && nearest < danger_distance) {
    // Grace override (2026-07-29): if we're still inside the post-replan
    // grace window AND not already braking, only let this through when the
    // situation is genuinely worse than it was when the replan fired —
    // otherwise a replan that had nothing to do with this obstacle (e.g. a
    // far-away segment fix or a frontier hop) would blind the corridor to
    // it for the full grace duration. "Worse" = closer than the baseline
    // captured at replan time (which is +inf if nothing was visible then,
    // so a brand-new detection always passes).
    if (!corridor_braking_ && this->get_clock()->now() < corridor_grace_until_ &&
      nearest >= corridor_nearest_at_replan_)
    {
      corridor_confirm_count_ = 0;
      return;
    }
    corridor_confirm_count_++;
    if (corridor_confirm_count_ < corridor_persistence_) {
      return;
    }
    // Binary decision, not a continuous proportional ramp: "can I stop in
    // time? no -> brake; yes -> don't touch anything" (this is the actual
    // physics question danger_distance answers). A continuous ramp needs
    // many modify() calls as distance shrinks (~15 in one flight, observed
    // 2026-07-27) — and *every* modify() call makes FollowPath's
    // position-mode plugin re-anchor from a fresh short path near the
    // drone, which visibly jerks it backward before continuing forward
    // (the same artifact trigger_replan()'s DEAD_ZONE trim exists for).
    // Many of those jerks in a row compounded into multi-metre oscillation.
    // Sending the floor speed exactly once when entering the brake state
    // (and cruise speed exactly once on release, below) keeps modify()
    // calls rare enough that this artifact stays a one-off, not a pattern.
    if (!corridor_braking_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[LAT] REACTIVE_DECEL t=%.6f d=%.2fm v=%.2f->%.2f",
        this->get_clock()->now().seconds(), nearest, cruise_speed, corridor_floor_speed_);
      send_follow_path_modify_from_here(corridor_floor_speed_);
      last_corridor_speed_sent_ = corridor_floor_speed_;
      corridor_last_modify_time_ = this->get_clock()->now();
      corridor_braking_since_ = this->get_clock()->now();
    }
    corridor_braking_ = true;
  } else {
    corridor_confirm_count_ = 0;
    // Release condition is deliberately omnidirectional (not the narrow
    // forward corridor): a replan can change the target waypoint direction
    // within one tick, instantly making the same, still-close obstacle
    // read as "outside the corridor" before the drone has actually moved.
    // Also requires a minimum hold time to avoid chattering on readings
    // that hover right at the threshold.
    if (corridor_braking_) {
      double time_braking = (this->get_clock()->now() - corridor_braking_since_).seconds();
      double omni_nearest = nearest_omnidirectional_range(corridor_stop_distance_ * 2.5);
      if (time_braking >= corridor_min_brake_hold_s_ &&
        omni_nearest > corridor_stop_distance_ * 2.2)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[LAT] REACTIVE_CLEAR t=%.6f omni_nearest=%.2f held=%.2fs — restoring %.2f",
          this->get_clock()->now().seconds(), omni_nearest, time_braking, cruise_speed);
        send_follow_path_modify_from_here(cruise_speed);
        last_corridor_speed_sent_ = cruise_speed;
        corridor_last_modify_time_ = this->get_clock()->now();
        corridor_braking_ = false;
      }
    }
  }
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

bool PathPlannerBehavior::send_follow_path_modify(double max_speed)
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

  as2_msgs::action::FollowPath::Impl::SendGoalService::Request req;
  req.goal = goal_msg;
  as2_msgs::action::FollowPath::Impl::SendGoalService::Response res;
  bool ok = follow_path_modify_client_->sendRequest(req, res, 1);
  return ok && res.accepted;
}

// Same as send_follow_path_modify(), but rebuilds the waypoint list from the
// drone's CURRENT position instead of reusing path_ as-is. path_[0] is the
// drone's position at the *last actual replan* (see trigger_replan()) — for
// a single modify() right after a replan that's still fresh, but the
// reactive corridor below calls this repeatedly, seconds apart, as it
// ramps speed up and down. Resending path_ unchanged each time means
// resending that now-stale waypoint[0], and FollowPath's position-mode
// plugin restarts tracking from it — observed directly as the drone
// visibly reversing back towards where it was several seconds earlier,
// every single time the corridor changed speed (2026-07-27).
bool PathPlannerBehavior::send_follow_path_modify_from_here(double max_speed)
{
  if (path_.size() < 2) {
    return send_follow_path_modify(max_speed);
  }
  // Distance-only trimming (as trigger_replan()'s own DEAD_ZONE does) is not
  // enough here: it only drops a *leading* waypoint if it happens to be
  // within 0.4m of the drone, and stops at the first one that isn't. If the
  // drone has travelled further than that since the last real replan
  // (easily 2-3m during the 2.5s post-replan grace at cruise speed) several
  // waypoints can already be behind it without ever being that close, and
  // none of them get dropped — sent as-is, FollowPath routes the drone
  // backward through all of them before it resumes forward progress
  // (observed directly in RViz as "a lot of waypoints behind the intended
  // trajectory", 2026-07-27). Instead, find which *segment* of path_ the
  // drone is currently nearest to (same approach used for the corridor's
  // own forward-direction lookup) and keep only that segment's far
  // endpoint onward — independent of how far the drone has travelled.
  std::vector<geometry_msgs::msg::Point> corrected;
  {
    size_t best_i = 0;
    double best_seg_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < path_.size(); ++i) {
      const auto & p1 = path_[i];
      const auto & p2 = path_[i + 1];
      double dx = p2.x - p1.x, dy = p2.y - p1.y;
      double seg_len2 = dx * dx + dy * dy;
      double t = seg_len2 > 1e-9 ?
        ((drone_pose_.pose.position.x - p1.x) * dx +
        (drone_pose_.pose.position.y - p1.y) * dy) / seg_len2 : 0.0;
      t = std::clamp(t, 0.0, 1.0);
      double px = p1.x + t * dx, py = p1.y + t * dy;
      double d = std::hypot(
        px - drone_pose_.pose.position.x, py - drone_pose_.pose.position.y);
      if (d < best_seg_dist) {
        best_seg_dist = d;
        best_i = i;
      }
    }
    corrected.assign(path_.begin() + best_i + 1, path_.end());
  }
  constexpr double DEAD_ZONE = 0.4;
  while (corrected.size() > 1) {
    const auto & wp = corrected.front();
    double dx = wp.x - drone_pose_.pose.position.x;
    double dy = wp.y - drone_pose_.pose.position.y;
    if (std::sqrt(dx * dx + dy * dy) < DEAD_ZONE) {
      corrected.erase(corrected.begin());
    } else {
      break;
    }
  }
  corrected.insert(corrected.begin(), drone_pose_.pose.position);

  // ── DIAGNOSTIC BLOCK (2026-07-29) ───────────────────────────────────────
  // Same format as trigger_replan()'s DIAG block, but for corridor-driven
  // modify() calls specifically — the previous DIAG block never fired here,
  // so a stale/behind waypoint sent by the corridor's own speed changes was
  // invisible in the logs.
  RCLCPP_INFO(
    this->get_logger(),
    "[DIAG corridor_modify] Drone pos : [%.3f, %.3f, %.3f] | max_speed=%.2f",
    drone_pose_.pose.position.x, drone_pose_.pose.position.y,
    drone_pose_.pose.position.z, max_speed);
  for (size_t k = 0; k < corrected.size(); ++k) {
    const auto & wp = corrected[k];
    double dx = wp.x - drone_pose_.pose.position.x;
    double dy = wp.y - drone_pose_.pose.position.y;
    RCLCPP_INFO(
      this->get_logger(),
      "[DIAG corridor_modify] Waypoint[%zu/%zu] : [%.3f, %.3f] | delta_drone->[%.3f, %.3f]",
      k, corrected.size() - 1, wp.x, wp.y, dx, dy);
  }
  // ── END DIAGNOSTIC ──────────────────────────────────────────────────────

  auto goal_msg = as2_msgs::action::FollowPath::Goal();
  goal_msg.header.frame_id = "earth";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.yaw = original_goal_.yaw;
  goal_msg.max_speed = static_cast<float>(max_speed);
  int i = 0;
  for (auto & p : corrected) {
    as2_msgs::msg::PoseWithID pid;
    pid.id = std::to_string(i++);
    pid.pose.position = p;
    pid.pose.position.z = original_goal_.point.point.z;
    goal_msg.path.push_back(pid);
  }

  as2_msgs::action::FollowPath::Impl::SendGoalService::Request req;
  req.goal = goal_msg;
  as2_msgs::action::FollowPath::Impl::SendGoalService::Response res;
  bool ok = follow_path_modify_client_->sendRequest(req, res, 1);
  return ok && res.accepted;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlannerBehavior)
