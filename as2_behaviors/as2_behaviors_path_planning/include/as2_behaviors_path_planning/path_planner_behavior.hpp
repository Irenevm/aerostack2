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
 *  \file       path_planner_behavior.hpp
 *  \brief      path_planner_behavior header file.
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernandez-Cortizas
 ********************************************************************************/

#ifndef AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_BEHAVIOR_HPP_
#define AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_BEHAVIOR_HPP_

#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_msgs/action/navigate_to_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pluginlib/class_loader.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_srvs/srv/trigger.hpp"
#include "as2_core/synchronous_service_client.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "as2_msgs/action/follow_path.hpp"
#include "as2_behaviors_path_planning/path_planner_plugin_base.hpp"

class PathPlannerBehavior
  : public as2_behavior::BehaviorServer<as2_msgs::action::NavigateToPoint>
{
public:
  explicit PathPlannerBehavior(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PathPlannerBehavior() {}

private:
  // Behavior action parameters
  as2_msgs::msg::YawMode yaw_mode_;
  as2_msgs::action::NavigateToPoint::Goal goal_;
  as2_msgs::action::NavigateToPoint::Feedback feedback_;
  as2_msgs::action::NavigateToPoint::Result result_;

  // Planner variables
  bool enable_visualization_ = false;
  bool enable_path_optimizer_ = false;
  geometry_msgs::msg::PoseStamped drone_pose_;
  double safety_distance_ = 1.0;  // aprox drone size [m]
  int drone_mask_factor_ = 1;  // factor to increase the masked area around the drone
  bool simplify_path_ = false; // enable path simplification using rdp
  double dist_to_line_threshold_ = 1.0; // maximum distance to straight line to filter a point
  std::vector<geometry_msgs::msg::Point> path_;

  bool navigation_aborted_ = false;
  std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> follow_path_feedback_;
  bool follow_path_rejected_ = false;
  bool follow_path_succeeded_ = false;
  // True between an accepted FollowPath goal response and its result callback.
  // Gates whether a replan can use the "modify" service (in-place waypoint
  // update on the running goal) instead of cancel + brand-new goal.
  bool follow_path_active_ = false;

  // Reactive re-navigation loop
  as2_msgs::action::NavigateToPoint::Goal original_goal_;
  bool is_intermediate_goal_ = false;
  bool need_replan_ = false;
  int replan_count_ = 0;
  int max_replans_ = 15;

  // Periodic map-check: detect when direct path to goal becomes available
  rclcpp::TimerBase::SharedPtr map_check_timer_;
  bool check_map_ = false;
  geometry_msgs::msg::PoseStamped pose_at_plan_start_;
  // Set when MAP_CHECK cancels FollowPath; replan fires only after cancel is confirmed
  // to avoid cancel_all_goals() killing the newly-sent FollowPath goal.
  bool waiting_for_map_check_replan_ = false;

  // ── LiDAR Reactive Safety Layer ──────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::TimerBase::SharedPtr lidar_check_timer_;
  bool check_lidar_ = false;
  bool lidar_braking_ = false;      // FollowPath is paused due to LiDAR
  int  consecutive_detections_ = 0; // persistence counter
  double last_sent_speed_ = 0.0;    // last max_speed sent to FollowPath (histéresis)
  rclcpp::Time lidar_brake_time_;   // when braking started (for timeout warn)

  // LiDAR safety parameters
  bool   enable_lidar_safety_ = true;
  double lidar_check_period_ = 0.05;
  double lidar_danger_distance_ = 1.5;
  double lidar_stop_distance_ = 0.7;
  double lidar_corridor_half_width_ = 0.5;
  int    lidar_min_cluster_size_ = 3;
  int    lidar_persistence_count_ = 2;
  bool   enable_lidar_decel_ = true;
  double lidar_kp_ = 1.0;
  double decel_speed_epsilon_ = 0.1;

private:
  /** As2 Behavior methods **/
  bool on_activate(std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal) override;

  bool on_modify(std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> goal) override;

  bool on_deactivate(const std::shared_ptr<std::string> & message) override;

  bool on_pause(const std::shared_ptr<std::string> & message) override;

  bool on_resume(const std::shared_ptr<std::string> & message) override;

  as2_behavior::ExecutionStatus on_run(
    const std::shared_ptr<const as2_msgs::action::NavigateToPoint::Goal> & goal,
    std::shared_ptr<as2_msgs::action::NavigateToPoint::Feedback> & feedback_msg,
    std::shared_ptr<as2_msgs::action::NavigateToPoint::Result> & result_msg) override;

  void on_execution_end(const as2_behavior::ExecutionStatus & state) override;

private:
  /** Path Planner Behavior plugin **/
  std::filesystem::path plugin_name_;
  std::shared_ptr<pluginlib::ClassLoader<as2_behaviors_path_planning::PluginBase>> loader_;
  std::shared_ptr<as2_behaviors_path_planning::PluginBase> path_planner_plugin_;

  /* Other ROS 2 interfaces */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
  // TODO(pariaspe): where to place visualization publisher. In plugin or here?
  // For path returned by plugin probably here
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  rclcpp_action::Client<as2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr follow_path_pause_client_ =
    nullptr;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr follow_path_resume_client_ =
    nullptr;
  // Updates the running FollowPath goal's waypoints in place (no cancel, no
  // stop-and-restart): calls own_modify() on the plugin instead of tearing
  // down and resending the whole action goal.
  as2::SynchronousServiceClient<as2_msgs::action::FollowPath::Impl::SendGoalService>::SharedPtr
    follow_path_modify_client_ = nullptr;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private:
  void drone_pose_cbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void trigger_replan();

  // FollowPath Action Client
  void follow_path_response_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr & goal_handle);
  void follow_path_feedback_cbk(
    rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::SharedPtr goal_handle,
    const std::shared_ptr<const as2_msgs::action::FollowPath::Feedback> feedback);
  void follow_path_result_cbk(
    const rclcpp_action::ClientGoalHandle<as2_msgs::action::FollowPath>::WrappedResult & result);

  // FollowPath goal helper (eliminates duplicated goal-build/send code)
  void send_follow_path_goal(double max_speed);

  // Builds the same goal message as send_follow_path_goal() but sends it
  // through the "modify" service instead of the action client, so the
  // running goal is updated in place. Returns whether the plugin accepted it.
  bool send_follow_path_modify(double max_speed);

  // LiDAR reactive safety
  void lidar_scan_cbk(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool compute_travel_axis(geometry_msgs::msg::Vector3 & axis_out);
  int  count_corridor_hits(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::Vector3 & axis,
    double & nearest_along_out);
  bool evaluate_lidar_corridor(double & nearest_along);
  void engage_lidar_brake();
  double compute_safe_speed(double d) const;
};

#endif  // AS2_BEHAVIORS_PATH_PLANNING__PATH_PLANNER_BEHAVIOR_HPP_
