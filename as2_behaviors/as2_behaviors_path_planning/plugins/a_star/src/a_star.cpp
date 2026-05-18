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
 *  \file       a_star.cpp
 *  \brief      a_star implementation file.
 *  \authors    Pedro Arias Pérez
 ********************************************************************************/

#include <a_star.hpp>
#include <utils.hpp>

#include <deque>
#include <limits>

namespace a_star
{
void Plugin::initialize(as2::Node * node_ptr, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ptr_ = node_ptr;
  tf_buffer_ = tf_buffer;

  a_star_searcher_ = AStarSearcher();

  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing A* plugin");

  // node_ptr_->declare_parameter("safety_distance", 0.5);
  safety_distance_ = node_ptr_->get_parameter("safety_distance").as_double();

  drone_mask_factor_ = node_ptr_->get_parameter("drone_mask_factor").as_int();

  // node_ptr_->declare_parameter("enable_path_optimizer", false);
  enable_path_optimizer_ = node_ptr_->get_parameter("enable_path_optimizer").as_bool();

  // node_ptr_->declare_parameter("enable_visualization", true);
  enable_visualization_ = node_ptr_->get_parameter("enable_visualization").as_bool();
  enable_visualization_ = true;  // TODO(pariaspe): not publish when false

  // simplify_path_ = node_ptr_->get_parameter("simplify_path)").as_bool();
  // dist_to_line_threshold_ = node_ptr->get_parameter("dist_to_line_threshold").as_double();

  simplify_path_ = true;
  dist_to_line_threshold_ = 1.2;

  occ_grid_sub_ = node_ptr_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 1, std::bind(&Plugin::occ_grid_cbk, this, std::placeholders::_1));

  if (enable_visualization_) {
    viz_pub_ =
      node_ptr_->create_publisher<visualization_msgs::msg::Marker>("plugin_viz/marker", 10);
    viz_obstacle_grid_pub_ =
      node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>("plugin_viz/obstacle_map", 10);
  }
}

void Plugin::occ_grid_cbk(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  last_occ_grid_ = *(msg);
  // RCLCPP_INFO(
  //   node_ptr_->get_logger(), "Received occupancy grid with size [%d, %d]",
  //   last_occ_grid_.info.width, last_occ_grid_.info.height);
}

bool Plugin::on_activate(
  geometry_msgs::msg::PoseStamped drone_pose, as2_msgs::action::NavigateToPoint::Goal goal)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Activating A* plugin");
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Drone pose: [%f, %f] (%s)", drone_pose.pose.position.x,
    drone_pose.pose.position.y, drone_pose.header.frame_id.c_str());
  RCLCPP_INFO(
    node_ptr_->get_logger(), "Going to [%f, %f] (%s)", goal.point.point.x,
    goal.point.point.y, goal.point.header.frame_id.c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "Target frame (%s)", last_occ_grid_.header.frame_id.c_str());

  Point2i goal_cell = utils::pointToCell(
    goal.point, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
  Point2i drone_cell = utils::poseToCell(
    drone_pose, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);

  if (n_times_executed_ > 0) {
    std::cout << "Recomputing path. Previous path executed " << n_times_executed_ << " times."
              << std::endl;
    drone_mask_factor_ = 1;
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Computing first path with configured drone mask");
  }

  auto test = a_star_searcher_.update_grid(
    last_occ_grid_, drone_cell, safety_distance_,
    drone_mask_factor_);

  // n_times_executed_++;    //  vaya guarrada jajajajaj

  RCLCPP_INFO(node_ptr_->get_logger(), "Publishing obstacle map");
  viz_obstacle_grid_pub_->publish(test);

  std::vector<Point2i> path = a_star_searcher_.solve_graph(drone_cell, goal_cell);
  if (path.size() == 0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Path to goal not found. Goal Rejected.");
    return false;
  }
  std::vector<Point2i> simplified_path = utils::simplify_path_rdp(path, dist_to_line_threshold_);
  // Aquí fusilo puntos del path
  if (simplify_path_) {

    n_times_executed_++;

    if (enable_path_optimizer_) {
      // TODO(pariaspe): Implement path optimizer
      RCLCPP_WARN(node_ptr_->get_logger(), "Path optimizer not implemented yet");
      // path = path_optimizer::solve(path);
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Path size: %ld", path.size());
    RCLCPP_INFO(node_ptr_->get_logger(), "Reduced path size: %ld", simplified_path.size());

    // Visualize path
    auto path_marker = get_path_marker(
      last_occ_grid_.header.frame_id, node_ptr_->get_clock()->now(), simplified_path,
      last_occ_grid_.info, last_occ_grid_.header);
    RCLCPP_INFO(node_ptr_->get_logger(), "Publishing path");
    viz_pub_->publish(path_marker);

    // TODO(pariasp): split path generator from visualization
    path_ = path_marker.points;

    return true;
  }

  n_times_executed_++;

  if (enable_path_optimizer_) {
    // TODO(pariaspe): Implement path optimizer
    RCLCPP_WARN(node_ptr_->get_logger(), "Path optimizer not implemented yet");
    // path = path_optimizer::solve(path);
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Path size: %ld", path.size());
  RCLCPP_INFO(node_ptr_->get_logger(), "Reduced path size: %ld", path.size());

  // Visualize path
  auto path_marker = get_path_marker(
    last_occ_grid_.header.frame_id, node_ptr_->get_clock()->now(), path,
    last_occ_grid_.info, last_occ_grid_.header);
  RCLCPP_INFO(node_ptr_->get_logger(), "Publishing path");
  viz_pub_->publish(path_marker);

  // TODO(pariasp): split path generator from visualization
  path_ = path_marker.points;

  return true;
}

bool Plugin::on_deactivate()
{
  return true;
}

bool Plugin::on_modify()
{
  return true;
}

bool Plugin::on_pause()
{
  return true;
}

bool Plugin::on_resume()
{
  return true;
}

void Plugin::on_execution_end()
{
}

as2_behavior::ExecutionStatus Plugin::on_run()
{
  return as2_behavior::ExecutionStatus::SUCCESS;
}

bool Plugin::is_occupied(const geometry_msgs::msg::PointStamped & point)
{
  Point2i cell = utils::pointToCell(
    point, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
  if (!cell_in_map(cell)) {
    RCLCPP_WARN(
      node_ptr_->get_logger(), "Point [%.2f, %.2f] is outside the occupancy grid.",
      point.point.x, point.point.y);
    return true;
  }
  return a_star_searcher_.cell_occuppied(cell);
}

bool Plugin::is_unknown(const geometry_msgs::msg::PointStamped & point)
{
  Point2i cell = utils::pointToCell(
    point, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
  return is_unknown_cell(cell);
}

bool Plugin::is_path_traversable(const std::vector<geometry_msgs::msg::PointStamped> & path)
{
  for (const auto & p : path) {
    Point2i cell = utils::pointToCell(
      p, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
    if (!is_known_free_cell(cell)) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Path is not traversable. Cell (%d, %d) value=%d unknown=%s occupied_or_inflated=%s.",
        cell.x, cell.y, occupancy_value(cell), is_unknown_cell(cell) ? "true" : "false",
        is_occupied(p) ? "true" : "false");
      return false;
    }
  }
  return true;
}

bool Plugin::cell_in_map(const Point2i & cell) const
{
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < static_cast<int>(last_occ_grid_.info.width) &&
         cell.y < static_cast<int>(last_occ_grid_.info.height);
}

int Plugin::occupancy_value(const Point2i & cell) const
{
  if (!cell_in_map(cell)) {
    return 100;
  }
  const size_t index =
    static_cast<size_t>(cell.y) * last_occ_grid_.info.width + static_cast<size_t>(cell.x);
  return static_cast<int>(last_occ_grid_.data[index]);
}

bool Plugin::is_unknown_cell(const Point2i & cell) const
{
  return !cell_in_map(cell) || occupancy_value(cell) < 0;
}

bool Plugin::is_known_free_cell(const Point2i & cell)
{
  if (!cell_in_map(cell) || is_unknown_cell(cell)) {
    return false;
  }
  if (occupancy_value(cell) > 30) {
    return false;
  }
  return !a_star_searcher_.cell_occuppied(cell);
}

std::vector<geometry_msgs::msg::PointStamped> Plugin::bresenham_line(
  const geometry_msgs::msg::PointStamped & start,
  const geometry_msgs::msg::PointStamped & end)
{
  std::vector<geometry_msgs::msg::PointStamped> line_points;

  Point2i start_cell = utils::pointToCell(
    start, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
  Point2i end_cell = utils::pointToCell(
    end, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
  int x1 = static_cast<int>(std::round(start_cell.x));
  int y1 = static_cast<int>(std::round(start_cell.y));
  int x2 = static_cast<int>(std::round(end_cell.x));
  int y2 = static_cast<int>(std::round(end_cell.y));

  int dx = std::abs(x2 - x1);
  int dy = std::abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    double x = static_cast<double>(x1);
    double y = static_cast<double>(y1);
    geometry_msgs::msg::PointStamped point = utils::cellToPoint(
      Point2i(x1, y1), last_occ_grid_.info, last_occ_grid_.header);
    line_points.push_back(point);
    if (x1 == x2 && y1 == y2) {
      break;
    }
    int err2 = 2 * err;
    if (err2 > -dy) {
      err -= dy;
      x1 += sx;
    }
    if (err2 < dx) {
      err += dx;
      y1 += sy;
    }
  }

  return line_points;
}

geometry_msgs::msg::PointStamped Plugin::closest_free_point(
  const geometry_msgs::msg::PointStamped & start,
  const geometry_msgs::msg::PointStamped & goal)
{
  Point2i start_cell = utils::pointToCell(
    start, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);
  Point2i goal_cell = utils::pointToCell(
    goal, last_occ_grid_.info, last_occ_grid_.header.frame_id, tf_buffer_);

  if (!cell_in_map(start_cell)) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Start cell (%d, %d) is outside the occupancy grid.",
      start_cell.x, start_cell.y);
    return start;
  }

  const int width = static_cast<int>(last_occ_grid_.info.width);
  const int height = static_cast<int>(last_occ_grid_.info.height);
  std::vector<bool> visited(static_cast<size_t>(width * height), false);
  std::deque<Point2i> queue;

  auto index = [width](const Point2i & cell) {
      return static_cast<size_t>(cell.y * width + cell.x);
    };

  queue.push_back(start_cell);
  visited[index(start_cell)] = true;

  Point2i best_cell = start_cell;
  double best_distance = std::numeric_limits<double>::max();
  bool found_reachable_free_cell = false;

  const std::vector<Point2i> neighbors = {
    Point2i(1, 0), Point2i(-1, 0), Point2i(0, 1), Point2i(0, -1),
    Point2i(1, 1), Point2i(1, -1), Point2i(-1, 1), Point2i(-1, -1)
  };

  while (!queue.empty()) {
    Point2i current = queue.front();
    queue.pop_front();

    if (is_known_free_cell(current)) {
      const double dx = static_cast<double>(current.x - goal_cell.x);
      const double dy = static_cast<double>(current.y - goal_cell.y);
      const double distance = dx * dx + dy * dy;
      if (distance < best_distance) {
        best_distance = distance;
        best_cell = current;
        found_reachable_free_cell = true;
      }
    }

    for (const auto & offset : neighbors) {
      Point2i next(current.x + offset.x, current.y + offset.y);
      if (!cell_in_map(next) || visited[index(next)] || !is_known_free_cell(next)) {
        continue;
      }
      visited[index(next)] = true;
      queue.push_back(next);
    }
  }

  if (found_reachable_free_cell) {
    geometry_msgs::msg::PointStamped best_point;
    best_point.header = last_occ_grid_.header;
    best_point.point.x =
      best_cell.x * last_occ_grid_.info.resolution +
      last_occ_grid_.info.origin.position.x +
      last_occ_grid_.info.resolution / 2.0;
    best_point.point.y =
      best_cell.y * last_occ_grid_.info.resolution +
      last_occ_grid_.info.origin.position.y +
      last_occ_grid_.info.resolution / 2.0;
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "Found reachable known-free point closest to goal at [%.2f, %.2f], cell (%d, %d), "
      "goal cell (%d, %d), squared cell distance %.2f.",
      best_point.point.x, best_point.point.y, best_cell.x, best_cell.y, goal_cell.x, goal_cell.y,
      best_distance);
    return best_point;
  }

  RCLCPP_ERROR(node_ptr_->get_logger(), "No reachable known-free point found.");
  return start;
}

visualization_msgs::msg::Marker Plugin::get_path_marker(
  std::string frame_id, rclcpp::Time stamp,
  std::vector<Point2i> path, nav_msgs::msg::MapMetaData map_info,
  std_msgs::msg::Header map_header)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "a_star";
  marker.id = 33;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.lifetime = rclcpp::Duration::from_seconds(0);  // Lifetime forever

  for (auto & p : path) {
    auto point = utils::cellToPoint(p, map_info, map_header);
    marker.points.push_back(point.point);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    marker.colors.push_back(color);
  }
  return marker;
}

}  // namespace a_star

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(a_star::Plugin, as2_behaviors_path_planning::PluginBase)
