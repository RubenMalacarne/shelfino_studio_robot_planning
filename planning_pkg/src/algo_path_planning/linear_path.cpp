#include "planning_pkg/linear_path.hpp"
#include "planning_pkg/common_function.hpp"
#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace planning_pkg
{
  LinearPathGenerator::LinearPathGenerator(std::string default_frame_id,
                                           double default_step_size)
      : default_frame_id_(std::move(default_frame_id)),
        default_step_size_(default_step_size)
  {
  }

  nav_msgs::msg::Path LinearPathGenerator::generate(
      const std::vector<std::pair<double, double>> &waypoints,
      const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
      const geometry_msgs::msg::Polygon &arena)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = default_frame_id_;

    if (waypoints.size() < 2)
    {
      return path; // niente da fare se meno di 2 punti
    }

    // Considera solo il primo e l'ultimo punto
    const auto [x0, y0] = waypoints.front();
    const auto [x1, y1] = waypoints.back();

    // Verifica se il percorso diretto è fattibile
    if (!is_direct_path_feasible({x0, y0}, {x1, y1}, obstacles, arena, 0.1))
    {
      RCLCPP_WARN(rclcpp::get_logger("linear_path_generator"),
                  "Percorso diretto non fattibile, ritorno path vuoto");
      return path;
    }

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double distance = std::hypot(dx, dy);

    // Numero di passi in base alla distanza e step size
    const std::size_t steps = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::floor(distance / default_step_size_)));

    const double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    for (std::size_t s = 0; s <= steps; ++s)
    {
      const double ratio = static_cast<double>(s) / static_cast<double>(steps);
      const double x = x0 + ratio * dx;
      const double y = y0 + ratio * dy;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = path.header.stamp;
      pose.header.frame_id = default_frame_id_;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf2::toMsg(q);

      path.poses.push_back(std::move(pose));
    }

    return path;
  }

  bool LinearPathGenerator::is_direct_path_feasible(
      const std::pair<double, double> &start,
      const std::pair<double, double> &goal,
      const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
      const geometry_msgs::msg::Polygon &arena,
      double clearance) const
  {
    // Converti i punti in geometry_msgs::msg::Point
    geometry_msgs::msg::Point start_point;
    start_point.x = start.first;
    start_point.y = start.second;
    start_point.z = 0.0;

    geometry_msgs::msg::Point goal_point;
    goal_point.x = goal.first;
    goal_point.y = goal.second;
    goal_point.z = 0.0;

    // Usa CommonFunction::segment_is_valid per verificare il segmento diretto
    const double sample_step = 0.05; // Passo di campionamento per la verifica

    bool is_valid = CommonFunction::segment_is_valid(start_point, goal_point,
                                                     arena, obstacles,
                                                     clearance, sample_step);

    if (is_valid)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("linear_path_generator"),
                   "Percorso diretto fattibile da (%.2f, %.2f) a (%.2f, %.2f)",
                   start.first, start.second, goal.first, goal.second);
    }
    else
    {
      RCLCPP_DEBUG(rclcpp::get_logger("linear_path_generator"),
                   "Percorso diretto bloccato da ostacoli o fuori arena");
    }

    return is_valid;
  }
  void LinearPathGenerator::set_default_frame_id(const std::string &fid)
  {
    default_frame_id_ = fid;
  }

  void LinearPathGenerator::set_default_step_size(double s)
  {
    default_step_size_ = s;
  }

  const std::string &LinearPathGenerator::default_frame_id() const
  {
    return default_frame_id_;
  }

  double LinearPathGenerator::default_step_size() const
  {
    return default_step_size_;
  }

} // namespace planning_pkg
