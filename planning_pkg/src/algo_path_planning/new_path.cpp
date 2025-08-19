#include "planning_pkg/linear_path.hpp"

#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace planning_pkg
{

PathGenerator::PathGenerator(std::string default_frame_id,
                                         double default_step_size)
: default_frame_id_(std::move(default_frame_id)),
  default_step_size_(default_step_size)
{}

nav_msgs::msg::Path PathGenerator::generate(
  const std::vector<std::pair<double,double>>& waypoints,
  const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
  const geometry_msgs::msg::Polygon& arena) const
{
  nav_msgs::msg::Path path;
  
  // Use default values
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = "map";

  if (waypoints.size() < 2) {
    return path; // vuoto
  }
  

  //create your path
  //input dato:
  // - waypoints : che contine inizio e fine del path
  // - obstacles: ostacoli da evitare
  // - arena: il bordo dell'arena

  return path;
}

void PathGenerator::set_default_frame_id(const std::string& fid)
{
  default_frame_id_ = fid;
}

void PathGenerator::set_default_step_size(double s)
{
  default_step_size_ = s;
}

const std::string& PathGenerator::default_frame_id() const
{
  return default_frame_id_;
}

double PathGenerator::default_step_size() const
{
  return default_step_size_;
}

} // namespace planning_pkg
