#include "planning_pkg/linear_path.hpp"

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
{}

nav_msgs::msg::Path LinearPathGenerator::generate(
  const std::vector<std::pair<double,double>>& waypoints,
  const rclcpp::Time& stamp,
  const std::string& frame_id,
  double step_size) const
{
  nav_msgs::msg::Path path;
  const std::string fid = frame_id.empty() ? default_frame_id_ : frame_id;
  const double ds = (step_size > 0.0) ? step_size : default_step_size_;

  path.header.stamp = stamp;
  path.header.frame_id = fid;

  if (waypoints.size() < 2) {
    return path; // vuoto
  }

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto [x0, y0] = waypoints[i];
    const auto [x1, y1] = waypoints[i + 1];

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double distance = std::hypot(dx, dy);

    const std::size_t steps = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::floor(distance / ds)));

    const double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    for (std::size_t s = 0; s <= steps; ++s) {
      const double ratio = static_cast<double>(s) / static_cast<double>(steps);
      const double x = x0 + ratio * dx;
      const double y = y0 + ratio * dy;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = fid;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(std::move(pose));
    }
  }

  return path;
}

void LinearPathGenerator::set_default_frame_id(const std::string& fid)
{
  default_frame_id_ = fid;
}

void LinearPathGenerator::set_default_step_size(double s)
{
  default_step_size_ = s;
}

const std::string& LinearPathGenerator::default_frame_id() const
{
  return default_frame_id_;
}

double LinearPathGenerator::default_step_size() const
{
  return default_step_size_;
}

} // namespace planning_pkg
