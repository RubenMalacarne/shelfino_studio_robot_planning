#pragma once

#include <vector>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"

namespace planning_pkg
{

  class LinearPathGenerator
  {
  public:
    explicit LinearPathGenerator(std::string default_frame_id = "map",
                                 double default_step_size = 0.1);

    nav_msgs::msg::Path generate(
        const std::vector<std::pair<double, double>> &waypoints,
        const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
        const geometry_msgs::msg::Polygon &arena);

    void set_default_frame_id(const std::string &fid);
    void set_default_step_size(double s);
    const std::string &default_frame_id() const;
    double default_step_size() const;

    bool is_direct_path_feasible(const std::pair<double, double> &start,
                                 const std::pair<double, double> &goal,
                                 const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                                 const geometry_msgs::msg::Polygon &arena,
                                 double clearance = 0.15) const;

  private:
    std::string default_frame_id_;
    double default_step_size_;
  };

} // namespace planning_pkg
