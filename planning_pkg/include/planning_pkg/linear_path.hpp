#pragma once

#include <vector>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

namespace planning_pkg
{

class LinearPathGenerator
{
public:
  explicit LinearPathGenerator(std::string default_frame_id = "map",
                               double default_step_size = 0.1);

  /**
   * @brief Genera un path lineare interpolando i waypoint con posa orientata lungo la direzione.
   * @param waypoints  vettore di (x,y)
   * @param stamp      timestamp da usare nelle poses e nell'header
   * @param frame_id   opzionale; se vuoto usa quello di default
   * @param step_size  opzionale; se < 0 usa quello di default
   */
  nav_msgs::msg::Path generate(
    const std::vector<std::pair<double,double>>& waypoints,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "",
    double step_size = -1.0) const;

  // Setter e getter opzionali
  void set_default_frame_id(const std::string& fid);
  void set_default_step_size(double s);
  const std::string& default_frame_id() const;
  double default_step_size() const;

private:
  std::string default_frame_id_;
  double      default_step_size_;
};

} // namespace planning_pkg
