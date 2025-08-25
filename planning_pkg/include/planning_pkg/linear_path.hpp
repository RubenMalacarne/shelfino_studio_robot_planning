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

  /**
   * @brief Genera un path lineare interpolando i waypoint con posa orientata lungo la direzione.
   * @param waypoints  vettore di (x,y)
   * @param obstacles  array di ostacoli da evitare
   * @param arena      poligono che definisce l'arena di gioco
   */
  nav_msgs::msg::Path generate(
    const std::vector<std::pair<double,double>>& waypoints,
    const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
    const geometry_msgs::msg::Polygon& arena) const;

  /**
   * @brief Genera un path lineare semplice interpolando i waypoint
   * @param waypoints  vettore di (x,y)
   * @param timestamp  timestamp per il path
   * @param frame_id   frame di riferimento
   * @param step_size  dimensione del passo per l'interpolazione
   */
  nav_msgs::msg::Path generate(
    const std::vector<std::pair<double,double>>& waypoints,
    const rclcpp::Time& timestamp,
    const std::string& frame_id,
    double step_size) const;

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
