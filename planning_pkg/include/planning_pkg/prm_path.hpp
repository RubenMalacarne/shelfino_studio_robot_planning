#pragma once

#include <string>
#include <vector>
#include <utility>
#include <random>
#include <cstddef>   

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"


#include <queue>
#include <functional>
namespace planning_pkg
{

class PrmPathGenerator
{
public:
  PrmPathGenerator(std::string default_frame_id, double default_step_size);

  PrmPathGenerator() : PrmPathGenerator("default_frame", 0.1) {}

  nav_msgs::msg::Path generate(
    const std::vector<std::pair<double, double>>& waypoints,
    const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
    const geometry_msgs::msg::Polygon& arena) const;

  void sample_random_points(
    const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
    const geometry_msgs::msg::Polygon& arena,
    std::size_t count);

  void build_knn_edges(const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
                       const geometry_msgs::msg::Polygon& arena,
                       std::size_t k_neighbors = 7,
                       double clearance = 0.15,
                       double sample_step = 0.1);

  
  std::vector<geometry_msgs::msg::Point> random_points;
  std::vector<std::vector<int>> knn_adj;
  void set_default_frame_id(const std::string& fid);
  void set_default_step_size(double s);

  const std::string& default_frame_id() const;
  double default_step_size() const;
  std::vector<std::pair<double,double>>
  indices_to_polyline(const std::vector<int>& idx_path) const;

  const std::vector<std::vector<int>>& knn_adjacency() const { return knn_adj; }
  
private:
  std::string default_frame_id_;
  double default_step_size_;

  std::vector<int> dijkstra_shortest_path(int start, int goal) const;

};

} // namespace planning_pkg
