// include/your_pkg/visualization_utils.hpp
#pragma once

#include <vector>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "planning_pkg/comb_path.hpp"

#include "planning_pkg/common.hpp"
namespace planning_pkg
{
class VisualizationUtils
{
public:
    VisualizationUtils(rclcpp::Node *node);


    void vis_points(const std::vector<std::pair<double, double>> &points);
    void vis_cells(const std::vector<planning_pkg::Cell> &cells);
    void vis_line();
    void vis_arcs(const std::vector<std::vector<int>> &arc_list,
                  const std::vector<std::pair<double, double>> &points_line,
                  const std::vector<std::pair<double, double>> &points_centroids);

private:
    rclcpp::Node *node_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_points_markers;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cells_markers;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vertical_line;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_arcs_markers;

    std::vector<std::tuple<int, double,
                           std::pair<double, double>,
                           std::pair<double, double>>> vertical_lines_data_;
};
} // namespace planning_pkg