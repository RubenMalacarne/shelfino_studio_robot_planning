#pragma once
#include <string>
#include <vector>
#include <utility>
#include <random>
#include <cstddef> 
#include <map>
#include <cmath>
#include <optional>
#include <queue>
#include <functional>
#include <set>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include <queue>
#include <functional>

namespace planning_pkg
{
    struct HorizontalLine
    {
        double y;       
        double x_start; 
        double x_end;   

        HorizontalLine(double y_coord, double x_s, double x_e)
            : y(y_coord), x_start(x_s), x_end(x_e) {}
    };

    struct Cell
    {
        double x_min;
        double x_max;
        double y_min;
        double y_max;
        double center_x;
        double center_y;
        double width;
        double height;
    };

    class CombPathGenerator
    {
    public:
        
        CombPathGenerator(std::string default_frame_id, double default_step_size);
        CombPathGenerator() : CombPathGenerator("default_frame", 0.1) {}

        nav_msgs::msg::Path generate(
            const std::vector<std::pair<double, double>> &waypoints,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);

        void set_default_frame_id(const std::string &fid);
        void set_default_step_size(double s);
        const std::string &default_frame_id();
        double default_step_size();

        std::vector<std::pair<double, double>> get_pointlist(
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        std::vector<HorizontalLine> set_vertical_line(
            const std::vector<std::pair<double, double>> &points,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        std::vector<std::pair<double, double>> set_point_in_vertical_line(
            const HorizontalLine &horizontal_line, double offset) const;

        std::vector<HorizontalLine> get_horizontal_lines(
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);
        std::pair<double, double> get_cell_centroid(const Cell &cell);
        std::vector<Cell> get_cells_btw_vlines(
            const std::vector<HorizontalLine> &horizontal_lines) const;

        std::vector<std::vector<int>> get_arc(
            std::vector<HorizontalLine> horizontal_lines,
            std::vector<std::pair<double, double>> points,
            std::vector<std::pair<double, double>> points_centroids);


    private:
        std::string default_frame_id_;
        double default_step_size_;

        bool point_in_polygon(const geometry_msgs::msg::Polygon &poly, double x, double y) const;
        std::vector<std::pair<double, double>> get_intersection_points(
            double y,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;
    };
} // namespace planning_pkg