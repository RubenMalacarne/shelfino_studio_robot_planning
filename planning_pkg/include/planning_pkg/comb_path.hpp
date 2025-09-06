#pragma once
#include <string>
#include <vector>
#include <utility>
#include <random>
#include <cstddef> // std::size_t
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
    // Struttura per rappresentare una linea retta
    struct HorizontalLine
    {
        double y;       // coordinata y della linea
        double x_start; // punto di inizio x
        double x_end;   // punto di fine x

        HorizontalLine(double y_coord, double x_s, double x_e)
            : y(y_coord), x_start(x_s), x_end(x_e) {}
    };

    // Struttura per rappresentare una cella rettangolare
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
        // Ctor principale
        CombPathGenerator(std::string default_frame_id, double default_step_size);
        // Ctor di default (valori comodi)
        CombPathGenerator() : CombPathGenerator("default_frame", 0.1) {}

        // Genera un Path
        nav_msgs::msg::Path generate(
            const std::vector<std::pair<double, double>> &waypoints,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);

        // Setter / getter parametri di default
        void set_default_frame_id(const std::string &fid);
        void set_default_step_size(double s);
        const std::string &default_frame_id();
        double default_step_size();

        // Metodi richiesti
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

        std::vector<int> dijkstra_shortest_path_temp(
        int start, int goal,
        const std::vector<geometry_msgs::msg::Point> &points,
        const std::vector<std::vector<int>> &adjacency) const;

    private:
        std::string default_frame_id_;
        double default_step_size_;

        // Metodi di utilità privati
        bool point_in_polygon(const geometry_msgs::msg::Polygon &poly, double x, double y) const;
        std::vector<std::pair<double, double>> get_intersection_points(
            double y,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;
    };
} // namespace planning_pkg