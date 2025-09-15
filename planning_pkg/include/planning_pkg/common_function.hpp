#ifndef PLANNING_PKG_COMMON_FUNCTION_HPP
#define PLANNING_PKG_COMMON_FUNCTION_HPP

#include <vector>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <obstacles_msgs/msg/obstacle_array_msg.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace planning_pkg
{
    class CommonFunction
    {
    public:
        static void compute_bbox(const geometry_msgs::msg::Polygon &poly,
                                 double &minx, double &miny,
                                 double &maxx, double &maxy);

        static bool point_in_polygon(const geometry_msgs::msg::Polygon &poly, double x, double y);

        static double point_to_segment_distance(double px, double py,
                                                double ax, double ay,
                                                double bx, double by);

        static double distance_to_polygon_edges(const geometry_msgs::msg::Polygon &poly,
                                                double x, double y);

        static double dist2(const geometry_msgs::msg::Point &a,
                            const geometry_msgs::msg::Point &b);

        static bool point_is_valid(double x, double y,
                                   const geometry_msgs::msg::Polygon &arena,
                                   const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                                   double clearance);

        static bool segment_is_valid(const geometry_msgs::msg::Point &a,
                                     const geometry_msgs::msg::Point &b,
                                     const geometry_msgs::msg::Polygon &arena,
                                     const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                                     double clearance,
                                     double sample_step);

        static std::vector<double> line_segment_intersection(double y, double x1, double y1, double x2, double y2);

        static std::vector<double> line_circle_intersection(double y, double center_x, double center_y, double radius);

        static std::vector<geometry_msgs::msg::Point> optimize_path_with_raycasting(
            const std::vector<geometry_msgs::msg::Point> &original_path,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena,
            double clearance,
            double sample_step);

        static std::vector<int> dijkstra_shortest_path_temp(
            int start, int goal,
            const std::vector<geometry_msgs::msg::Point> &points,
            const std::vector<std::vector<int>> &adjacency);

        static std::vector<int> astar_shortest_path_temp(
            int start, int goal,
            const std::vector<geometry_msgs::msg::Point> &points,
            const std::vector<std::vector<int>> &adjacency);

        static double yaw_from_quat_(const geometry_msgs::msg::Quaternion &qmsg);

        // Performance testing structures
        struct PathFindingResult {
            std::vector<int> path;
            double execution_time_ms;
            int nodes_explored;
            double path_cost;
            bool success;
        };

        static PathFindingResult test_dijkstra_performance(
            int start, int goal,
            const std::vector<geometry_msgs::msg::Point> &points,
            const std::vector<std::vector<int>> &adjacency);

        static PathFindingResult test_astar_performance(
            int start, int goal,
            const std::vector<geometry_msgs::msg::Point> &points,
            const std::vector<std::vector<int>> &adjacency);

    private:
        static inline double sqr(double v) { return v * v; }
    };
}

#endif // PLANNING_PKG_COMMON_FUNCTION_HPP
