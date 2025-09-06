#ifndef PLANNING_PKG_COMMON_FUNCTION_HPP
#define PLANNING_PKG_COMMON_FUNCTION_HPP

#include <vector>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <obstacles_msgs/msg/obstacle_array_msg.hpp>

namespace planning_pkg
{
    class CommonFunction
    {
    public:
        // Bounding box computation
        static void compute_bbox(const geometry_msgs::msg::Polygon &poly,
                                double &minx, double &miny,
                                double &maxx, double &maxy);

        // Point in polygon test
        static bool point_in_polygon(const geometry_msgs::msg::Polygon &poly, double x, double y);

        // Distance calculations
        static double point_to_segment_distance(double px, double py,
                                               double ax, double ay,
                                               double bx, double by);

        static double distance_to_polygon_edges(const geometry_msgs::msg::Polygon &poly,
                                               double x, double y);

        static double dist2(const geometry_msgs::msg::Point &a,
                           const geometry_msgs::msg::Point &b);

        // Validation functions
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

        // Intersection calculations
        static std::vector<double> line_segment_intersection(double y, double x1, double y1, double x2, double y2);

        static std::vector<double> line_circle_intersection(double y, double center_x, double center_y, double radius);

        // Path optimization
        static std::vector<geometry_msgs::msg::Point> optimize_path_with_raycasting(
            const std::vector<geometry_msgs::msg::Point> &original_path,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena,
            double clearance,
            double sample_step);

    private:
        // Utility functions
        static inline double sqr(double v) { return v * v; }

    };
}

#endif // PLANNING_PKG_COMMON_FUNCTION_HPP
