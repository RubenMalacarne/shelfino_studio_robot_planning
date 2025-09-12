#ifndef PLANNING_PKG__DUBINS_CURVE_HPP_
#define PLANNING_PKG__DUBINS_CURVE_HPP_

#include <vector>
#include <tuple>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

using KDPoint = std::vector<double>;
using Path = std::tuple<std::vector<double>, std::vector<double>>;

class DubinsPath {
public:
    DubinsPath(const std::vector<double>& start, const std::vector<double>& end, double r);
    
    std::vector<std::vector<std::vector<double>>> calc_paths();
    std::tuple<std::vector<std::vector<double>>, double> get_shortest_path_cost();
    
private:
    std::vector<std::vector<double>> _s;
    std::vector<std::vector<double>> _e;
    double _r;
    std::vector<std::vector<std::vector<double>>> _paths;
    
    std::vector<double> calc_end();
    double mod2pi(double theta);
    
    std::vector<std::vector<double>> calc_lsl_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_rsr_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_lsr_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_rsl_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_rlr_from_origin(std::vector<double> e);
    std::vector<std::vector<double>> calc_lrl_from_origin(std::vector<double> e);
};

std::tuple<std::vector<double>, std::vector<double>> gen_path(
    const std::vector<double> &s, const std::vector<std::vector<double>> &path,
    double r = 1.0, double step = 0.1);

std::tuple<std::vector<KDPoint>, double, std::vector<std::vector<double>>>
get_dubins_best_path_and_cost(std::vector<double> q_near,
                              std::vector<double> q_rand, double _radius,
                              double step);

double point_to_line_distance(double px, double py, double x1, double y1, 
                             double x2, double y2);

bool is_point_in_obstacle(double x, double y, 
                         const obstacles_msgs::msg::ObstacleArrayMsg &obstacles, 
                         double safety_margin = 0.1);

bool is_path_collision_free(const std::vector<KDPoint> &path, 
                           const obstacles_msgs::msg::ObstacleArrayMsg &obstacles, 
                           double safety_margin = 0.1);

std::vector<KDPoint> dubinise_path(const std::vector<KDPoint> &path, 
                                  double radius, double step);

#endif  // PLANNING_PKG__DUBINS_CURVE_HPP_