#include <memory>
#include <vector>
#include <cmath>
#include <algorithm> 
#include <limits>
#include <tuple>

#include "planning_pkg/dubins_curve.hpp"

DubinsPath::DubinsPath(const std::vector<double>& start, const std::vector<double>& end, double r)
    : _s({start}), _e({end}), _r(r) {}

std::tuple<std::vector<double>, std::vector<double>> gen_path(
    const std::vector<double> &s, const std::vector<std::vector<double>> &path,
    double r, double step) {
    
    auto calc_TurnCenter = [](const std::vector<double> &point, char dir,
                            double r) -> std::tuple<double, double> {
        double ang = point[2] + (dir == 'l' ? M_PI/2 : -M_PI/2);
        return {point[0] + std::cos(ang) * r, point[1] + std::sin(ang) * r};
    };

    std::vector<double> r_x, r_y, ps_x, ps_y;
    std::vector<double> start = s;
    double yaw = s[2], step_angle = step / r;

    for (const auto &p : path) {
        if (p[0] == 's') {
            for (double l = 0.0; l < p[1]; l += step) {
                ps_x.push_back(start[0] + std::cos(yaw) * l);
                ps_y.push_back(start[1] + std::sin(yaw) * l);
            }
            ps_x.push_back(start[0] + std::cos(yaw) * p[1]);
            ps_y.push_back(start[1] + std::sin(yaw) * p[1]);
        } else {
            auto center = calc_TurnCenter(start, static_cast<char>(p[0]), r);
            double ang_start = std::atan2(start[1] - std::get<1>(center),
                                        start[0] - std::get<0>(center));
            double ang_end = ang_start + p[1] * (p[0] == 'l' ? 1 : -1);
            for (double ang = ang_start;
                (p[0] == 'l' ? ang <= ang_end : ang >= ang_end);
                ang += (p[0] == 'l' ? step_angle : -step_angle)) {
                ps_x.push_back(std::get<0>(center) + std::cos(ang) * r);
                ps_y.push_back(std::get<1>(center) + std::sin(ang) * r);
            }
            ps_x.push_back(std::get<0>(center) + std::cos(ang_end) * r);
            ps_y.push_back(std::get<1>(center) + std::sin(ang_end) * r);
            yaw = start[2] + p[1] * (p[0] == 'l' ? 1 : -1);
        }
        r_x.insert(r_x.end(), ps_x.begin(), ps_x.end());
        r_y.insert(r_y.end(), ps_y.begin(), ps_y.end());
        start = {ps_x.back(), ps_y.back(), yaw};
        ps_x.clear(); ps_y.clear();
    }
    return {r_x, r_y};
}

std::vector<std::vector<std::vector<double>>> DubinsPath::calc_paths() {
    auto le = calc_end();
    std::vector<std::string> types = {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"};
    for (const auto& type : types) {
        auto path = calc_path_generic(le, type);
        if (!path.empty()) _paths.push_back(path);
    }
    return _paths;
}

std::tuple<std::vector<std::vector<double>>, double> DubinsPath::get_shortest_path_cost() {
    std::tuple<std::vector<std::vector<double>>, double> shortest_path_cost;
    std::get<1>(shortest_path_cost) = std::numeric_limits<double>::infinity();
    for (auto &path : _paths) {
        double cost = 0;
        for (auto &p : path) cost += (p[0] == 's') ? p[1] : p[1] * _r;
        if (cost < std::get<1>(shortest_path_cost)) {
            std::get<0>(shortest_path_cost) = path;
            std::get<1>(shortest_path_cost) = cost;
        }
    }
    return shortest_path_cost;
}

std::vector<double> DubinsPath::calc_end() {
    double ex = _e[0][0] - _s[0][0], ey = _e[0][1] - _s[0][1];
    double lex = (cos(_s[0][2]) * ex + sin(_s[0][2]) * ey) / _r;
    double ley = (-sin(_s[0][2]) * ex + cos(_s[0][2]) * ey) / _r;
    return {lex, ley, _e[0][2] - _s[0][2]};
}

double DubinsPath::mod2pi(double theta) {
    return theta - 2.0 * M_PI * floor(theta / (2.0 * M_PI));
}

std::vector<std::vector<double>> DubinsPath::calc_path_generic(std::vector<double> e, const std::string& type) {
    // Trasformazione simmetrica per tipi R**
    if (type[0] == 'R') {
        e[1] = -e[1];
        e[2] = mod2pi(-e[2]);
    }
    
    std::vector<std::vector<double>> path;
    
    if (type == "LSL" || type == "RSR") {
        double x_ = e[0] - sin(e[2]), y_ = e[1] - 1 + cos(e[2]);
        double u = sqrt(x_*x_ + y_*y_), t = mod2pi(atan2(y_, x_)), v = mod2pi(e[2] - t);
        path = {{type[0], t}, {'s', u * _r}, {type[2], v}};
    }
    else if (type == "LSR" || type == "RSL") {
        double x_ = e[0] + sin(e[2]), y_ = e[1] - 1 - cos(e[2]);
        double u1_square = x_*x_ + y_*y_;
        if (u1_square >= 4) {
            double t1 = mod2pi(atan2(y_, x_)), u = sqrt(u1_square - 4);
            double theta = mod2pi(atan(2 / u)), t = mod2pi(t1 + theta), v = mod2pi(t - e[2]);
            path = {{type[0], t}, {'s', u * _r}, {type[2], v}};
        }
    }
    else if (type == "LRL" || type == "RLR") {
        double x_ = e[0] - sin(e[2]), y_ = e[1] - 1 + cos(e[2]);
        double u1 = sqrt(x_*x_ + y_*y_);
        if (u1 <= 4) {
            double t1 = atan2(y_, x_), theta = acos(u1 / 4);
            double t = mod2pi(M_PI/2 + t1 + theta), u = mod2pi(M_PI + 2*theta);
            double v = mod2pi(M_PI/2 - t1 + theta + e[2]);
            path = {{type[0], t}, {type[1], u}, {type[2], v}};
        }
    }
    
    // Converti char a minuscolo per compatibilità
    for (auto& segment : path) {
        if (segment[0] != 's') segment[0] = tolower(segment[0]);
    }
    
    return path;
}

// Metodi legacy per compatibilità
std::vector<std::vector<double>> DubinsPath::calc_lsl_from_origin(std::vector<double> e) { return calc_path_generic(e, "LSL"); }
std::vector<std::vector<double>> DubinsPath::calc_rsr_from_origin(std::vector<double> e) { return calc_path_generic(e, "RSR"); }
std::vector<std::vector<double>> DubinsPath::calc_lsr_from_origin(std::vector<double> e) { return calc_path_generic(e, "LSR"); }
std::vector<std::vector<double>> DubinsPath::calc_rsl_from_origin(std::vector<double> e) { return calc_path_generic(e, "RSL"); }
std::vector<std::vector<double>> DubinsPath::calc_lrl_from_origin(std::vector<double> e) { return calc_path_generic(e, "LRL"); }
std::vector<std::vector<double>> DubinsPath::calc_rlr_from_origin(std::vector<double> e) { return calc_path_generic(e, "RLR"); }