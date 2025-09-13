#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav_msgs/msg/path.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planning_pkg/common.hpp"
#include "planning_pkg/dubins_curve.hpp"

using namespace std::chrono_literals;

std::tuple<std::vector<KDPoint>, double, std::vector<std::vector<double>>>
get_dubins_best_path_and_cost(std::vector<double> q_near,
                              std::vector<double> q_rand, double _radius,
                              double step)
{
    DubinsPath dubins_path(q_near, q_rand, _radius);
    auto paths = dubins_path.calc_paths();
    auto shortest_path_cost = dubins_path.get_shortest_path_cost();
    Path full_path = gen_path(q_near, std::get<0>(shortest_path_cost), _radius, step);

    std::vector<KDPoint> path_points;
    for (size_t i = 0; i < std::get<0>(full_path).size(); i++)
    {
        path_points.push_back({std::get<0>(full_path)[i], std::get<1>(full_path)[i]});
    }

    return std::make_tuple(path_points, std::get<1>(shortest_path_cost), std::get<0>(shortest_path_cost));
}

std::vector<KDPoint> dubinise_path(const std::vector<KDPoint> &path, double radius, double step)
{
    std::vector<KDPoint> dubins_path;
    
    if (path.size() < 2) {
        return dubins_path;
    }
    
    auto compute_yaw = [](const KDPoint &p1, const KDPoint &p2) {
        return atan2(p2[1] - p1[1], p2[0] - p1[0]);
    };

    for (size_t i = 0; i < path.size() - 1; i++)
    {
        std::vector<double> p1 = {path[i][0], path[i][1]};
        std::vector<double> p2 = {path[i + 1][0], path[i + 1][1]};
        
        // Add yaw angles
        if (i == path.size() - 2) {
            // Last segment - use final orientation if available, otherwise compute
            p1.push_back(compute_yaw(path[i], path[i + 1]));
            if (path[i + 1].size() > 2) {
                p2.push_back(path[i + 1][2]);
            } else {
                p2.push_back(compute_yaw(path[i], path[i + 1]));
            }
        } else {
            p1.push_back(compute_yaw(path[i], path[i + 1]));
            p2.push_back(compute_yaw(path[i + 1], path[i + 2]));
        }
        
        auto best_path_and_cost = get_dubins_best_path_and_cost(p1, p2, radius, step);
        auto segment_path = std::get<0>(best_path_and_cost);
        
        if (i == 0) {
            dubins_path.insert(dubins_path.end(), segment_path.begin(), segment_path.end());
        } else {
            // Skip first point to avoid duplication
            dubins_path.insert(dubins_path.end(), segment_path.begin() + 1, segment_path.end());
        }
    }
    
    return dubins_path;
}

class DubinsCurvePublisher : public rclcpp::Node
{
public:
    DubinsCurvePublisher() : Node("DubinsCurvePublisher"), dubins_radius_(0.5), dubins_step_(0.1), min_distance_threshold_(1.0)
    {
        init_subscribers_and_publishers();
        init_services_and_timer();
        RCLCPP_INFO(get_logger(), "DubinsCurvePublisher initialized");
    }

private:
    void init_subscribers_and_publishers()
    {
        const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);
        const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_publishers);

        // Subscribers
        sub_path_pos1_ = create_subscription<nav_msgs::msg::Path>(
            "/path_pos1_to_gates", qos_sub, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) { process_path_pos1(msg); });
            
        sub_path_pos2_ = create_subscription<nav_msgs::msg::Path>(
            "/path_pos2_to_gates", qos_sub, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) { process_path_pos2(msg); });
            
        sub_pos1_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/published_pos1", qos_sub, 
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { 
                last_pos1_ = *msg; got_pos1_ = true; });
                
        sub_pos2_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/published_pos2", qos_sub, 
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { 
                last_pos2_ = *msg; got_pos2_ = true; });
                
        sub_gates_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/published_gates", qos_sub, 
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) { 
                last_gates_ = *msg; got_gates_ = true; });
                
        sub_obstacles_ = create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/inflated_obstacles", qos_sub, 
            [this](const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) { 
                last_obstacles_ = *msg; got_obstacles_ = true;
                RCLCPP_INFO(get_logger(), "Received obstacles: %zu obstacles", msg->obstacles.size()); });
                
        sub_arena_ = create_subscription<geometry_msgs::msg::Polygon>(
            "/inflated_arena", qos_sub, 
            [this](const geometry_msgs::msg::Polygon::SharedPtr msg) { 
                last_arena_ = *msg; got_arena_ = true;
                RCLCPP_INFO(get_logger(), "Received arena: %zu points", msg->points.size()); });

        // Publishers
        pub_dubins_path_pos1_ = create_publisher<nav_msgs::msg::Path>("/dubins_path_pos1", qos_pub);
        pub_dubins_path_pos2_ = create_publisher<nav_msgs::msg::Path>("/dubins_path_pos2", qos_pub);
    }

    void init_services_and_timer()
    {
        client_re_mapping_trigger = create_client<std_srvs::srv::Trigger>("/service_trigger_inflated");
        
        srv_trigger = create_service<std_srvs::srv::Trigger>(
            "/service_trigger_dubins_path",   
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                return trigger_paths_publication(request, response);
            });
            
        init_timer_ = create_wall_timer(3s, [this]() { init_service_call(); });
    }

    void trigger_paths_publication(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        bool success = false;
        
        if (!latest_dubins_path_1_.poses.empty()) {
            pub_dubins_path_pos1_->publish(latest_dubins_path_1_);
            RCLCPP_INFO(get_logger(), "Published /dubins_path_pos1 via trigger with %zu poses.", 
                       latest_dubins_path_1_.poses.size());
            success = true;
        } else {
            RCLCPP_WARN(get_logger(), "No path_1 available to publish via trigger.");
        }

        if (!latest_dubins_path_2_.poses.empty()) {
            pub_dubins_path_pos2_->publish(latest_dubins_path_2_);
            RCLCPP_INFO(get_logger(), "Published /dubins_path_pos2 via trigger with %zu poses.", 
                       latest_dubins_path_2_.poses.size());
            success = true;
        } else {
            RCLCPP_WARN(get_logger(), "No path_2 available to publish via trigger.");
        }

        response->success = success;
        response->message = success ? "Paths published successfully." : "No paths available.";
        RCLCPP_INFO(get_logger(), "Trigger service called and paths published.");
    }

    void init_service_call()
    {
        init_timer_->cancel();
        send_trigger_once();
    }

    void send_trigger_once()
    {
        for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
            if (client_re_mapping_trigger->wait_for_service(1s))
                break;
            RCLCPP_WARN(get_logger(), "Waiting for /service_trigger_inflated service...");
        }
        
        if (!client_re_mapping_trigger->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "Service /service_trigger_inflated not available.");
            return;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        client_re_mapping_trigger->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    auto resp = future.get();
                    if (resp->success) {
                        RCLCPP_INFO(get_logger(), "Trigger OK: %s", resp->message.c_str());
                    } else {
                        RCLCPP_WARN(get_logger(), "Trigger FAIL: %s", resp->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
                }
            });
    }

    double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &qmsg) const
    {
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
    }

    geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) const
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::Quaternion qmsg;
        tf2::convert(q, qmsg);
        return qmsg;
    }

    double calculate_distance(const geometry_msgs::msg::Point &p1, 
                            const geometry_msgs::msg::Point &p2) const
    {
        const double dx = p1.x - p2.x;
        const double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double point_to_line_distance(double px, double py, double x1, double y1,
                                 double x2, double y2) const
    {
        const double A = px - x1;
        const double B = py - y1;
        const double C = x2 - x1;
        const double D = y2 - y1;
        const double dot = A * C + B * D;
        const double len_sq = C * C + D * D;

        if (len_sq < 1e-6)
            return std::sqrt(A * A + B * B);

        const double param = dot / len_sq;
        double xx, yy;
        
        if (param < 0) {
            xx = x1; yy = y1;
        } else if (param > 1) {
            xx = x2; yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        const double dx = px - xx;
        const double dy = py - yy;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool is_point_in_obstacle(double x, double y,
                             const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                             double safety_margin) const
    {
        for (const auto &obstacle : obstacles.obstacles) {
            // Check circular obstacles
            if (obstacle.polygon.points.size() == 1 && obstacle.radius > 0.0) {
                const auto &center = obstacle.polygon.points[0];
                const double dx = x - center.x;
                const double dy = y - center.y;
                const double dist = std::sqrt(dx * dx + dy * dy);

                if (dist < obstacle.radius + safety_margin)
                    return true;
            }
            
            // Check polygon obstacles
            bool inside = false;
            for (size_t j = 0, k = obstacle.polygon.points.size() - 1; j < obstacle.polygon.points.size(); k = j++) {
                const auto &pi = obstacle.polygon.points[j];
                const auto &pj = obstacle.polygon.points[k];

                if (((pi.y > y) != (pj.y > y)) &&
                    (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x)) {
                    inside = !inside;
                }
            }

            if (inside) return true;

            // Check distance to polygon edges
            for (size_t i = 0; i < obstacle.polygon.points.size(); ++i) {
                const auto &p1 = obstacle.polygon.points[i];
                const auto &p2 = obstacle.polygon.points[(i + 1) % obstacle.polygon.points.size()];

                if (point_to_line_distance(x, y, p1.x, p1.y, p2.x, p2.y) < safety_margin)
                    return true;
            }
        }
        return false;
    }

    bool is_point_in_arena(double x, double y, const geometry_msgs::msg::Polygon &arena) const
    {
        if (arena.points.empty())
            return true;

        bool inside = false;
        for (size_t i = 0, j = arena.points.size() - 1; i < arena.points.size(); j = i++) {
            const auto &pi = arena.points[i];
            const auto &pj = arena.points[j];

            if (((pi.y > y) != (pj.y > y)) &&
                (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x)) {
                inside = !inside;
            }
        }
        return inside;
    }

    std::vector<geometry_msgs::msg::PoseStamped> filter_path_points(
        const nav_msgs::msg::Path &original_path,
        const geometry_msgs::msg::Point &robot_pos) const
    {
        std::vector<geometry_msgs::msg::PoseStamped> filtered_points;
        
        for (const auto &pose : original_path.poses) {
            if (calculate_distance(pose.pose.position, robot_pos) >= min_distance_threshold_) {
                filtered_points.push_back(pose);
            }
        }
        return filtered_points;
    }

    nav_msgs::msg::Path generate_dubins_path(
        const geometry_msgs::msg::Pose &robot_pose,
        const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
        const std::string &frame_id)
    {
        nav_msgs::msg::Path dubins_path;
        dubins_path.header.frame_id = frame_id;
        dubins_path.header.stamp = get_clock()->now();

        if (waypoints.empty()) {
            RCLCPP_WARN(get_logger(), "No waypoints to generate Dubins path");
            return dubins_path;
        }

        std::vector<KDPoint> kd_waypoints;
        const double robot_yaw = yaw_from_quaternion(robot_pose.orientation);
        kd_waypoints.push_back({robot_pose.position.x, robot_pose.position.y, robot_yaw});

        // Add waypoints with computed orientations
        for (size_t i = 0; i < waypoints.size(); ++i) {
            double waypoint_yaw;
            if (i == waypoints.size() - 1) {
                waypoint_yaw = yaw_from_quaternion(waypoints[i].pose.orientation);
            } else {
                const auto &current = waypoints[i].pose.position;
                const auto &next = waypoints[i + 1].pose.position;
                waypoint_yaw = atan2(next.y - current.y, next.x - current.x);
            }

            kd_waypoints.push_back({waypoints[i].pose.position.x,
                                    waypoints[i].pose.position.y,
                                    waypoint_yaw});
        }

        // Generate Dubins path
        std::vector<KDPoint> dubins_points;
        if (got_obstacles_ && got_arena_) {
            dubins_points = generate_safe_dubins_path(kd_waypoints);
        } else {
            RCLCPP_WARN(get_logger(), "Obstacle or arena data not available, generating unconstrained path");
            dubins_points = dubinise_path(kd_waypoints, dubins_radius_, dubins_step_);
        }

        // Convert to ROS path
        for (const auto &point : dubins_points) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = frame_id;
            pose_stamped.header.stamp = dubins_path.header.stamp;
            pose_stamped.pose.position.x = point[0];
            pose_stamped.pose.position.y = point[1];
            pose_stamped.pose.position.z = 0.0;

            // Compute orientation for this point
            double yaw = 0.0;
            if (dubins_points.size() > 1) {
                auto it = std::find(dubins_points.begin(), dubins_points.end(), point);
                if (it != dubins_points.end() && std::next(it) != dubins_points.end()) {
                    auto next_point = *std::next(it);
                    yaw = atan2(next_point[1] - point[1], next_point[0] - point[0]);
                }
            }

            pose_stamped.pose.orientation = quaternion_from_yaw(yaw);
            dubins_path.poses.push_back(pose_stamped);
        }

        return dubins_path;
    }

    bool is_path_segment_valid(const KDPoint &start, const KDPoint &end) const
    {
        // Check if both points are in arena
        if (!is_point_in_arena(start[0], start[1], last_arena_) ||
            !is_point_in_arena(end[0], end[1], last_arena_))
            return false;

        // Check collision along the segment
        const double dx = end[0] - start[0];
        const double dy = end[1] - start[1];
        const double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < 1e-6) return true;
        
        const int num_checks = static_cast<int>(distance / (dubins_step_ * 0.5)) + 1;
        
        for (int i = 0; i <= num_checks; ++i) {
            const double t = static_cast<double>(i) / num_checks;
            const double x = start[0] + t * dx;
            const double y = start[1] + t * dy;
            
            if (is_point_in_obstacle(x, y, last_obstacles_, 0.1))
                return false;
        }
        
        return true;
    }

    std::vector<KDPoint> generate_safe_dubins_path(const std::vector<KDPoint> &waypoints)
    {
        std::vector<KDPoint> safe_path;
        if (waypoints.size() < 2) return safe_path;

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            const KDPoint start = waypoints[i];
            const KDPoint end = waypoints[i + 1];

            // Try different turning radii if the original fails
            const std::vector<double> radii_to_try = {
                dubins_radius_, dubins_radius_ * 0.8, dubins_radius_ * 1.2, 
                dubins_radius_ * 0.6, dubins_radius_ * 1.5
            };

            bool path_found = false;
            for (double radius : radii_to_try) {
                auto segment_path = generate_safe_dubins_segment(start, end, radius);
                if (segment_path.empty()) continue;

                bool segment_valid = true;
                for (size_t j = 0; j < segment_path.size() - 1; ++j) {
                    if (!is_path_segment_valid(segment_path[j], segment_path[j + 1])) {
                        segment_valid = false;
                        break;
                    }
                }

                if (segment_valid) {
                    append_segment_to_path(safe_path, segment_path);
                    path_found = true;
                    break;
                }
            }

            if (!path_found) {
                RCLCPP_WARN(get_logger(), "Could not find collision-free Dubins path for segment %zu", i);
                auto fallback_segment = generate_fallback_segment(start, end);
                append_segment_to_path(safe_path, fallback_segment);
            }
        }

        return safe_path;
    }

    void append_segment_to_path(std::vector<KDPoint> &path, const std::vector<KDPoint> &segment)
    {
        if (path.empty()) {
            path.insert(path.end(), segment.begin(), segment.end());
        } else {
            path.insert(path.end(), segment.begin() + 1, segment.end());
        }
    }

    std::vector<KDPoint> generate_safe_dubins_segment(const KDPoint &start, const KDPoint &end, double radius)
    {
        const std::vector<double> start_vec = {start[0], start[1], start[2]};
        const std::vector<double> end_vec = {end[0], end[1], end[2]};
        auto best_path_and_cost = get_dubins_best_path_and_cost(start_vec, end_vec, radius, dubins_step_);
        return std::get<0>(best_path_and_cost);
    }

    std::vector<KDPoint> generate_fallback_segment(const KDPoint &start, const KDPoint &end)
    {
        std::vector<KDPoint> segment;
        const double dx = end[0] - start[0];
        const double dy = end[1] - start[1];
        const double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 1e-6) {
            segment.push_back(start);
            return segment;
        }

        const int num_points = static_cast<int>(distance / dubins_step_) + 1;

        for (int i = 0; i <= num_points; ++i) {
            const double t = static_cast<double>(i) / num_points;
            KDPoint point = {start[0] + t * dx, start[1] + t * dy};
            segment.push_back(point);
        }

        return segment;
    }

    void process_path_pos1(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!got_pos1_) {
            RCLCPP_WARN(get_logger(), "Robot 1 position not available yet");
            return;
        }

        RCLCPP_INFO(get_logger(), "Processing Dubins path for Robot 1");
        auto filtered_points = filter_path_points(*msg, last_pos1_.pose.pose.position);
        latest_dubins_path_1_ = generate_dubins_path(
            last_pos1_.pose.pose, filtered_points, msg->header.frame_id);
    }

    void process_path_pos2(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!got_pos2_) {
            RCLCPP_WARN(get_logger(), "Robot 2 position not available yet");
            return;
        }

        RCLCPP_INFO(get_logger(), "Processing Dubins path for Robot 2");
        auto filtered_points = filter_path_points(*msg, last_pos2_.pose.pose.position);
        latest_dubins_path_2_ = generate_dubins_path(
            last_pos2_.pose.pose, filtered_points, msg->header.frame_id);
    }

    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_pos1_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_pos2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_arena_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dubins_path_pos1_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dubins_path_pos2_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_re_mapping_trigger;
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Data storage
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos1_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos2_;
    nav_msgs::msg::Path latest_dubins_path_1_;
    nav_msgs::msg::Path latest_dubins_path_2_;
    geometry_msgs::msg::PoseArray last_gates_;
    obstacles_msgs::msg::ObstacleArrayMsg last_obstacles_;
    geometry_msgs::msg::Polygon last_arena_;

    // State flags
    bool got_pos1_{false};
    bool got_pos2_{false};
    bool got_gates_{false};
    bool got_obstacles_{false};
    bool got_arena_{false};

    // Configuration parameters
    const double dubins_radius_;
    const double dubins_step_;
    const double min_distance_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DubinsCurvePublisher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    RCLCPP_INFO(node->get_logger(), "Starting DubinsCurvePublisher with MultiThreadedExecutor");
    exec.spin();
    rclcpp::shutdown();
    return 0;
}