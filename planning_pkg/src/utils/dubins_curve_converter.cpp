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

    std::tuple<std::vector<KDPoint>, double, std::vector<std::vector<double>>> best_path_and_cost;

    for (size_t i = 0; i < std::get<0>(full_path).size(); i++)
    {
        std::get<0>(best_path_and_cost).push_back({std::get<0>(full_path)[i], std::get<1>(full_path)[i]});
    }

    std::get<1>(best_path_and_cost) = std::get<1>(shortest_path_cost);
    std::get<2>(best_path_and_cost) = std::get<0>(shortest_path_cost);
    return best_path_and_cost;
}

std::vector<KDPoint> dubinise_path(std::vector<KDPoint> &waypoints, double r, double step)
{
    std::vector<KDPoint> path;
    auto compute_yaw = [](KDPoint &p1, KDPoint &p2)
    {
        return atan2(p2[1] - p1[1], p2[0] - p1[0]);
    };

    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        KDPoint p1 = waypoints[i];
        KDPoint p2 = waypoints[i + 1];
        if (i == waypoints.size() - 2)
        {
            p1.push_back(compute_yaw(waypoints[i], waypoints[i + 1]));
            p2.push_back(waypoints[i + 1][2]);
        }
        else
        {
            p1.push_back(compute_yaw(waypoints[i], waypoints[i + 1]));
            p2.push_back(compute_yaw(waypoints[i + 1], waypoints[i + 2]));
        }
        auto best_path_and_cost = get_dubins_best_path_and_cost(p1, p2, r, step);
        path.insert(path.end(), std::get<0>(best_path_and_cost).begin(),
                    std::get<0>(best_path_and_cost).end());
    }
    return path;
}

class DubinsCurvePublisher : public rclcpp::Node
{
public:
    DubinsCurvePublisher() : Node("DubinsCurvePublisher")
    {
        const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);
        const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_publishers);

        // Subscribers
        sub_path_pos1_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path_pos1_to_gates", qos_sub, std::bind(&DubinsCurvePublisher::cb_path_pos1_, this, std::placeholders::_1));
        sub_path_pos2_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path_pos2_to_gates", qos_sub, std::bind(&DubinsCurvePublisher::cb_path_pos2_, this, std::placeholders::_1));
        sub_pos1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/published_pos1", qos_sub, std::bind(&DubinsCurvePublisher::cb_pos1_, this, std::placeholders::_1));

        sub_pos2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/published_pos2", qos_sub, std::bind(&DubinsCurvePublisher::cb_pos2_, this, std::placeholders::_1));
        sub_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/published_gates", qos_sub, std::bind(&DubinsCurvePublisher::cb_gates_, this, std::placeholders::_1));
        sub_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/inflated_obstacles", qos_sub, std::bind(&DubinsCurvePublisher::cb_obstacles_, this, std::placeholders::_1));
        sub_arena_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "/inflated_arena", qos_sub, std::bind(&DubinsCurvePublisher::cb_arena_, this, std::placeholders::_1));

        // Publishers
        pub_dubins_path_pos1_ = this->create_publisher<nav_msgs::msg::Path>("/dubins_path_pos1", qos_pub);
        pub_dubins_path_pos2_ = this->create_publisher<nav_msgs::msg::Path>("/dubins_path_pos2", qos_pub);

        // Service client for triggering remapping
        client_re_mapping_trigger = this->create_client<std_srvs::srv::Trigger>("/service_trigger_inflated");
        // Trigger service
        srv_trigger = this->create_service<std_srvs::srv::Trigger>(
        "/service_trigger_dubins_path",   
        std::bind(&DubinsCurvePublisher::callback_trigger, this, std::placeholders::_1, std::placeholders::_2)
        );
        init_timer_ = this->create_wall_timer(3s, std::bind(&DubinsCurvePublisher::init_service_call_, this));

        // Dubins parameters
        dubins_radius_ = .5;           // Turning radius
        dubins_step_ = 0.1;            // Discretization step
        min_distance_threshold_ = 1.0; // Minimum distance to keep points

        RCLCPP_INFO(this->get_logger(), "DubinsCurvePublisher initialized");
    }

private:
    
    void callback_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
    if (!latest_dubins_path_1_.poses.empty())
    {
      pub_dubins_path_pos1_->publish(latest_dubins_path_1_);
      RCLCPP_INFO(get_logger(), "Published /dubins_path_pos1 via trigger with %zu poses.", latest_dubins_path_1_.poses.size());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "No path_1 available to publish via trigger.");
    }

    if (!latest_dubins_path_2_.poses.empty())
    {
      pub_dubins_path_pos2_->publish(latest_dubins_path_2_);
      RCLCPP_INFO(get_logger(), "Published /dubins_path_pos2 via trigger with %zu poses.", latest_dubins_path_2_.poses.size());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "No path_2 available to publish via trigger.");
    }

    response->success = true;
    response->message = "Paths published successfully.";
    RCLCPP_INFO(get_logger(), "Trigger service called and paths published.");
  }

    void init_service_call_()
    {
        init_timer_->cancel();
        send_trigger_once_();
    }

    void send_trigger_once_()
    {
        for (int i = 0; i < 5 && rclcpp::ok(); ++i)
        {
            if (client_re_mapping_trigger->wait_for_service(1s))
                break;
            RCLCPP_WARN(this->get_logger(), "Waiting for /service_trigger_inflated service...");
        }
        if (!client_re_mapping_trigger->service_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Service /service_trigger_inflated not available.");
            return;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        (void)client_re_mapping_trigger->async_send_request(req,
                                                            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
                                                            {
                                                                try
                                                                {
                                                                    auto resp = future.get();
                                                                    if (resp->success)
                                                                    {
                                                                        RCLCPP_INFO(this->get_logger(), "Trigger OK: %s", resp->message.c_str());
                                                                    }
                                                                    else
                                                                    {
                                                                        RCLCPP_WARN(this->get_logger(), "Trigger FAIL: %s", resp->message.c_str());
                                                                    }
                                                                }
                                                                catch (const std::exception &e)
                                                                {
                                                                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                                                                }
                                                            });
    }

    static double yaw_from_quat_(const geometry_msgs::msg::Quaternion &qmsg)
    {
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
    }

    static geometry_msgs::msg::Quaternion quat_from_yaw_(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::Quaternion qmsg;
        tf2::convert(q, qmsg);
        return qmsg;
    }

    double distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return sqrt(dx * dx + dy * dy);
    }

    bool is_point_in_obstacle(double x, double y, const obstacles_msgs::msg::ObstacleArrayMsg &obstacles, double safety_margin = 0.1)
    {
        for (const auto &obstacle : obstacles.obstacles)
        {
            // Check if point is inside polygon using ray casting
            bool inside = false;
            for (size_t j = 0, k = obstacle.polygon.points.size() - 1; j < obstacle.polygon.points.size(); k = j++)
            {
                const auto &pi = obstacle.polygon.points[j];
                const auto &pj = obstacle.polygon.points[k];

                if (((pi.y > y) != (pj.y > y)) &&
                    (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x))
                {
                    inside = !inside;
                }
            }

            if (inside)
                return true;
            for (size_t i = 0; i < obstacle.polygon.points.size(); ++i)
            {
                const auto &p1 = obstacle.polygon.points[i];
                const auto &p2 = obstacle.polygon.points[(i + 1) % obstacle.polygon.points.size()];

                double dist_to_edge = point_to_line_distance(x, y, p1.x, p1.y, p2.x, p2.y);
                if (dist_to_edge < safety_margin)
                    return true;
            }
        }
        return false;
    }

    bool is_point_in_arena(double x, double y, const geometry_msgs::msg::Polygon &arena)
    {
        if (arena.points.empty())
            return true;

        bool inside = false;
        for (size_t i = 0, j = arena.points.size() - 1; i < arena.points.size(); j = i++)
        {
            const auto &pi = arena.points[i];
            const auto &pj = arena.points[j];

            if (((pi.y > y) != (pj.y > y)) &&
                (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    double point_to_line_distance(double px, double py, double x1, double y1, double x2, double y2)
    {
        double A = px - x1;
        double B = py - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;

        if (len_sq < 1e-6)
            return sqrt(A * A + B * B);

        double param = dot / len_sq;

        double xx, yy;
        if (param < 0)
        {
            xx = x1;
            yy = y1;
        }
        else if (param > 1)
        {
            xx = x2;
            yy = y2;
        }
        else
        {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = px - xx;
        double dy = py - yy;
        return sqrt(dx * dx + dy * dy);
    }

    bool is_path_segment_valid(const KDPoint &start, const KDPoint &end,
                               const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                               const geometry_msgs::msg::Polygon &arena,
                               double step_size = 0.05)
    {
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double distance = sqrt(dx * dx + dy * dy);

        if (distance < 1e-6)
            return true;

        int num_checks = static_cast<int>(distance / step_size) + 1;

        for (int i = 0; i <= num_checks; ++i)
        {
            double t = static_cast<double>(i) / num_checks;
            double x = start[0] + t * dx;
            double y = start[1] + t * dy;

            if (!is_point_in_arena(x, y, arena) || is_point_in_obstacle(x, y, obstacles))
            {
                return false;
            }
        }
        return true;
    }

    // Filter path points that are too close to robot
    std::vector<geometry_msgs::msg::PoseStamped> filter_path_points(
        const nav_msgs::msg::Path &original_path,
        const geometry_msgs::msg::Point &robot_pos)
    {
        std::vector<geometry_msgs::msg::PoseStamped> filtered_points;

        for (const auto &pose : original_path.poses)
        {
            if (distance(pose.pose.position, robot_pos) >= min_distance_threshold_)
            {
                filtered_points.push_back(pose);
            }
        }

        return filtered_points;
    }

    // Generate Dubins path
    nav_msgs::msg::Path generate_dubins_path(
        const geometry_msgs::msg::Pose &robot_pose,
        const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
        const std::string &frame_id)
    {
        nav_msgs::msg::Path dubins_path;
        dubins_path.header.frame_id = frame_id;
        dubins_path.header.stamp = this->get_clock()->now();

        if (waypoints.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No waypoints to generate Dubins path");
            return dubins_path;
        }

        std::vector<KDPoint> kd_waypoints;

        double robot_yaw = yaw_from_quat_(robot_pose.orientation);
        kd_waypoints.push_back({robot_pose.position.x, robot_pose.position.y, robot_yaw});

        // Add waypoints with computed orientations
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            double waypoint_yaw;
            if (i == waypoints.size() - 1)
            {
                waypoint_yaw = yaw_from_quat_(waypoints[i].pose.orientation);
            }
            else
            {
                const auto &current = waypoints[i].pose.position;
                const auto &next = waypoints[i + 1].pose.position;
                waypoint_yaw = atan2(next.y - current.y, next.x - current.x);
            }

            kd_waypoints.push_back({waypoints[i].pose.position.x,
                                    waypoints[i].pose.position.y,
                                    waypoint_yaw});
        }

        // Generate Dubins path (with collision checking if data available)
        std::vector<KDPoint> dubins_points;
        if (got_obstacles_ && got_arena_)
        {
            dubins_points = generate_safe_dubins_path(kd_waypoints);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle or arena data not available, generating unconstrained path");
            dubins_points = dubinise_path(kd_waypoints, dubins_radius_, dubins_step_);
        }

        for (const auto &point : dubins_points)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = frame_id;
            pose_stamped.header.stamp = dubins_path.header.stamp;

            pose_stamped.pose.position.x = point[0];
            pose_stamped.pose.position.y = point[1];
            pose_stamped.pose.position.z = 0.0;

            // Compute orientation for this point
            double yaw = 0.0;
            if (dubins_points.size() > 1)
            {
                auto it = std::find(dubins_points.begin(), dubins_points.end(), point);
                if (it != dubins_points.end() && std::next(it) != dubins_points.end())
                {
                    auto next_point = *std::next(it);
                    yaw = atan2(next_point[1] - point[1], next_point[0] - point[0]);
                }
            }

            pose_stamped.pose.orientation = quat_from_yaw_(yaw);
            dubins_path.poses.push_back(pose_stamped);
        }

        return dubins_path;
    }

    // Generate collision-free Dubins path
    std::vector<KDPoint> generate_safe_dubins_path(const std::vector<KDPoint> &waypoints)
    {
        std::vector<KDPoint> safe_path;

        if (waypoints.size() < 2)
        {
            return safe_path;
        }

        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            KDPoint start = waypoints[i];
            KDPoint end = waypoints[i + 1];

            // Try different turning radii if the original fails
            std::vector<double> radii_to_try = {dubins_radius_, dubins_radius_ * 0.8, dubins_radius_ * 1.2, dubins_radius_ * 0.6, dubins_radius_ * 1.5};

            bool path_found = false;
            for (double radius : radii_to_try)
            {
                auto segment_path = generate_safe_dubins_segment(start, end, radius);
                if (!segment_path.empty())
                {
                    bool segment_valid = true;
                    for (size_t j = 0; j < segment_path.size() - 1; ++j)
                    {
                        if (!is_path_segment_valid(segment_path[j], segment_path[j + 1], last_obstacles_, last_arena_))
                        {
                            segment_valid = false;
                            break;
                        }
                    }

                    if (segment_valid)
                    {
                        if (safe_path.empty())
                        {
                            safe_path.insert(safe_path.end(), segment_path.begin(), segment_path.end());
                        }
                        else
                        {
                            safe_path.insert(safe_path.end(), segment_path.begin() + 1, segment_path.end());
                        }
                        path_found = true;
                        break;
                    }
                }
            }

            if (!path_found)
            {
                RCLCPP_WARN(this->get_logger(), "Could not find collision-free Dubins path for segment %zu", i);
                auto fallback_segment = generate_fallback_segment(start, end);
                if (safe_path.empty())
                {
                    safe_path.insert(safe_path.end(), fallback_segment.begin(), fallback_segment.end());
                }
                else
                {
                    safe_path.insert(safe_path.end(), fallback_segment.begin() + 1, fallback_segment.end());
                }
            }
        }

        return safe_path;
    }

    // Generate single Dubins segment
    std::vector<KDPoint> generate_safe_dubins_segment(const KDPoint &start, const KDPoint &end, double radius)
    {
        std::vector<double> start_vec = {start[0], start[1], start[2]};
        std::vector<double> end_vec = {end[0], end[1], end[2]};

        auto best_path_and_cost = get_dubins_best_path_and_cost(start_vec, end_vec, radius, dubins_step_);
        return std::get<0>(best_path_and_cost);
    }

    // Generate fallback straight-line segment
    std::vector<KDPoint> generate_fallback_segment(const KDPoint &start, const KDPoint &end)
    {
        std::vector<KDPoint> segment;

        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double distance = sqrt(dx * dx + dy * dy);

        if (distance < 1e-6)
        {
            segment.push_back(start);
            return segment;
        }

        int num_points = static_cast<int>(distance / dubins_step_) + 1;

        for (int i = 0; i <= num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;
            KDPoint point = {
                start[0] + t * dx,
                start[1] + t * dy};
            segment.push_back(point);
        }

        return segment;
    }

    // Callbacks
    void cb_pos1_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pos1_ = *msg;
        got_pos1_ = true;
    }

    void cb_pos2_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pos2_ = *msg;
        got_pos2_ = true;
    }

    void cb_gates_(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        last_gates_ = *msg;
        got_gates_ = true;
    }

    void cb_obstacles_(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        last_obstacles_ = *msg;
        got_obstacles_ = true;
        RCLCPP_INFO(this->get_logger(), "Received obstacles: %zu obstacles", last_obstacles_.obstacles.size());
    }

    void cb_arena_(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        last_arena_ = *msg;
        got_arena_ = true;
        RCLCPP_INFO(this->get_logger(), "Received arena: %zu points", last_arena_.points.size());
    }

    void cb_path_pos1_(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!got_pos1_)
        {
            RCLCPP_WARN(this->get_logger(), "Robot 1 position not available yet");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Processing Dubins path for Robot 1");
        auto filtered_points = filter_path_points(*msg, last_pos1_.pose.pose.position);
        latest_dubins_path_1_ = generate_dubins_path(
            last_pos1_.pose.pose,
            filtered_points,
            msg->header.frame_id);
        //pub_dubins_path_pos1_->publish(dubins_path);
        // RCLCPP_INFO(this->get_logger(), "Published Dubins path for Robot 1 with %zu points",
        //             dubins_path.poses.size());
    }

    void cb_path_pos2_(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!got_pos2_)
        {
            RCLCPP_WARN(this->get_logger(), "Robot 2 position not available yet");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Processing Dubins path for Robot 2");
        auto filtered_points = filter_path_points(*msg, last_pos2_.pose.pose.position);
        latest_dubins_path_2_ = generate_dubins_path(
            last_pos2_.pose.pose,
            filtered_points,
            msg->header.frame_id);
        //pub_dubins_path_pos2_->publish(dubins_path);
        // RCLCPP_INFO(this->get_logger(), "Published Dubins path for Robot 2 with %zu points",
        //             dubins_path.poses.size());
    }

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
    // Data storage
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_re_mapping_trigger;
    rclcpp::TimerBase::SharedPtr init_timer_;

    geometry_msgs::msg::PoseWithCovarianceStamped last_pos1_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos2_;

    nav_msgs::msg::Path latest_dubins_path_1_;
    nav_msgs::msg::Path latest_dubins_path_2_;

    geometry_msgs::msg::PoseArray last_gates_;
    obstacles_msgs::msg::ObstacleArrayMsg last_obstacles_;
    geometry_msgs::msg::Polygon last_arena_;

    bool got_pos1_{false};
    bool got_pos2_{false};
    bool got_gates_{false};
    bool got_obstacles_{false};
    bool got_arena_{false};

    // Dubins parameters
    double dubins_radius_;
    double dubins_step_;
    double min_distance_threshold_;
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