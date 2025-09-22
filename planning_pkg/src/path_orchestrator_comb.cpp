#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <utility>
#include <algorithm>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <obstacles_msgs/msg/obstacle_array_msg.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planning_pkg/common.hpp"
#include "planning_pkg/common_function.hpp"
#include "planning_pkg/comb_path.hpp"
#include "planning_pkg/linear_path.hpp"
#include "planning_pkg/visualizer_rviz.hpp"
using rclcpp::QoS;
using namespace std::chrono_literals;

class PathPlanningOrchestratorClient : public rclcpp::Node
{
public:
    PathPlanningOrchestratorClient() : Node("PathPlanningOrchestratorClient"), visualizer_(this)
    {
        const auto now = this->get_clock()->now();

        // QoS for subscribers / publishers
        const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);
        const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_publishers);

        // Subscriber
        sub_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("/inflated_obstacles", qos_sub,
                                                                                          std::bind(&PathPlanningOrchestratorClient::cb_obstacles_, this, std::placeholders::_1));
        sub_arena_ = this->create_subscription<geometry_msgs::msg::Polygon>("/inflated_arena", qos_sub,
                                                                            std::bind(&PathPlanningOrchestratorClient::cb_arena_, this, std::placeholders::_1));
        sub_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/published_gates", qos_sub,
                                                                              std::bind(&PathPlanningOrchestratorClient::cb_gates_, this, std::placeholders::_1));
        sub_pos1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos1", qos_sub,
                                                                                             std::bind(&PathPlanningOrchestratorClient::cb_pos1_, this, std::placeholders::_1));
        sub_pos2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos2", qos_sub,
                                                                                             std::bind(&PathPlanningOrchestratorClient::cb_pos2_, this, std::placeholders::_1));
        // Client
        client_re_mapping_trigger = this->create_client<std_srvs::srv::Trigger>("/service_trigger_inflated");

        // Publisher
        pub_path_pos1_ = this->create_publisher<nav_msgs::msg::Path>("/path_pos1_to_gates", qos_pub);
        pub_path_pos2_ = this->create_publisher<nav_msgs::msg::Path>("/path_pos2_to_gates", qos_pub);

        connection_timer_ = this->create_wall_timer(2s, std::bind(&PathPlanningOrchestratorClient::on_connection_ready_, this));
        init_timer_ = this->create_wall_timer(3s, std::bind(&PathPlanningOrchestratorClient::init_service_call_, this));

        path_gen_ = planning_pkg::CombPathGenerator("map", 0.1);
        linear_gen_ = planning_pkg::LinearPathGenerator("map", 0.1);
        connections_ready_ = false;
        path_planning_ready_ = false;

        RCLCPP_INFO(this->get_logger(), "Orchestor Ready- waiting for connections...");
    }

    void call_service()
    {
        send_trigger_once_();
    }

private:
    // ===== Connection readiness callback =====
    void on_connection_ready_()
    {
        connection_timer_->cancel();
        connections_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Publisher connections established");
    }

    // ===== Initialization timer ===== > to call the service una sola volta all'avvio (per prendere i valori iniziali di posizione, ostacoli ..)
    void init_service_call_()
    {
        init_timer_->cancel();
        send_trigger_once_();
    }

    // ===== Service trigger re_mapper call =====
    void send_trigger_once_()
    {
        for (int i = 0; i < 5 && rclcpp::ok(); ++i)
        {
            if (client_re_mapping_trigger->wait_for_service(1s))
                break;
            RCLCPP_WARN(this->get_logger(), "Waiting for /service_trigger_inflated of remapping script service...");
        }
        if (!client_re_mapping_trigger->service_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Service /service_trigger_inflated of remapping script not available.");
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

    // ===== other methods to check connections =====
    bool wait_for_subscribers(int timeout_seconds = 3)
    {
        const auto timeout = std::chrono::seconds(timeout_seconds);
        const auto start = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() - start < timeout)
        {
            size_t path1_subs = pub_path_pos1_->get_subscription_count();
            size_t path2_subs = pub_path_pos2_->get_subscription_count();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Waiting for subscribers... path1: %zu, path2: %zu", path1_subs, path2_subs);
            if (path1_subs > 0 && path2_subs > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Subscribers connected! path1: %zu, path2: %zu", path1_subs, path2_subs);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_WARN(this->get_logger(), "No subscribers found, proceeding anyway...");
        return false;
    }

    void publish_path_with_retry(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                                 const nav_msgs::msg::Path &path,
                                 const std::string &name,
                                 int max_retries = 2)
    {
        if (path.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path %s vuoto, skip publish.", name.c_str());
            return;
        }

        for (int attempt = 0; attempt < max_retries; ++attempt)
        {
            try
            {
                pub->publish(path);
                RCLCPP_INFO(this->get_logger(), "Pubblicato %s (%zu poses) - tentativo %d",
                            name.c_str(), path.poses.size(), attempt + 1);
                // Piccola pausa per permettere la trasmissione
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                break; // Successo, esci dal loop
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Errore pubblicazione %s (tentativo %d): %s",
                            name.c_str(), attempt + 1, e.what());

                if (attempt < max_retries - 1)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
    }

    // ===== Callbacks =====
    void cb_obstacles_(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        last_obstacles_ = *msg;
        got_obstacles_ = true;
        RCLCPP_INFO(this->get_logger(), "Received obstacles inflated: %zu obstacles",
                    last_obstacles_.obstacles.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        path_planning();
    }

    void cb_arena_(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        last_arena_ = *msg;
        got_arena_ = true;
        RCLCPP_INFO(this->get_logger(), "Received arena inflated: %zu points",
                    last_arena_.points.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        path_planning();
    }

    void cb_gates_(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        last_gates_ = *msg;
        got_gates_ = true;

        RCLCPP_INFO(this->get_logger(), "Received gates: %zu", last_gates_.poses.size());
        for (size_t i = 0; i < last_gates_.poses.size(); ++i)
        {
            const auto &p = last_gates_.poses[i];
            const double yaw = planning_pkg::CommonFunction::yaw_from_quat_(p.orientation);
            RCLCPP_INFO(this->get_logger(), "Gate %zu -> x=%.3f, y=%.3f, yaw=%.3f",
                        i, p.position.x, p.position.y, yaw);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        path_planning();
    }

    void cb_pos1_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pos1_ = *msg;
        got_pos1_ = true;

        const auto &p = last_pos1_.pose.pose;
        const double yaw = planning_pkg::CommonFunction::yaw_from_quat_(p.orientation);
        RCLCPP_INFO(this->get_logger(), "Received Pos1 -> x=%.3f, y=%.3f, yaw=%.3f",
                    p.position.x, p.position.y, yaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        path_planning();
    }

    void cb_pos2_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pos2_ = *msg;
        got_pos2_ = true;

        const auto &p = last_pos2_.pose.pose;
        const double yaw = planning_pkg::CommonFunction::yaw_from_quat_(p.orientation);
        RCLCPP_INFO(this->get_logger(), "Received Pos2 -> x=%.3f, y=%.3f, yaw=%.3f",
                    p.position.x, p.position.y, yaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        path_planning();
    }

    std::pair<double, double> find_nearest_gate(const geometry_msgs::msg::Point &pos, const geometry_msgs::msg::PoseArray &gates)
    {
        double min_dist_sq = std::numeric_limits<double>::max();
        std::pair<double, double> nearest_gate_pos;

        for (const auto &g : gates.poses)
        {
            const double dx = g.position.x - pos.x;
            const double dy = g.position.y - pos.y;
            const double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                nearest_gate_pos = {g.position.x, g.position.y};
            }
        }
        return nearest_gate_pos;
    }

    void path_planning()
    {
        if (path_planning_ready_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Path planning already in progress, skipping...");
            return;
        }
        if (!got_obstacles_ || !got_arena_ || !got_gates_ || !got_pos1_ || !got_pos2_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Data not complete yet, WAITING... obstacles:%d arena:%d gates:%d pos1:%d pos2:%d",
                                 got_obstacles_, got_arena_, got_gates_, got_pos1_, got_pos2_);
            return;
        }
        path_planning_ready_ = true;
        if (!connections_ready_)
        {
            RCLCPP_WARN(this->get_logger(), "Connections not ready yet, proceeding anyway...");
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(this->get_logger(), "=== Starting path planning ===");
        const auto &p1 = last_pos1_.pose.pose.position;
        const auto &p2 = last_pos2_.pose.pose.position;

        // 0) find nearest gate for each robot
        const auto nearest_gate_pos1 = find_nearest_gate(p1, last_gates_);
        RCLCPP_INFO(this->get_logger(), "Nearest gate for Robot 1: x=%.2f, y=%.2f",
                    nearest_gate_pos1.first, nearest_gate_pos1.second);
        const auto nearest_gate_pos2 = find_nearest_gate(p2, last_gates_);
        RCLCPP_INFO(this->get_logger(), "Nearest gate for Robot 2: x=%.2f, y=%.2f",
                    nearest_gate_pos2.first, nearest_gate_pos2.second);

        std::vector<std::pair<double, double>> wps_pos1;
        wps_pos1.emplace_back(p1.x, p1.y);
        wps_pos1.emplace_back(nearest_gate_pos1.first, nearest_gate_pos1.second);
        std::vector<std::pair<double, double>> wps_pos2;
        wps_pos2.emplace_back(p2.x, p2.y);
        wps_pos2.emplace_back(nearest_gate_pos2.first, nearest_gate_pos2.second);

        nav_msgs::msg::Path path1;
        nav_msgs::msg::Path path2;
        
        // 1) check if the path is a straight line (no obstacles in between)
        bool is_linear_1 = linear_gen_.is_direct_path_feasible(wps_pos1.front(), wps_pos1.back(), last_obstacles_, last_arena_);
        bool is_linear_2 = linear_gen_.is_direct_path_feasible(wps_pos2.front(), wps_pos2.back(), last_obstacles_, last_arena_);
        
        // is_linear_1 = false; // FORZATURA PER TESTING COMB
        // is_linear_2 = false; // FORZATURA PER TESTING COMB
        
        if (is_linear_1)
        {
            RCLCPP_INFO(this->get_logger(), "Direct path feasible for Robot 1, using linear path generator.");
            path1 = linear_gen_.generate(wps_pos1, last_obstacles_, last_arena_);
        }
        if (is_linear_2)
        {
            RCLCPP_INFO(this->get_logger(), "Direct path feasible for Robot 2,using linear path generator.");
            path2 = linear_gen_.generate(wps_pos2, last_obstacles_, last_arena_);
        }

        // 4) implement COMB only if at least one path is not linear
        if (!is_linear_1 || !is_linear_2)
        {

            // ===== visuallizer part =====
            std::vector<std::pair<double, double>> points_line;
            std::vector<std::pair<double, double>> points_centroids;
            std::vector<std::pair<double, double>> points = path_gen_.get_pointlist(last_obstacles_, last_arena_);
            std::vector<planning_pkg::HorizontalLine> horizontal_lines = path_gen_.get_horizontal_lines(last_obstacles_, last_arena_);

            for (const auto &line : horizontal_lines)
            {
                auto line_points = path_gen_.set_point_in_vertical_line(line, 1.0);
                points_line.insert(points_line.end(), line_points.begin(), line_points.end());
            }
            std::vector<planning_pkg::Cell> cells = path_gen_.get_cells_btw_vlines(horizontal_lines);
            for (const auto &cell : cells)
            {
                auto centroid = path_gen_.get_cell_centroid(cell);
                points_centroids.push_back(centroid);
            }
            std::vector<std::vector<int>> arc_list = path_gen_.get_arc(horizontal_lines, points_line, points_centroids);
            vertical_lines_data_.clear();
            for (const auto &line : horizontal_lines)
            {
                vertical_lines_data_.emplace_back(
                    line.y,
                    0.0, // z coordinate (sempre 0 per linee orizzontali)
                    std::make_pair(line.x_start, line.y),
                    std::make_pair(line.x_end, line.y));
            }

            visualizer_.vis_points(points_centroids); // "points_centroids" or "points_line" or "points"
            visualizer_.vis_line(horizontal_lines);
            visualizer_.vis_cells(cells);
            visualizer_.vis_arcs(arc_list, points_line, points_centroids);
        }

        // 4) Generate patjc with step of 0.1 m and frame "map"
        RCLCPP_INFO(this->get_logger(), "Generating path for Robot 1 and 2...");
        try
        {
            if (!is_linear_1)
            {
                path1 = path_gen_.generate(wps_pos1, last_obstacles_, last_arena_);
            }
            if (!is_linear_2)
            {
                path2 = path_gen_.generate(wps_pos2, last_obstacles_, last_arena_);
            }
            if (!path1.poses.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Path generate for Robot 1: %zu waypoints", path1.poses.size());
            }

            if (!path2.poses.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Path generate for Robot 2: %zu waypoints", path2.poses.size());
            }
            else if (path1.poses.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "NO path for Robot 1");
            }
            else if (path2.poses.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "NO path for Robot 2");
            }
        }

        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error to generate the path for robots: %s", e.what());
            path1 = nav_msgs::msg::Path(); // Path vuoto
            path1.header.frame_id = "map";
            path1.header.stamp = this->get_clock()->now();
        }

        // 5) check for subscribers before publishing
        wait_for_subscribers(1);

        // 6) ===== PATH PUBLICATION =====
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");

        // 7) execution time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "Path planning execution time: %ld ms", duration.count());

        publish_path_with_retry(pub_path_pos1_, path1, "path_pos1_to_gates");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        publish_path_with_retry(pub_path_pos2_, path2, "path_pos2_to_gates");

        RCLCPP_INFO(this->get_logger(), "=== Path planning completed ===");
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_re_mapping_trigger;
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr connection_timer_;

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_arena_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos2_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_pos1_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_pos2_;

    obstacles_msgs::msg::ObstacleArrayMsg last_obstacles_;
    geometry_msgs::msg::Polygon last_arena_;
    geometry_msgs::msg::PoseArray last_gates_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos1_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos2_;

    bool got_obstacles_{false};
    bool got_arena_{false};
    bool got_gates_{false};
    bool got_pos1_{false};
    bool got_pos2_{false};
    bool connections_ready_{false};
    bool path_planning_ready_{false};

    planning_pkg::CombPathGenerator path_gen_;
    planning_pkg::LinearPathGenerator linear_gen_;
    planning_pkg::VisualizationUtils visualizer_;
    std::vector<std::tuple<double, double, std::pair<double, double>, std::pair<double, double>>> vertical_lines_data_;
    static std::pair<double, double> xy_from_(const geometry_msgs::msg::Point &p) { return {p.x, p.y}; }
    static std::pair<double, double> xy_from_(const geometry_msgs::msg::Pose &p) { return {p.position.x, p.position.y}; }
    template <typename A, typename B>
    static std::pair<double, double> xy_from_(const std::pair<A, B> &p) { return {static_cast<double>(p.first), static_cast<double>(p.second)}; }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningOrchestratorClient>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    RCLCPP_INFO(node->get_logger(), "Starting PathPlanningOrchestratorClient with MultiThreadedExecutor");

    exec.spin();
    rclcpp::shutdown();
    return 0;
}