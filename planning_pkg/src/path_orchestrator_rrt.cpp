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
#include "planning_pkg/rrt_path.hpp"
#include "planning_pkg/multiagent_astar.hpp"
#include "planning_pkg/linear_path.hpp"
#include "planning_pkg/visualizer_rviz.hpp"

using rclcpp::QoS;
using namespace std::chrono_literals;

class RRTPlanningOrchestratorClient : public rclcpp::Node
{
public:
    RRTPlanningOrchestratorClient() : Node("RRTPlanningOrchestratorClient"), visualizer_(this)
    {
        const auto now = this->get_clock()->now();

        // QoS for subscribers / publishers
        const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);
        const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_publishers);

        // Subscriber
        sub_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>("/inflated_obstacles", qos_sub,
                                                                                          std::bind(&RRTPlanningOrchestratorClient::cb_obstacles_, this, std::placeholders::_1));
        sub_arena_ = this->create_subscription<geometry_msgs::msg::Polygon>("/inflated_arena", qos_sub,
                                                                            std::bind(&RRTPlanningOrchestratorClient::cb_arena_, this, std::placeholders::_1));
        sub_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/published_gates", qos_sub,
                                                                              std::bind(&RRTPlanningOrchestratorClient::cb_gates_, this, std::placeholders::_1));
        sub_pos1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos1", qos_sub,
                                                                                             std::bind(&RRTPlanningOrchestratorClient::cb_pos1_, this, std::placeholders::_1));
        sub_pos2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos2", qos_sub,
                                                                                             std::bind(&RRTPlanningOrchestratorClient::cb_pos2_, this, std::placeholders::_1));
        // Client
        client_re_mapping_trigger = this->create_client<std_srvs::srv::Trigger>("/service_trigger_inflated");

        // Publisher
        pub_path_pos1_ = this->create_publisher<nav_msgs::msg::Path>("/path_pos1_to_gates", qos_pub);
        pub_path_pos2_ = this->create_publisher<nav_msgs::msg::Path>("/path_pos2_to_gates", qos_pub);

        connection_timer_ = this->create_wall_timer(2s, std::bind(&RRTPlanningOrchestratorClient::on_connection_ready_, this));
        init_timer_ = this->create_wall_timer(3s, std::bind(&RRTPlanningOrchestratorClient::init_service_call_, this));

        // Initialize planners
        rrt_gen_ = planning_pkg::RRTPathGenerator("map", 0.1);
        multiagent_astar_ = planning_pkg::MultiAgentAStar("map", 0.1);
        linear_gen_ = planning_pkg::LinearPathGenerator("map", 0.1);
        
        // Configure RRT
        planning_pkg::RRTConfig rrt_config;
        rrt_config.step_size = 0.5;
        rrt_config.goal_tolerance = 0.3;
        rrt_config.max_iterations = 5000;
        rrt_config.goal_bias = 0.1;
        rrt_config.obstacle_clearance = 0.15;
        rrt_gen_.set_config(rrt_config);

        // Configure Multi-Agent A*
        planning_pkg::AStarCAConfig astar_config;
        astar_config.time_step = 0.1;
        astar_config.max_timesteps = 1000;
        astar_config.collision_radius = 0.2;
        astar_config.max_speed = 1.0;
        astar_config.goal_tolerance = 0.3;
        astar_config.use_heuristic = true;
        astar_config.use_conflict_avoidance = true;
        multiagent_astar_.set_config(astar_config);

        connections_ready_ = false;
        path_planning_ready_ = false;
        use_multiagent_planning_ = false;

        RCLCPP_INFO(this->get_logger(), "RRT Orchestrator Ready - waiting for connections...");
    }

    void call_service()
    {
        send_trigger_once_();
    }

    void set_use_multiagent_planning(bool use_multiagent)
    {
        use_multiagent_planning_ = use_multiagent;
        RCLCPP_INFO(this->get_logger(), "Multi-agent planning: %s", use_multiagent ? "ENABLED" : "DISABLED");
    }

private:
    // ===== Connection readiness callback =====
    void on_connection_ready_()
    {
        connection_timer_->cancel();
        connections_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Publisher connections established");
    }

    // ===== Initialization timer =====
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
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                break;
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
        RCLCPP_INFO(this->get_logger(), "=== Starting RRT path planning ===");
        
        const auto &p1 = last_pos1_.pose.pose.position;
        const auto &p2 = last_pos2_.pose.pose.position;

        // Find nearest gate for each robot
        const auto nearest_gate_pos1 = find_nearest_gate(p1, last_gates_);
        RCLCPP_INFO(this->get_logger(), "Nearest gate for Robot 1: x=%.2f, y=%.2f",
                    nearest_gate_pos1.first, nearest_gate_pos1.second);
        const auto nearest_gate_pos2 = find_nearest_gate(p2, last_gates_);
        RCLCPP_INFO(this->get_logger(), "Nearest gate for Robot 2: x=%.2f, y=%.2f",
                    nearest_gate_pos2.first, nearest_gate_pos2.second);

        nav_msgs::msg::Path path1, path2;

        if (use_multiagent_planning_)
        {
            // Use Multi-Agent A*CA* for coordinated planning
            RCLCPP_INFO(this->get_logger(), "Using Multi-Agent A*CA* planning");
            
            std::vector<planning_pkg::Agent> agents;
            
            geometry_msgs::msg::Point goal1;
            goal1.x = nearest_gate_pos1.first;
            goal1.y = nearest_gate_pos1.second;
            goal1.z = 0.0;
            
            geometry_msgs::msg::Point goal2;
            goal2.x = nearest_gate_pos2.first;
            goal2.y = nearest_gate_pos2.second;
            goal2.z = 0.0;
            
            agents.emplace_back(1, p1, goal1);
            agents.emplace_back(2, p2, goal2);

            std::vector<planning_pkg::AgentPath> agent_paths = multiagent_astar_.plan_paths(agents, last_obstacles_, last_arena_);
            
            if (agent_paths.size() >= 2 && agent_paths[0].valid && agent_paths[1].valid)
            {
                path1 = multiagent_astar_.agent_path_to_nav_path(agent_paths[0]);
                path2 = multiagent_astar_.agent_path_to_nav_path(agent_paths[1]);
                
                auto stats = multiagent_astar_.get_last_planning_stats();
                RCLCPP_INFO(this->get_logger(), "Multi-agent planning completed: %d conflicts detected, %d constraints added",
                           stats.total_conflicts_detected, stats.total_constraints_added);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Multi-agent planning failed, falling back to individual RRT");
                use_multiagent_planning_ = false;
            }
        }

        if (!use_multiagent_planning_)
        {
            // Use individual RRT planning
            RCLCPP_INFO(this->get_logger(), "Using individual RRT planning");
            
            std::vector<std::pair<double, double>> wps_pos1;
            wps_pos1.emplace_back(p1.x, p1.y);
            wps_pos1.emplace_back(nearest_gate_pos1.first, nearest_gate_pos1.second);
            
            std::vector<std::pair<double, double>> wps_pos2;
            wps_pos2.emplace_back(p2.x, p2.y);
            wps_pos2.emplace_back(nearest_gate_pos2.first, nearest_gate_pos2.second);

            // Check if direct paths are feasible
            bool is_linear_1 = linear_gen_.is_direct_path_feasible(wps_pos1.front(), wps_pos1.back(), last_obstacles_, last_arena_);
            bool is_linear_2 = linear_gen_.is_direct_path_feasible(wps_pos2.front(), wps_pos2.back(), last_obstacles_, last_arena_);

            if (is_linear_1)
            {
                RCLCPP_INFO(this->get_logger(), "Direct path feasible for Robot 1, using linear path generator.");
                path1 = linear_gen_.generate(wps_pos1, last_obstacles_, last_arena_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Using RRT for Robot 1");
                path1 = rrt_gen_.generate(wps_pos1, last_obstacles_, last_arena_);
            }

            if (is_linear_2)
            {
                RCLCPP_INFO(this->get_logger(), "Direct path feasible for Robot 2, using linear path generator.");
                path2 = linear_gen_.generate(wps_pos2, last_obstacles_, last_arena_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Using RRT for Robot 2");
                path2 = rrt_gen_.generate(wps_pos2, last_obstacles_, last_arena_);
            }
        }

        // Check for subscribers before publishing
        wait_for_subscribers(1);

        // Publish paths
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "Path planning execution time: %ld ms", duration.count());

        publish_path_with_retry(pub_path_pos1_, path1, "path_pos1_to_gates");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        publish_path_with_retry(pub_path_pos2_, path2, "path_pos2_to_gates");

        RCLCPP_INFO(this->get_logger(), "=== RRT path planning completed ===");
        path_planning_ready_ = false;
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
    bool use_multiagent_planning_{false};

    planning_pkg::RRTPathGenerator rrt_gen_;
    planning_pkg::MultiAgentAStar multiagent_astar_;
    planning_pkg::LinearPathGenerator linear_gen_;
    planning_pkg::VisualizationUtils visualizer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTPlanningOrchestratorClient>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    RCLCPP_INFO(node->get_logger(), "Starting RRTPlanningOrchestratorClient with MultiThreadedExecutor");

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
