#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <utility>   // std::pair
#include <algorithm> // std::max
#include <thread>    // std::this_thread

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
#include "planning_pkg/prm_path.hpp"
#include "planning_pkg/visualizer_rviz.hpp"
using rclcpp::QoS;
using namespace std::chrono_literals;

class PathPlanningOrchestratorClient : public rclcpp::Node
{
public:
    PathPlanningOrchestratorClient() : Node("PathPlanningOrchestratorClient") ,visualizer_(this)
    {
        const auto now = this->get_clock()->now();
        // QoS per subscriber
        const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);
        // QoS per marker di visualizzazione
        const auto qos_markers = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_markers);
        // QoS migliorata per publisher
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

        RCLCPP_INFO(this->get_logger(), "Orchestor client initialized!");

        connection_timer_ = this->create_wall_timer(2s, std::bind(&PathPlanningOrchestratorClient::on_connection_ready_, this));
        init_timer_ = this->create_wall_timer(3s, std::bind(&PathPlanningOrchestratorClient::init_service_call_, this));

        RCLCPP_INFO(this->get_logger(), "Orchestor - waiting for connections...");
        // path planning initialization
        path_gen_ = planning_pkg::PrmPathGenerator("map", 0.1);

        // Flag per tracciare se le connessioni sono pronte
        connections_ready_ = false;
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
            const double yaw = yaw_from_quat_(p.orientation);
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
        const double yaw = yaw_from_quat_(p.orientation);
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
        const double yaw = yaw_from_quat_(p.orientation);
        RCLCPP_INFO(this->get_logger(), "Received Pos2 -> x=%.3f, y=%.3f, yaw=%.3f",
                    p.position.x, p.position.y, yaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        path_planning();
    }

    // methods from quaternion to RPY --> nav2 accetta un path con orientamento in RPY
    static double yaw_from_quat_(const geometry_msgs::msg::Quaternion &qmsg)
    {
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
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

    // Convert Dijkstra path indices to nav_msgs::msg::Path
    nav_msgs::msg::Path indices_to_nav_path(const std::vector<int> &path_indices)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->get_clock()->now();

        for (int idx : path_indices)
        {
            if (idx >= 0 && static_cast<size_t>(idx) < path_gen_.random_points.size())
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header = path.header;

                pose_stamped.pose.position = path_gen_.random_points[idx];
                pose_stamped.pose.orientation.w = 1.0; // Default orientation

                path.poses.push_back(pose_stamped);
            }
        }

        return path;
    }

    void path_planning()
    {
        if (!got_obstacles_ || !got_arena_ || !got_gates_ || !got_pos1_ || !got_pos2_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Data not complete yet, WAITING... obstacles:%d arena:%d gates:%d pos1:%d pos2:%d",
                                 got_obstacles_, got_arena_, got_gates_, got_pos1_, got_pos2_);
            return;
        }

        if (!connections_ready_)
        {
            RCLCPP_WARN(this->get_logger(), "Connections not ready yet, proceeding anyway...");
        }

        RCLCPP_INFO(this->get_logger(), "=== Starting path planning ===");

        const auto &p1 = last_pos1_.pose.pose.position;
        const auto &p2 = last_pos2_.pose.pose.position;

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
        // step:

        // 1) Sample random points for PRM
        RCLCPP_INFO(this->get_logger(), "Sampling random points...");
        path_gen_.sample_random_points(last_obstacles_, last_arena_, 100);

        // 2) Build k-NN edges
        RCLCPP_INFO(this->get_logger(), "Building k-NN edges...");
        path_gen_.build_knn_edges(last_obstacles_, last_arena_, 7, 0.15, 0.1);

        // 3) Genera path con step di 0.1 m e frame "map"
        RCLCPP_INFO(this->get_logger(), "Generating paths...");
        nav_msgs::msg::Path path1 = path_gen_.generate(wps_pos1, last_obstacles_, last_arena_);
        nav_msgs::msg::Path path2 = path_gen_.generate(wps_pos2, last_obstacles_, last_arena_);

        // Verifica connessioni prima di pubblicare (con timeout breve)
        wait_for_subscribers(1);
        // Pubblicazione robusta con delay tra i messaggi
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");

        publish_path_with_retry(pub_path_pos1_, path1, "path pos1->gates");

        // Delay tra pubblicazioni per evitare congestione
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        publish_path_with_retry(pub_path_pos2_, path2, "path pos2->gates");

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        visualizer_.vis_random_points_markers(path_gen_.random_points);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        visualizer_.vis_knn_edges_markers(path_gen_.random_points, path_gen_.knn_adj);

        RCLCPP_INFO(this->get_logger(), "=== Path planning completed ===");
    }


    // ===== Member variables =====
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

    planning_pkg::PrmPathGenerator path_gen_;
    planning_pkg::VisualizationUtils visualizer_;
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