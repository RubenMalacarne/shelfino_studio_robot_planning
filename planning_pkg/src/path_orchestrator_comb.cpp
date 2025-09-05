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
#include "planning_pkg/comb_path.hpp"

using rclcpp::QoS;
using namespace std::chrono_literals;

class PathPlanningOrchestratorClient : public rclcpp::Node
{
public:
    PathPlanningOrchestratorClient() : Node("PathPlanningOrchestratorClient")
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

        pub_vertical_line = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vertical_line", qos_markers);
        pub_centroid_cel = this->create_publisher<visualization_msgs::msg::MarkerArray>("/centroid_cell", qos_markers);
        pub_region = this->create_publisher<visualization_msgs::msg::MarkerArray>("/region", qos_markers);

        RCLCPP_INFO(this->get_logger(), "Orchestor client initialized!");

        connection_timer_ = this->create_wall_timer(2s, std::bind(&PathPlanningOrchestratorClient::on_connection_ready_, this));
        init_timer_ = this->create_wall_timer(3s, std::bind(&PathPlanningOrchestratorClient::init_service_call_, this));

        RCLCPP_INFO(this->get_logger(), "Orchestor - waiting for connections...");
        // path planning initialization
        path_gen_ = planning_pkg::CombPathGenerator("map", 0.1);

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
        // 1. calcolo decomposizione in celle
        path_gen_.compute_cells_decomposition(last_obstacles_, last_arena_);
        
        // Store vertical lines for visualization
        //vertical_lines_data_ = vertical_lines_data;

        // 2. create region
        path_gen_.create_region(last_obstacles_, last_arena_);
        // Publish generated regions (cells) to /region
        vis_region();
        // Verifica connessioni prima di pubblicare (con timeout breve)
        wait_for_subscribers(1);
        // Pubblicazione robusta con delay tra i messaggi
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");
        nav_msgs::msg::Path path1 = path_gen_.generate(wps_pos1, last_obstacles_, last_arena_);  
        nav_msgs::msg::Path path2 = path_gen_.generate(wps_pos2, last_obstacles_, last_arena_); 
        publish_path_with_retry(pub_path_pos1_, path1, "path pos1->gates");

        // Delay tra pubblicazioni per evitare congestione
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // publish_path_with_retry(pub_path_pos2_, path2, "path pos2->gates");

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        vis_vertical_line();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        vis_centroid_cel();

        RCLCPP_INFO(this->get_logger(), "=== Path planning completed ===");
    }
    
    // ===== Visualization methods =====
    void vis_vertical_line()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing vertical lines for decomposition visualization");
        
        if (vertical_lines_data_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No vertical lines data available");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        for (const auto &line_data : vertical_lines_data_)
        {
            double xL = std::get<0>(line_data);
            double xR = std::get<1>(line_data);
            auto left_interval = std::get<2>(line_data);
            auto right_interval = std::get<3>(line_data);

            // Create left vertical line
            visualization_msgs::msg::Marker left_line;
            left_line.header.frame_id = "map";
            left_line.header.stamp = this->get_clock()->now();
            left_line.ns = "vertical_lines";
            left_line.id = marker_id++;
            left_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            left_line.action = visualization_msgs::msg::Marker::ADD;

            left_line.scale.x = 0.02; 
            left_line.color.r = 0.0;
            left_line.color.g = 1.0;
            left_line.color.b = 0.0;
            left_line.color.a = 0.8;

            geometry_msgs::msg::Point p1, p2;
            p1.x = xL;
            p1.y = left_interval.first;
            p1.z = 0.0;
            p2.x = xL;
            p2.y = left_interval.second;
            p2.z = 0.0;

            left_line.points.push_back(p1);
            left_line.points.push_back(p2);
            marker_array.markers.push_back(left_line);

            // Create right vertical line
            visualization_msgs::msg::Marker right_line;
            right_line.header.frame_id = "map";
            right_line.header.stamp = this->get_clock()->now();
            right_line.ns = "vertical_lines";
            right_line.id = marker_id++;
            right_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            right_line.action = visualization_msgs::msg::Marker::ADD;

            right_line.scale.x = 0.02;
            right_line.color.r = 1.0;
            right_line.color.g = 0.0;
            right_line.color.b = 0.0;
            right_line.color.a = 0.8;

            p1.x = xR;
            p1.y = right_interval.first;
            p1.z = 0.0;
            p2.x = xR;
            p2.y = right_interval.second;
            p2.z = 0.0;

            right_line.points.push_back(p1);
            right_line.points.push_back(p2);
            marker_array.markers.push_back(right_line);

            // Create connecting line between intervals (optional)
            visualization_msgs::msg::Marker connect_line;
            connect_line.header.frame_id = "map";
            connect_line.header.stamp = this->get_clock()->now();
            connect_line.ns = "vertical_lines";
            connect_line.id = marker_id++;
            connect_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            connect_line.action = visualization_msgs::msg::Marker::ADD;

            connect_line.scale.x = 0.01; 
            connect_line.color.r = 0.5;
            connect_line.color.g = 0.5;
            connect_line.color.b = 1.0;
            connect_line.color.a = 0.5;

            // Connect centers of intervals
            p1.x = xL;
            p1.y = (left_interval.first + left_interval.second) / 2.0;
            p1.z = 0.0;
            p2.x = xR;
            p2.y = (right_interval.first + right_interval.second) / 2.0;
            p2.z = 0.0;

            connect_line.points.push_back(p1);
            connect_line.points.push_back(p2);
            marker_array.markers.push_back(connect_line);
        }

        pub_vertical_line->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu vertical line markers", marker_array.markers.size());
    }
    void vis_centroid_cel()
    {
        RCLCPP_INFO(this->get_logger(), "publish centroid of cells about vertical decomposition");
        return;
    }
    void vis_region()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing regions (cells) for decomposition visualization");

        if (path_gen_.cells.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No cells available in path_gen to publish");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        for (const auto &cell : path_gen_.cells)
        {
            double xL, xR, yb, yt;
            std::tie(xL, xR, yb, yt) = cell;

            visualization_msgs::msg::Marker rect;
            rect.header.frame_id = "map";
            rect.header.stamp = this->get_clock()->now();
            rect.ns = "regions";
            rect.id = marker_id++;
            rect.type = visualization_msgs::msg::Marker::LINE_STRIP;
            rect.action = visualization_msgs::msg::Marker::ADD;
            rect.scale.x = 0.02;
            rect.color.r = 0.0f;
            rect.color.g = 0.4f;
            rect.color.b = 1.0f;
            rect.color.a = 0.8f;

            geometry_msgs::msg::Point p;
            p.z = 0.05;
            p.x = xL; p.y = yb; rect.points.push_back(p);
            p.x = xR; p.y = yb; rect.points.push_back(p);
            p.x = xR; p.y = yt; rect.points.push_back(p);
            p.x = xL; p.y = yt; rect.points.push_back(p);
            p.x = xL; p.y = yb; rect.points.push_back(p); // close

            marker_array.markers.push_back(rect);

            // Centroid sphere
            visualization_msgs::msg::Marker centroid;
            centroid.header.frame_id = "map";
            centroid.header.stamp = this->get_clock()->now();
            centroid.ns = "region_centroid";
            centroid.id = marker_id++;
            centroid.type = visualization_msgs::msg::Marker::SPHERE;
            centroid.action = visualization_msgs::msg::Marker::ADD;
            centroid.pose.position.x = (xL + xR) / 2.0;
            centroid.pose.position.y = (yb + yt) / 2.0;
            centroid.pose.position.z = 0.05;
            centroid.pose.orientation.w = 1.0;
            centroid.scale.x = std::max(0.05, std::min(0.5, (xR - xL) * 0.2));
            centroid.scale.y = centroid.scale.x;
            centroid.scale.z = centroid.scale.x;
            centroid.color.r = 1.0f;
            centroid.color.g = 0.5f;
            centroid.color.b = 0.0f;
            centroid.color.a = 0.9f;

            marker_array.markers.push_back(centroid);
        }

        pub_region->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu region markers", marker_array.markers.size());
    }

    
    // Supporta elementi dei tipi: geometry_msgs::msg::Point, geometry_msgs::msg::Pose, std::pair<double,double>
    static std::pair<double, double> xy_from_(const geometry_msgs::msg::Point &p) { return {p.x, p.y}; }
    static std::pair<double, double> xy_from_(const geometry_msgs::msg::Pose &p) { return {p.position.x, p.position.y}; }
    template <typename A, typename B>
    static std::pair<double, double> xy_from_(const std::pair<A, B> &p) { return {static_cast<double>(p.first), static_cast<double>(p.second)}; }

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

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vertical_line;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_centroid_cel;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_region;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_critic_points_;

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

    planning_pkg::CombPathGenerator path_gen_;
    std::vector<std::tuple<double, double, std::pair<double, double>, std::pair<double, double>>> vertical_lines_data_;
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