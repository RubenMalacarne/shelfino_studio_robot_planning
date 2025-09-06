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

        // QoS for subscribers / markers / publishers
        const auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);
        const auto qos_markers = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_markers);
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
        pub_points_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pointlist_markers", qos_markers);
        pub_cells_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cells_markers", qos_markers);
        pub_arcs_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/arcs_markers", qos_markers);
        connection_timer_ = this->create_wall_timer(2s, std::bind(&PathPlanningOrchestratorClient::on_connection_ready_, this));
        init_timer_ = this->create_wall_timer(3s, std::bind(&PathPlanningOrchestratorClient::init_service_call_, this));
        
        path_gen_ = planning_pkg::CombPathGenerator("map", 0.1);        
        connections_ready_ = false;

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
        // step0: find nearest gate for each robot
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


        // ===== GENERAZIONE PATH PER ROBOT 1 =====
        // Genera le linee orizzontali usando CombPathGenerator
        std::vector<planning_pkg::HorizontalLine> horizontal_lines = path_gen_.get_horizontal_lines(last_obstacles_, last_arena_);

        // Ottieni anche i punti per la visualizzazione
        std::vector<std::pair<double, double>> points = path_gen_.get_pointlist(last_obstacles_, last_arena_);
        std::vector<std::pair<double, double>> points_line;
        std::vector<std::pair<double, double>> points_centroids;

        // Ciclo per usare set_point_in_vertical_line
        for (const auto &line : horizontal_lines)
        {
            auto line_points = path_gen_.set_point_in_vertical_line(line, 1.0);
            points_line.insert(points_line.end(), line_points.begin(), line_points.end());
        }

        // Memorizza i dati per la visualizzazione
        vertical_lines_data_.clear();
        for (const auto &line : horizontal_lines)
        {
            vertical_lines_data_.emplace_back(
                line.y,
                0.0, // z coordinate (sempre 0 per linee orizzontali)
                std::make_pair(line.x_start, line.y),
                std::make_pair(line.x_end, line.y));
        }

        // Ottieni i centroidi delle celle
        std::vector<planning_pkg::Cell> cells = path_gen_.get_cells_btw_vlines(horizontal_lines);
        for (const auto &cell : cells)
        {
            auto centroid = path_gen_.get_cell_centroid(cell);
            points_centroids.push_back(centroid);
        }

        // Visualizzazione
        vis_points(points_centroids);
        vis_line();
        vis_cells(cells);

        std::vector<std::vector<int>> arc_list = path_gen_.get_arc(horizontal_lines, points_line, points_centroids);
        vis_arcs(arc_list, points_line, points_centroids);

        RCLCPP_INFO(this->get_logger(), "Generated %zu points and %zu horizontal lines", points.size(), horizontal_lines.size());

        // ===== GENERAZIONE PATH PER ROBOT 1 =====
        RCLCPP_INFO(this->get_logger(), "Generating path for Robot 1...");
        nav_msgs::msg::Path path1;
        try
        {
            path1 = path_gen_.generate(wps_pos1, last_obstacles_, last_arena_);

            if (!path1.poses.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Path generato per Robot 1: %zu waypoints", path1.poses.size());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Nessun path trovato per Robot 1");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nella generazione del path per Robot 1: %s", e.what());
            path1 = nav_msgs::msg::Path(); // Path vuoto
            path1.header.frame_id = "map";
            path1.header.stamp = this->get_clock()->now();
        }

        // ===== GENERAZIONE PATH PER ROBOT 2 =====
        RCLCPP_INFO(this->get_logger(), "Generating path for Robot 2...");
        nav_msgs::msg::Path path2;
        try
        {
            path2 = path_gen_.generate(wps_pos2, last_obstacles_, last_arena_);

            if (!path2.poses.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Path generato per Robot 2: %zu waypoints", path2.poses.size());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Nessun path trovato per Robot 2");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nella generazione del path per Robot 2: %s", e.what());
            path2 = nav_msgs::msg::Path(); // Path vuoto
            path2.header.frame_id = "map";
            path2.header.stamp = this->get_clock()->now();
        }

        // Verifica connessioni prima di pubblicare (con timeout breve)
        wait_for_subscribers(1);

        // ===== PUBBLICAZIONE PATH =====
        RCLCPP_INFO(this->get_logger(), "Publishing paths...");

        // Pubblica path per Robot 1
        publish_path_with_retry(pub_path_pos1_, path1, "path_pos1_to_gates");

        // Delay tra pubblicazioni per evitare congestione
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Pubblica path per Robot 2
        publish_path_with_retry(pub_path_pos2_, path2, "path_pos2_to_gates");

        RCLCPP_INFO(this->get_logger(), "=== Path planning completed ===");
    }

    // ===== Visualization methods =====
    void vis_points(const std::vector<std::pair<double, double>> &points)
    {
        if (points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points to visualize");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto &point = points[i];

            visualization_msgs::msg::Marker point_marker;
            point_marker.header.frame_id = "map";
            point_marker.header.stamp = this->get_clock()->now();
            point_marker.ns = "pointlist_markers";
            point_marker.id = static_cast<int>(i);
            point_marker.type = visualization_msgs::msg::Marker::SPHERE;
            point_marker.action = visualization_msgs::msg::Marker::ADD;

            // Posizione del punto
            point_marker.pose.position.x = point.first;
            point_marker.pose.position.y = point.second;
            point_marker.pose.position.z = 0.1; // Leggermente sopra il piano per visibilità
            point_marker.pose.orientation.x = 0.0;
            point_marker.pose.orientation.y = 0.0;
            point_marker.pose.orientation.z = 0.0;
            point_marker.pose.orientation.w = 1.0;

            // Scala del marker (dimensione della sfera)
            point_marker.scale.x = 0.15;
            point_marker.scale.y = 0.15;
            point_marker.scale.z = 0.15;

            // Colore del marker (rosso)
            point_marker.color.r = 1.0;
            point_marker.color.g = 0.0;
            point_marker.color.b = 0.0;
            point_marker.color.a = 0.8; // trasparenza

            // Durata del marker
            point_marker.lifetime = rclcpp::Duration::from_seconds(0); // persistente

            marker_array.markers.push_back(point_marker);
        }

        // Pubblica il marker array
        pub_points_markers->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %zu point markers for visualization",
                    marker_array.markers.size());
    }
    void vis_cells(const std::vector<planning_pkg::Cell> &cells)
    {
        if (cells.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No cells to visualize");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < cells.size(); ++i)
        {
            const auto &cell = cells[i];

            visualization_msgs::msg::Marker cell_marker;
            cell_marker.header.frame_id = "map";
            cell_marker.header.stamp = this->get_clock()->now();
            cell_marker.ns = "cells_markers";
            cell_marker.id = static_cast<int>(i);
            cell_marker.type = visualization_msgs::msg::Marker::CUBE;
            cell_marker.action = visualization_msgs::msg::Marker::ADD;

            // Posizione del centro della cella
            cell_marker.pose.position.x = cell.center_x;
            cell_marker.pose.position.y = cell.center_y;
            cell_marker.pose.position.z = 0.05; // Leggermente sopra il piano
            cell_marker.pose.orientation.x = 0.0;
            cell_marker.pose.orientation.y = 0.0;
            cell_marker.pose.orientation.z = 0.0;
            cell_marker.pose.orientation.w = 1.0;

            // Scala del marker (dimensioni della cella)
            cell_marker.scale.x = cell.width;
            cell_marker.scale.y = cell.height;
            cell_marker.scale.z = 0.1; // Altezza sottile per visualizzazione 2D

            // Colore del marker (blu trasparente)
            cell_marker.color.r = 0.0;
            cell_marker.color.g = 0.0;
            cell_marker.color.b = 1.0;
            cell_marker.color.a = 0.3; // Molto trasparente per vedere attraverso

            // Durata del marker
            cell_marker.lifetime = rclcpp::Duration::from_seconds(0); // persistente

            marker_array.markers.push_back(cell_marker);
        }

        // Pubblica il marker array
        pub_cells_markers->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %zu cell markers for visualization",
                    marker_array.markers.size());
    }
    void vis_line()
    {
        if (vertical_lines_data_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No lines to visualize");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < vertical_lines_data_.size(); ++i)
        {
            const auto &line_data = vertical_lines_data_[i];

            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = this->get_clock()->now();
            line_marker.ns = "horizontal_lines";
            line_marker.id = static_cast<int>(i);
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;

            // Posizione e orientamento
            line_marker.pose.position.x = 0.0;
            line_marker.pose.position.y = 0.0;
            line_marker.pose.position.z = 0.0;
            line_marker.pose.orientation.x = 0.0;
            line_marker.pose.orientation.y = 0.0;
            line_marker.pose.orientation.z = 0.0;
            line_marker.pose.orientation.w = 1.0;

            // Scala e colore
            line_marker.scale.x = 0.05; // spessore della linea
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0; // verde
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.8; // trasparenza

            // Punti della linea
            geometry_msgs::msg::Point start_point;
            start_point.x = std::get<2>(line_data).first;  // x_start
            start_point.y = std::get<2>(line_data).second; // y (uguale per entrambi i punti)
            start_point.z = std::get<1>(line_data);        // z coordinate

            geometry_msgs::msg::Point end_point;
            end_point.x = std::get<3>(line_data).first;  // x_end
            end_point.y = std::get<3>(line_data).second; // y (uguale per entrambi i punti)
            end_point.z = std::get<1>(line_data);        // z coordinate

            line_marker.points.push_back(start_point);
            line_marker.points.push_back(end_point);

            marker_array.markers.push_back(line_marker);
        }

        // Pubblica il marker array
        pub_vertical_line->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %zu line markers for visualization",
                    marker_array.markers.size());
    }
    void vis_arcs(const std::vector<std::vector<int>> &arc_list,
                  const std::vector<std::pair<double, double>> &points_line,
                  const std::vector<std::pair<double, double>> &points_centroids)
    {
        if (arc_list.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No arcs to visualize");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        // Combina tutti i punti in un unico vettore per l'indicizzazione
        std::vector<std::pair<double, double>> all_points;
        all_points.insert(all_points.end(), points_line.begin(), points_line.end());
        all_points.insert(all_points.end(), points_centroids.begin(), points_centroids.end());

        int marker_id = 0;

        // Per ogni nodo nel grafo
        for (size_t i = 0; i < arc_list.size(); ++i)
        {
            if (i >= all_points.size())
                continue; // Controllo di sicurezza

            const auto &connections = arc_list[i];
            const auto &start_point = all_points[i];

            // Per ogni connessione di questo nodo
            for (int connected_node : connections)
            {
                if (connected_node >= static_cast<int>(all_points.size()) || connected_node < 0)
                    continue; // Controllo di sicurezza

                const auto &end_point = all_points[connected_node];

                // Crea un marker per l'arco
                visualization_msgs::msg::Marker arc_marker;
                arc_marker.header.frame_id = "map";
                arc_marker.header.stamp = this->get_clock()->now();
                arc_marker.ns = "graph_arcs";
                arc_marker.id = marker_id++;
                arc_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                arc_marker.action = visualization_msgs::msg::Marker::ADD;

                // Posizione e orientamento
                arc_marker.pose.position.x = 0.0;
                arc_marker.pose.position.y = 0.0;
                arc_marker.pose.position.z = 0.0;
                arc_marker.pose.orientation.x = 0.0;
                arc_marker.pose.orientation.y = 0.0;
                arc_marker.pose.orientation.z = 0.0;
                arc_marker.pose.orientation.w = 1.0;

                // Stile della linea
                arc_marker.scale.x = 0.02; // Spessore della linea (più sottile delle linee orizzontali)

                // Colore diverso per i diversi tipi di connessioni
                if (i < points_line.size() && connected_node >= static_cast<int>(points_line.size()))
                {
                    // Connessione da punto a centroide (rosso)
                    arc_marker.color.r = 1.0;
                    arc_marker.color.g = 0.0;
                    arc_marker.color.b = 0.0;
                    arc_marker.color.a = 0.6;
                }
                else if (i >= points_line.size() && connected_node < static_cast<int>(points_line.size()))
                {
                    // Connessione da centroide a punto (arancione)
                    arc_marker.color.r = 1.0;
                    arc_marker.color.g = 0.5;
                    arc_marker.color.b = 0.0;
                    arc_marker.color.a = 0.6;
                }
                else
                {
                    // Altri tipi di connessioni (viola)
                    arc_marker.color.r = 0.5;
                    arc_marker.color.g = 0.0;
                    arc_marker.color.b = 1.0;
                    arc_marker.color.a = 0.6;
                }

                // Punti della linea
                geometry_msgs::msg::Point start_geom_point;
                start_geom_point.x = start_point.first;
                start_geom_point.y = start_point.second;
                start_geom_point.z = 0.05; // Leggermente sopra il piano

                geometry_msgs::msg::Point end_geom_point;
                end_geom_point.x = end_point.first;
                end_geom_point.y = end_point.second;
                end_geom_point.z = 0.05; // Leggermente sopra il piano

                arc_marker.points.push_back(start_geom_point);
                arc_marker.points.push_back(end_geom_point);

                // Durata del marker
                arc_marker.lifetime = rclcpp::Duration::from_seconds(0); // persistente

                marker_array.markers.push_back(arc_marker);
            }
        }

        // Pubblica il marker array
        pub_arcs_markers->publish(marker_array);

        RCLCPP_INFO(this->get_logger(), "Published %d arc markers for visualization", marker_id);
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_points_markers;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cells_markers;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_arcs_markers;

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