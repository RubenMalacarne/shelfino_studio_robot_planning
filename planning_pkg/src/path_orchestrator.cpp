#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <utility>   // std::pair
#include <algorithm> // std::max

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planning_pkg/common.hpp"
#include "planning_pkg/linear_path.hpp"

using rclcpp::QoS;
using namespace std::chrono_literals;

// questo codice contiene tutto quello che riguarda per GESTIRE il plannig il planning viene letto da uno script posto in "algo_path_planning" folder

class PathPlanningOrchestratorClient : public rclcpp::Node
{
public:
    PathPlanningOrchestratorClient() : Node("PathPlanningOrchestratorClient")
    {
        
        const auto now = this->get_clock()->now();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);

        // Subscriber
        sub_obstacles_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/inflated_obstacles", qos,
                                                                                         std::bind(&PathPlanningOrchestratorClient::cb_obstacles_, this, std::placeholders::_1));
        sub_arena_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/arena_markers", qos,
                                                                                     std::bind(&PathPlanningOrchestratorClient::cb_arena_, this, std::placeholders::_1));
        sub_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/published_gates", qos,
                                                                              std::bind(&PathPlanningOrchestratorClient::cb_gates_, this, std::placeholders::_1));
        sub_pos1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos1", qos,
                                                                                             std::bind(&PathPlanningOrchestratorClient::cb_pos1_, this, std::placeholders::_1));
        sub_pos2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos2", qos,
                                                                                             std::bind(&PathPlanningOrchestratorClient::cb_pos2_, this, std::placeholders::_1));

        // Client
        client_re_mapping_trigger = this->create_client<std_srvs::srv::Trigger>("/service_trigger_inflated");

        // Publisher
        pub_path_pos1_ = this->create_publisher<nav_msgs::msg::Path>("/path_pos1_to_gates", qos);
        pub_path_pos2_ = this->create_publisher<nav_msgs::msg::Path>("/path_pos2_to_gates", qos);
        RCLCPP_INFO(this->get_logger(), "Orchestor client initialized!");

        // Timer for initialization
        init_timer_ = this->create_wall_timer(100ms, std::bind(&PathPlanningOrchestratorClient::init_service_call_, this));
        RCLCPP_INFO(this->get_logger(), "Orchestor");
    }

    // Public method to call service
    void call_service()
    {
        send_trigger_once_();
    }

private:
    // ===== Initialization timer ===== > to call the service una sola volta all'avvio (per prendere i valori iniziali di poosizione, ostacoli ..)
    void init_service_call_()
    {
        init_timer_->cancel();
        send_trigger_once_();
    }

    // ===== Service call =====
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
            RCLCPP_ERROR(this->get_logger(), "Service /service_trigger_inflated non disponibile.");
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
                        RCLCPP_WARN(this->get_logger(), "Trigger fallito: %s", resp->message.c_str());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });
    }

    // ===== Callbacks =====

    void cb_obstacles_(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        last_obstacles_ = *msg;
        got_obstacles_ = true;
        RCLCPP_INFO(this->get_logger(), "Recived obstacles inflated: %zu markers",
                    last_obstacles_.markers.size());
        path_planning();
    }

    void cb_arena_(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        last_arena_ = *msg;
        got_arena_ = true;
        RCLCPP_INFO(this->get_logger(), "Recived arena inflated: %zu markers",
                    last_arena_.markers.size());
        path_planning();
    }

    void cb_gates_(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        last_gates_ = *msg;
        got_gates_ = true;

        RCLCPP_INFO(this->get_logger(), "Recived gates: %zu", last_gates_.poses.size());
        for (size_t i = 0; i < last_gates_.poses.size(); ++i)
        {
            const auto &p = last_gates_.poses[i];
            const double yaw = yaw_from_quat_(p.orientation);
            RCLCPP_INFO(this->get_logger(), "Gate %zu -> x=%.3f, y=%.3f, yaw=%.3f",
                        i, p.position.x, p.position.y, yaw);
        }
        path_planning();
    }

    void cb_pos1_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pos1_ = *msg;
        got_pos1_ = true;

        const auto &p = last_pos1_.pose.pose;
        const double yaw = yaw_from_quat_(p.orientation);
        RCLCPP_INFO(this->get_logger(), "Recived Pos1 -> x=%.3f, y=%.3f, yaw=%.3f",
                    p.position.x, p.position.y, yaw);
        path_planning();
    }

    void cb_pos2_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pos2_ = *msg;
        got_pos2_ = true;

        const auto &p = last_pos2_.pose.pose;
        const double yaw = yaw_from_quat_(p.orientation);
        RCLCPP_INFO(this->get_logger(), "Recived Pos2 -> x=%.3f, y=%.3f, yaw=%.3f",
                    p.position.x, p.position.y, yaw);
        path_planning();
    }

    // ---------------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------------
    // methods from quaternion to RPY --> nav2 accetta un path con orientamento in RPY
    static double yaw_from_quat_(const geometry_msgs::msg::Quaternion &qmsg)
    {
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
    }

    // navy path planning (liear path)
    planning_pkg::LinearPathGenerator path_gen_;

    std::pair<double, double> find_nearest_gate(
    const geometry_msgs::msg::Point &pos,
    const geometry_msgs::msg::PoseArray &gates)
        {
            double min_dist_sq = std::numeric_limits<double>::max();
            std::pair<double, double> nearest_gate_pos;

            for (const auto &g : gates.poses)
            {
                const double dx = g.position.x - pos.x;
                const double dy = g.position.y - pos.y;
                const double dist_sq = dx * dx + dy * dy; // Distanza al quadrato per efficienza

                if (dist_sq < min_dist_sq)
                {
                    min_dist_sq = dist_sq;
                    nearest_gate_pos = {g.position.x, g.position.y};
                }
            }
            return nearest_gate_pos;
        }

    // path planning minimale (lineare, senza ostacoli/bordi)
    void path_planning()
    {
        if (!got_obstacles_ || !got_arena_ || !got_gates_ || !got_pos1_ || !got_pos2_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "data not just completed, WAIT!!...");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Inizio path planning (lineare)...");

        const auto &p1 = last_pos1_.pose.pose.position;
        const auto &p2 = last_pos2_.pose.pose.position;

        const auto nearest_gate_pos1 = find_nearest_gate(p1, last_gates_);
        RCLCPP_INFO(this->get_logger(), "Gate più vicino per Robot 1: x=%.2f, y=%.2f",nearest_gate_pos1.first, nearest_gate_pos1.second);

        const auto nearest_gate_pos2 = find_nearest_gate(p2, last_gates_);
        RCLCPP_INFO(this->get_logger(), "Gate più vicino per Robot 2: x=%.2f, y=%.2f", nearest_gate_pos2.first, nearest_gate_pos2.second);

        std::vector<std::pair<double, double>> wps_pos1;
        wps_pos1.emplace_back(p1.x, p1.y);
        wps_pos1.emplace_back(nearest_gate_pos1.first, nearest_gate_pos1.second);

        std::vector<std::pair<double, double>> wps_pos2;
        wps_pos2.emplace_back(p2.x, p2.y);
        wps_pos2.emplace_back(nearest_gate_pos2.first, nearest_gate_pos2.second);

        // Genera path con step di 0.1 m e frame "map"
        nav_msgs::msg::Path path1 = path_gen_.generate(wps_pos1, this->now(), "map", 0.1);  
        nav_msgs::msg::Path path2 = path_gen_.generate(wps_pos2, this->now(), "map", 0.1); 

        if (path1.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path pos1->gates vuoto, skip publish.");
        }
        else
        {
            pub_path_pos1_->publish(path1);
            RCLCPP_INFO(this->get_logger(), "Pubblicato path pos1->gates (%zu poses).", path1.poses.size());
        }

        if (path2.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path pos2->gates vuoto, skip publish.");
        }
        else
        {
            pub_path_pos2_->publish(path2);
            RCLCPP_INFO(this->get_logger(), "Pubblicato path pos2->gates (%zu poses).", path2.poses.size());
        }
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_re_mapping_trigger;
    rclcpp::TimerBase::SharedPtr init_timer_;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_arena_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pos2_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_pos1_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_pos2_;

    visualization_msgs::msg::MarkerArray last_obstacles_;
    visualization_msgs::msg::MarkerArray last_arena_;
    geometry_msgs::msg::PoseArray last_gates_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos1_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pos2_;

    bool got_obstacles_{false};
    bool got_arena_{false};
    bool got_gates_{false};
    bool got_pos1_{false};
    bool got_pos2_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningOrchestratorClient>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
