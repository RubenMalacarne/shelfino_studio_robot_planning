#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"

using FollowPath = nav2_msgs::action::FollowPath;

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher() : Node("Nav2Client")
  {
    // Action clients (nomi come in Nav2 multi-robot)
    client1_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino1/follow_path");
    client2_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino2/follow_path");

    // Sottoscrizioni ai topic path del planner (quelli pubblicati dal tuo PathPlanningOrchestratorClient)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    _path_subscription1 = this->create_subscription<nav_msgs::msg::Path>(
        "/path_pos1_to_gates", qos,
        std::bind(&PathPublisher::store_path1, this, std::placeholders::_1));

    _path_subscription2 = this->create_subscription<nav_msgs::msg::Path>(
        "/path_pos2_to_gates", qos,
        std::bind(&PathPublisher::store_path2, this, std::placeholders::_1));

    // Attendi i due action server (con timeout per non bloccare in eterno)
    if (!client1_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /shelfino1/follow_path non disponibile.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server /shelfino1/follow_path OK.");
    }

    if (!client2_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /shelfino2/follow_path non disponibile.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server /shelfino2/follow_path OK.");
    }

    RCLCPP_INFO(this->get_logger(), "Nav2Client pronto. In attesa dei path...");
  }

private:

  // ===== Helpers =====
  void send_path_goal_(const rclcpp_action::Client<FollowPath>::SharedPtr &client,
                       const nav_msgs::msg::Path &path,
                       const std::string &who)
  {
    if (!client || !client->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "[%s] action server non pronto.", who.c_str());
      return;
    }
    if (path.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "[%s] path vuoto, goal NON inviato.", who.c_str());
      return;
    }

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = "";       // usa controller di default
    goal.goal_checker_id = "";     // usa goal checker di default
    //da controllare !!!!
    rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
    opts.goal_response_callback =
        [this, who](std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowPath>> handle) {
          if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "[%s] goal REJECTED.", who.c_str());
          } else {
            RCLCPP_INFO(this->get_logger(), "[%s] goal ACCEPTED.", who.c_str());
          }
        };
    opts.result_callback =
        [this, who](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult &res) {
          switch (res.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "[%s] goal COMPLETATO.", who.c_str());
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "[%s] goal ABORTITO.", who.c_str());
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(this->get_logger(), "[%s] goal CANCELLATO.", who.c_str());
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "[%s] risultato sconosciuto.", who.c_str());
              break;
          }
        };

    client->async_send_goal(goal, opts);
  }

  // ===== Callbacks =====
  void store_path1(const nav_msgs::msg::Path &msg)
  {
    full_path1 = msg;
    // Assicura header coerente (se vuoi forzare)
    full_path1.header.stamp = this->get_clock()->now();
    if (full_path1.header.frame_id.empty()) full_path1.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "Path 1 ricevuto: %zu poses. Invio a shelfino1...", full_path1.poses.size());
    send_path_goal_(client1_ptr_, full_path1, "shelfino1");
  }

  void store_path2(const nav_msgs::msg::Path &msg)
  {
    full_path2 = msg;
    full_path2.header.stamp = this->get_clock()->now();
    if (full_path2.header.frame_id.empty()) full_path2.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "Path 2 ricevuto: %zu poses. Invio a shelfino2...", full_path2.poses.size());
    send_path_goal_(client2_ptr_, full_path2, "shelfino2");
  }

  // ===== Membri =====
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription1;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription2;

  rclcpp_action::Client<FollowPath>::SharedPtr client1_ptr_;
  rclcpp_action::Client<FollowPath>::SharedPtr client2_ptr_;

  nav_msgs::msg::Path full_path1;
  nav_msgs::msg::Path full_path2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
