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
    // Action clients 
    client1_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino1/follow_path");
    client2_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino2/follow_path");

   
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      _path_subscription1 = this->create_subscription<nav_msgs::msg::Path>(
    "/path_1_smoothed", qos,
    [this, client = client1_ptr_](nav_msgs::msg::Path::ConstSharedPtr msg) {
      this->store_path(*msg, client, "shelfino1");
    });

    _path_subscription2 = this->create_subscription<nav_msgs::msg::Path>(
    "/path_pos2_to_gates", qos,
    [this, client = client2_ptr_](nav_msgs::msg::Path::ConstSharedPtr msg) {
      this->store_path(*msg, client, "shelfino2");
    });

    
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

  void store_path(const nav_msgs::msg::Path &msg, 
                  const rclcpp_action::Client<FollowPath>::SharedPtr &client,
                  const std::string &robot_name)
  {
    nav_msgs::msg::Path path = msg;
    // Assicura header coerente (se vuoi forzare)
    path.header.stamp = this->get_clock()->now();
    if (path.header.frame_id.empty()) path.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "Path ricevuto per %s: %zu poses. Invio...", 
                robot_name.c_str(), path.poses.size());
    send_path_goal_(client, path, robot_name);
  }

  // ===== Membri =====
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription1;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription2;

  rclcpp_action::Client<FollowPath>::SharedPtr client1_ptr_;
  rclcpp_action::Client<FollowPath>::SharedPtr client2_ptr_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
