#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

using FollowPath = nav2_msgs::action::FollowPath;
class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher() : Node("Nav2Client")
  {
    client1_ptr_ =
        rclcpp_action::create_client<FollowPath>(this, "shelfino1/follow_path");

    _path_subscription1 = this->create_subscription<nav_msgs::msg::Path>(
        "shelfino1/plan1", 10,
        std::bind(&PathPublisher::store_path1, this, std::placeholders::_1));

    client2_ptr_ = rclcpp_action::create_client<FollowPath>(this, "shelfino2/follow_path");

    _path_subscription2 = this->create_subscription<nav_msgs::msg::Path>(
        "shelfino2/plan1", 10,
        std::bind(&PathPublisher::store_path2, this, std::placeholders::_1));

    if (!this->client1_ptr_->wait_for_action_server() && !this->client2_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }
    std::cout << "Action clients ready" << std::endl;
  }
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription1;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription2;

  rclcpp_action::Client<FollowPath>::SharedPtr client1_ptr_;
  rclcpp_action::Client<FollowPath>::SharedPtr client2_ptr_;

  nav_msgs::msg::Path full_path1;
  nav_msgs::msg::Path full_path2;

  bool waypoints1_received = false;
  bool waypoints2_received = false;

  void store_path1(const nav_msgs::msg::Path &msg)
  {
    if (waypoints1_received)
    {
      return;
    }
    full_path1 = msg;
    std::cout << "Path 1 stored!" << std::endl;
    waypoints1_received = true;
    return;
  }
  void store_path2(const nav_msgs::msg::Path &msg)
  {
    if (waypoints2_received)
    {
      return;
    }
    full_path2 = msg;
    std::cout << "Path 2 stored!" << std::endl;
    waypoints2_received = true;
    return;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  while (rclcpp::ok())
  {
    if (node->waypoints1_received)
    // if (node->waypoints1_received && node->waypoints2_received)
    {
      std::cout << "\033[1;32mSending paths to controllers\033[0m" << std::endl;
      // nav_msgs::msg::Path full_path1;
      node->full_path1.header.stamp = node->get_clock()->now();
      node->full_path1.header.frame_id = "map";

      // node->full_path2.header.stamp = node->get_clock()->now();
      // node->full_path2.header.frame_id = "map";

      auto goal1_msg = FollowPath::Goal();
      // auto goal2_msg = FollowPath::Goal();
      goal1_msg.path = node->full_path1;
      // goal2_msg.path = node->full_path2;
      goal1_msg.controller_id = "FollowPath";
      // goal2_msg.controller_id = "FollowPath";
      node->client1_ptr_->async_send_goal(goal1_msg);
      // node->client2_ptr_->async_send_goal(goal2_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
      // ensure that the path is sent
      node->client1_ptr_->async_send_goal(goal1_msg);
      // node->client2_ptr_->async_send_goal(goal2_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
      // ensure that the path is sent
      node->client1_ptr_->async_send_goal(goal1_msg);
      // node->client2_ptr_->async_send_goal(goal2_msg);
      node->waypoints1_received = false;
      // node->waypoints2_received = false;
    }
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}