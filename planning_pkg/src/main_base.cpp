#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std::chrono_literals;

static const rmw_qos_profile_t qos_profile_custom1 = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

class PathGenerator : public rclcpp::Node
{
public:
  PathGenerator() : Node("path_generator")
  {
    // Evita ParameterAlreadyDeclaredException
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", true);   // dichiara solo se manca
    }
    // Se vuoi forzarlo a true (solo se necessario):
    // this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Publisher del path (transient_local se vuoi latched-like)
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_custom1);
    // qos.transient_local();  // scommenta se vuoi che resti “latched”
    path1_publisher_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan1", qos);


    subscription_position1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/shelfino1/amcl_pose", qos, std::bind(&PathGenerator::callback_pos1, this, std::placeholders::_1));

    // Timer: pubblica ogni 5 s (oppure cancella il timer dopo la prima pubblicazione)
    // timer_ = this->create_wall_timer(5s, std::bind(&PathGenerator::publish_path_once, this));
  }

private:
  
  void callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    if (published_) return;
    current_position_ = msg->pose.pose;
    auto path1 = generate_path_for_shelfino1();
    if (path1.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Generated path is empty, skipping.");
      return;
    }
    path1_publisher_->publish(path1);
    published_ = true;
    RCLCPP_INFO(this->get_logger(), "Published path for shelfino1 (%zu poses).", path1.poses.size());

  }
  void publish_path_once()
  {
    auto path1 = generate_path_for_shelfino1();
    if (path1.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Generated path is empty, skipping.");
      return;
    }
    path1_publisher_->publish(path1);
    RCLCPP_INFO(this->get_logger(), "Published path for shelfino1 (%zu poses).", path1.poses.size());

    // Se vuoi pubblicare UNA SOLA VOLTA:
    // timer_->cancel();
  }
  nav_msgs::msg::Path generate_path_for_shelfino1()
  {
    nav_msgs::msg::Path path;
    rclcpp::Time stamp = this->get_clock()->now();
    path.header.stamp = stamp;
    path.header.frame_id = "map";

    // Lista dei waypoint principali
    std::vector<std::pair<double, double>> waypoints = {
        {current_position_.position.x, current_position_.position.y},
        {5.0, 0.0},
        {4.0, 4.0},
        {3.0, 3.0}};

    double step_size = 0.1;  // distanza tra punti consecutivi

    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
      auto [x0, y0] = waypoints[i];
      auto [x1, y1] = waypoints[i + 1];

      double dx = x1 - x0;
      double dy = y1 - y0;
      double distance = std::hypot(dx, dy);
      double steps = std::floor(distance / step_size);

      for (int s = 0; s <= steps; ++s)
      {
        double ratio = s / steps;
        double x = x0 + ratio * dx;
        double y = y0 + ratio * dy;
        double yaw = std::atan2(dy, dx);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);
      }
    }

    return path;
  }


  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_position1_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path1_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose current_position_;
  bool published_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathGenerator>();
  RCLCPP_INFO(node->get_logger(), "Path generator started. Publishing every 5s...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
