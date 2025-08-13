#include "planning_pkg/common.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <obstacles_msgs/msg/obstacle_array_msg.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <std_srvs/srv/trigger.hpp>

// Type definitions
using pose_t = std::vector<double>;

class ReMapping: public rclcpp::Node
{
public:

    ReMapping();
    ~ReMapping();

    //flag
    bool borders_received = false;
    bool obstacles_received = false;
    bool gates_received = false;
    bool position1_received = false;
    bool position2_received = false;

    bool all_map_regenerated = false;

    // Position flags
    bool pos1_r_ = false;
    bool pos2_r_ = false;
    bool have_obstacles_ = false;
    bool have_borders_ = false;

    //cuscino
    double get_shelfino_inflation() const {
        return this->get_parameter("shelfino_inflation").as_double();
    }
    double inflation_value;
    std::string get_marker_frame() const {
        return this->get_parameter("marker_frame").as_string();
    }

      

private:
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_position1;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_position2;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger;
  visualization_msgs::msg::MarkerArray last_marker_array_; 
  visualization_msgs::msg::MarkerArray last_marker_array_borders_; 

  // Publishers 
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_arena_;  
  // Position storage
  pose_t pos1;
  pose_t pos2;

  // Callbacks
  void callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg);
  void callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
  void callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callback_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  // Service callback
  void on_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res); 
  // Get
  pose_t get_pose1();
  pose_t get_pose2();

  // Set
  void set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  void set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
  void set_borders(const geometry_msgs::msg::Polygon &msg);
  void set_gates(const geometry_msgs::msg::PoseArray &msg);
  // Map values
  std::vector<pose_t> gates;
  
  bool is_map_created = false;

};