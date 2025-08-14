#include "planning_pkg/re_mapping.hpp"
#include "planning_pkg/common.hpp"

#include <memory>
#include "rclcpp/rclcpp.hpp"

ReMapping::ReMapping() : Node("ReMapping")
{
  
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);

  this->declare_parameter("shelfino_inflation", 0.5);
  this->declare_parameter("marker_frame", "map");
  inflation_value = get_shelfino_inflation();

  // Subscribers
  subscription_borders = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", qos, std::bind(&ReMapping::callback_borders, this, std::placeholders::_1));
  subscription_obstacles = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", qos, std::bind(&ReMapping::callback_obstacles, this, std::placeholders::_1));
  subscription_gates = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/gates", qos, std::bind(&ReMapping::callback_gates, this, std::placeholders::_1));
  subscription_position1 = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/shelfino1/amcl_pose", qos, std::bind(&ReMapping::callback_pos1, this, std::placeholders::_1));
  subscription_position2 = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/shelfino3/amcl_pose", qos, std::bind(&ReMapping::callback_pos2, this, std::placeholders::_1));
  
  // Publishers
  pub_obstacles_inflated = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("/inflated_obstacles", qos);
  pub_arena_inflated = this->create_publisher<geometry_msgs::msg::Polygon>("/inflated_arena", qos);
  pub_viz_obstacles_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/inflated_obstacles_viz", qos);
  pub_viz_arena_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/inflated_arena_viz", qos);
  pub_gates_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/published_gates", qos);
  pub_pos1_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos1", qos);
  pub_pos2_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/published_pos2", qos);

  // Service
  srv_trigger = this->create_service<std_srvs::srv::Trigger>(
    "/service_trigger_inflated",   // nome del service
    std::bind(&ReMapping::on_trigger, this, std::placeholders::_1, std::placeholders::_2)
  );
  // ros2 service call /service_trigger_inflated std_srvs/srv/Trigger {}
  }


ReMapping::~ReMapping(){}

//  ==== Callback methods ==== 
void ReMapping::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  subscription_borders.reset();
  if (!msg->points.empty()) {
    RCLCPP_INFO(this->get_logger(), "Received the arena");
    this->set_borders(*msg);
    borders_received = true;  
  }
  else{
    RCLCPP_WARN(this->get_logger(), "Received an empty arena");
  }
}

void ReMapping::callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
  subscription_obstacles.reset();
  if (!msg->obstacles.empty()) {
    RCLCPP_INFO(this->get_logger(), "Received obstacles");
    this->set_obstacles(*msg);
    obstacles_received = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Received an empty obstacle array");
  }
}

void ReMapping::callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    subscription_gates.reset();
    if (!msg->poses.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received gates");
        this->set_gates(*msg);
        gates_received = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Received an empty gate array");
    }
}

void ReMapping::callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->pose.pose.position.x != 0.0 || msg->pose.pose.position.y != 0.0) {
    RCLCPP_INFO(this->get_logger(), "Received pos2");
    subscription_position1.reset();
    //function to handle position 1
    this->set_pos1(*msg);
    pos1_r_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Received an invalid position for pos1");
  }
}

void ReMapping::callback_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if( msg->pose.pose.position.x != 0.0 || msg->pose.pose.position.y != 0.0) {
    RCLCPP_INFO(this->get_logger(), "Received pos2");
    subscription_position2.reset();
    this->set_pos2(*msg);
    //function to handle position 2
    pos2_r_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Received an invalid position for pos2");
  }
}

// ==== Service callback ====
void ReMapping::on_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string status_msg = "Published: ";
  bool success = false;
  
  if (have_obstacles_) {
    pub_obstacles_inflated->publish(last_inflated_obstacles_);
    pub_viz_obstacles_->publish(viz_obstacles_markers_);
    RCLCPP_INFO(this->get_logger(), "Published inflated obstacles.");
    status_msg += "obstacles ";
    success = true;
  }
  
  if (have_borders_) {
    pub_arena_inflated->publish(last_inflated_borders_);
    pub_viz_arena_->publish(viz_arena_markers_);
    RCLCPP_INFO(this->get_logger(), "Published inflated arena.");
    status_msg += "arena ";
    success = true;
  }
  
  // Publish gates
  if (!gates.empty()) {
    RCLCPP_INFO(this->get_logger(), "Published gates: %zu gates", gates.size());
    status_msg += "gates ";
    publish_gates();
    success = true;
  }
  //print the gates
  // if (!gates.empty()) {
  //   RCLCPP_INFO(this->get_logger(), "Gates:");
  //   for (const auto &gate : gates) {
  //     RCLCPP_INFO(this->get_logger(), "Gate: x=%.2f, y=%.2f, yaw=%.2f",
  //                 gate[0], gate[1], gate[2]);
  //   }
  //} 
  else {
    RCLCPP_WARN(this->get_logger(), "No gates received.");
    status_msg += "no-gates ";
  }
  
  if (pos1_r_) {
    publish_pos1();
    RCLCPP_INFO(this->get_logger(), "Position 1: x=%.2f, y=%.2f, yaw=%.2f",
                pos1[0], pos1[1], pos1[2]);
    status_msg += "pos1 ";
    success = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Position 1 not received.");
    status_msg += "no-pos1 ";
  }
  
  if (pos2_r_) {
    publish_pos2();
    RCLCPP_INFO(this->get_logger(), "Position 2: x=%.2f, y=%.2f, yaw=%.2f",
                pos2[0], pos2[1], pos2[2]);
    status_msg += "pos2 ";
    success = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Position 2 not received.");
    status_msg += "no-pos2 ";
  }
  
  response->success = success;
  response->message = status_msg;
}


// ==== Set methods ====
void ReMapping::set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
{
  inflated_obstacles.header = msg.header;

  for (const auto &obstacle : msg.obstacles) {
    obstacles_msgs::msg::ObstacleMsg inflated_obstacle;
    
    if (obstacle.radius == 0.0) {
      RCLCPP_INFO(this->get_logger(), "is a polygon!");
      
      inflated_obstacle.radius = 0.0;
      const auto n = obstacle.polygon.points.size();
      if (n < 2) continue;
      
      // Calculate centroid
      double cx = 0.0, cy = 0.0;
      for (const auto &p : obstacle.polygon.points) { 
        cx += p.x; 
        cy += p.y; 
      }
      cx /= static_cast<double>(n);
      cy /= static_cast<double>(n);
      
      // Inflate polygon points
      for (const auto &p : obstacle.polygon.points) {
        geometry_msgs::msg::Point32 inflated_point;
        double dx = p.x - cx, dy = p.y - cy;
        double len = std::sqrt(dx*dx + dy*dy);
        if (len > 1e-6) {
          double s = (len + inflation_value) / len; 
          inflated_point.x = cx + dx * s;
          inflated_point.y = cy + dy * s;
        } else {
          inflated_point.x = cx + inflation_value;
          inflated_point.y = cy;
        }
        inflated_point.z = p.z;
        inflated_obstacle.polygon.points.push_back(inflated_point);
      }
    }
    else {
      RCLCPP_INFO(this->get_logger(), "is a circle");
      inflated_obstacle.radius = obstacle.radius + inflation_value;
      inflated_obstacle.polygon = obstacle.polygon;
    }
    
    inflated_obstacles.obstacles.push_back(inflated_obstacle);
  }
  
  last_inflated_obstacles_ = inflated_obstacles;
  have_obstacles_ = !last_inflated_obstacles_.obstacles.empty();
  create_obstacles_markers();
  RCLCPP_INFO(this->get_logger(), "Inflated obstacles cached: %zu obstacles", last_inflated_obstacles_.obstacles.size());
}

void ReMapping::set_borders(const geometry_msgs::msg::Polygon &msg)
{
  // Handle the borders of the arena
  RCLCPP_INFO(this->get_logger(), "Setting borders");
  
  
  
  //compute centroid
  const auto &pts_in = msg.points;
  const size_t n = pts_in.size();

  double cx = 0.0, cy = 0.0;
  for (const auto &p : pts_in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);

  const double eps = 1e-6;
  for (const auto &p : pts_in) {
    geometry_msgs::msg::Point32 inflated_point;
    double dx = p.x - cx, dy = p.y - cy;
    double len = std::sqrt(dx*dx + dy*dy);
    if (len > eps) {
      // Scale towards center: (len - inflation_value)/len, clamp to > 0
      double k = std::max((len - inflation_value) / len, 0.0);
      inflated_point.x = cx + dx * k;
      inflated_point.y = cy + dy * k;
    } else {
      inflated_point.x = cx; 
      inflated_point.y = cy;
    }
    inflated_point.z = p.z;
    inflated_borders.points.push_back(inflated_point);
  }

  last_inflated_borders_ = inflated_borders;
  have_borders_ = !last_inflated_borders_.points.empty();
  create_arena_markers();
  RCLCPP_INFO(this->get_logger(), "Arena cached: %zu points", last_inflated_borders_.points.size());
}

void ReMapping::set_gates(const geometry_msgs::msg::PoseArray &msg)
{
  // Handle the gates
  RCLCPP_INFO(this->get_logger(), "Setting gates");
  
  for (const auto &pose : msg.poses)
  {
    std::vector<double> gate;
    gate.push_back(pose.position.x);
    gate.push_back(pose.position.y);
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    gate.push_back(y);
    gates.push_back(gate);
  }
}

void ReMapping::set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  RCLCPP_INFO(this->get_logger(), "Setting pos1");

  pos1.push_back(msg.pose.pose.position.x);
  pos1.push_back(msg.pose.pose.position.y);
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);
  pos1.push_back(y);
}

void ReMapping::set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  RCLCPP_INFO(this->get_logger(), "Setting pos2");

  pos2.push_back(msg.pose.pose.position.x);
  pos2.push_back(msg.pose.pose.position.y);
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);
  pos2.push_back(y);
}


// ==== Publish methods ====
void ReMapping::publish_gates()
{
  geometry_msgs::msg::PoseArray gates_msg;
  gates_msg.header.stamp = this->now();
  gates_msg.header.frame_id = get_marker_frame();
  
  for (const auto &gate : gates) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = gate[0];
    pose.position.y = gate[1];
    pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, gate[2]);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    gates_msg.poses.push_back(pose);
  }
  
  pub_gates_->publish(gates_msg);
}

void ReMapping::publish_pos1()
{
  if (pos1.size() >= 3) {
    geometry_msgs::msg::PoseWithCovarianceStamped pos1_msg;
    pos1_msg.header.stamp = this->now();
    pos1_msg.header.frame_id = get_marker_frame();
    
    pos1_msg.pose.pose.position.x = pos1[0];
    pos1_msg.pose.pose.position.y = pos1[1];
    pos1_msg.pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pos1[2]);
    pos1_msg.pose.pose.orientation.x = q.x();
    pos1_msg.pose.pose.orientation.y = q.y();
    pos1_msg.pose.pose.orientation.z = q.z();
    pos1_msg.pose.pose.orientation.w = q.w();
    
    pub_pos1_->publish(pos1_msg);
  }
}

void ReMapping::publish_pos2()
{
  if (pos2.size() >= 3) {
    geometry_msgs::msg::PoseWithCovarianceStamped pos2_msg;
    pos2_msg.header.stamp = this->now();
    pos2_msg.header.frame_id = get_marker_frame();
    
    pos2_msg.pose.pose.position.x = pos2[0];
    pos2_msg.pose.pose.position.y = pos2[1];
    pos2_msg.pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pos2[2]);
    pos2_msg.pose.pose.orientation.x = q.x();
    pos2_msg.pose.pose.orientation.y = q.y();
    pos2_msg.pose.pose.orientation.z = q.z();
    pos2_msg.pose.pose.orientation.w = q.w();
    
    pub_pos2_->publish(pos2_msg);
  }
}


// ==== Marker creation methods ====
void ReMapping::create_obstacles_markers()
{
  const std::string frame_id = this->get_parameter("marker_frame").as_string();
  rclcpp::Time now = this->now();

  viz_obstacles_markers_.markers.clear();
  int id = 0;

  for (const auto &obstacle : last_inflated_obstacles_.obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now;
    marker.ns = "inflated_obstacles";
    marker.id = id++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0f; marker.color.g = 0.6f; marker.color.b = 0.0f; marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration(0, 0);

    if (obstacle.radius == 0.0) {
      // Polygon obstacle - use LINE_STRIP
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.03;
      
      for (const auto &p : obstacle.polygon.points) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x; pt.y = p.y; pt.z = 0.0;
        marker.points.push_back(pt);
      }
      
      // Close the polygon
      if (!obstacle.polygon.points.empty()) {
        geometry_msgs::msg::Point first;
        first.x = obstacle.polygon.points.front().x;
        first.y = obstacle.polygon.points.front().y;
        first.z = 0.0;
        marker.points.push_back(first);
      }
    }
    else {
      // Circular obstacle - use CYLINDER
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      if (!obstacle.polygon.points.empty()) {
        marker.pose.position.x = obstacle.polygon.points[0].x;
        marker.pose.position.y = obstacle.polygon.points[0].y;
        marker.pose.position.z = 0.0;
      }
      marker.scale.x = 2.0 * obstacle.radius;
      marker.scale.y = 2.0 * obstacle.radius;
      marker.scale.z = 0.02;
      
    }
    
    viz_obstacles_markers_.markers.push_back(marker);
  }
}

void ReMapping::create_arena_markers()
{
  const std::string frame_id = this->get_parameter("marker_frame").as_string();
  rclcpp::Time now = this->now();

  viz_arena_markers_.markers.clear();
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = now;
  marker.ns = "inflated_arena";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.r = 1.0f; marker.color.g = 0.6f; marker.color.b = 0.0f; marker.color.a = 1.0f;
  marker.lifetime = rclcpp::Duration(0, 0);
  
  for (const auto &p : last_inflated_borders_.points) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x; pt.y = p.y; pt.z = 0.0;
    marker.points.push_back(pt);
  }
  
  // Close the polygon
  if (!last_inflated_borders_.points.empty()) {
    geometry_msgs::msg::Point first;
    first.x = last_inflated_borders_.points.front().x;
    first.y = last_inflated_borders_.points.front().y;
    first.z = 0.0;
    marker.points.push_back(first);
  }
  
  viz_arena_markers_.markers.push_back(marker);
}

// ==== Main function ====
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReMapping>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  RCLCPP_INFO(node->get_logger(), "ReMapping node started. Waiting for messages...");
  exec.spin();

  RCLCPP_INFO(node->get_logger(), "ReMapping node shutting down.");
  rclcpp::shutdown();
  return 0;
}
