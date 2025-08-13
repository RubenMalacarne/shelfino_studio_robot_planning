#include "planning_pkg/re_mapping.hpp"
#include "planning_pkg/common.hpp"


#include <memory>
#include "rclcpp/rclcpp.hpp"


ReMapping::ReMapping() : Node("ReMapping")
{
  // Define QoS profile
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);

  this->declare_parameter("shelfino_inflation", 0.5);
  this->declare_parameter("marker_frame", "map");
  inflation_value = get_shelfino_inflation();

  // Create subscribers
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


  // Create publishers
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/inflated_obstacles", 10);
  pub_arena_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/arena_markers", 10);
  // Create service
  srv_trigger = this->create_service<std_srvs::srv::Trigger>(
    "/publish_inflated",   // nome del service
    std::bind(&ReMapping::on_trigger, this, std::placeholders::_1, std::placeholders::_2)
  );
  // ros2 service call /publish_inflated std_srvs/srv/Trigger {}
  }


ReMapping::~ReMapping(){}

//  ==== callback methods ==== 
void ReMapping::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  subscription_borders.reset();
  if (!msg->points.empty()) {
    RCLCPP_INFO(this->get_logger(), "Received the arena");
    //TO DO: function to handle null arena
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
        //TO DO: function to handle gates
        this->set_gates(*msg);
        gates_received = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Received an empty gate array");
    }
}

void ReMapping::callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->pose.pose.position.x != 0.0 || msg->pose.pose.position.y != 0.0) {
    RCLCPP_INFO(this->get_logger(), "Received pos");
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
    RCLCPP_WARN(this->get_logger(), "Received pos");
    subscription_position2.reset();
    this->set_pos2(*msg);
    //function to handle position 2
    pos2_r_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Received an invalid position for pos2");
  }
}

// ==== service callback ====
void ReMapping::on_trigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (have_obstacles_) {
    pub_markers_->publish(last_marker_array_);
    RCLCPP_INFO(this->get_logger(), "Published inflated obstacles to RViz2.");
  }
  
  if (have_borders_) {
    pub_arena_->publish(last_marker_array_borders_);
    RCLCPP_INFO(this->get_logger(), "Published inflated arena to RViz2.");
  }
  else {
    response->success = false;
    response->message = "No inflated to publish.";
    RCLCPP_WARN(this->get_logger(), "No inflated obstacles to publish.");
  }
}

//  ==== get methods ==== 
pose_t ReMapping::get_pose1()
{
  return pos1;
}

pose_t ReMapping::get_pose2()
{
  return pos2;
}


// ==== set methods ====
void ReMapping::set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
{
  const std::string frame_id = this->get_parameter("marker_frame").as_string();
  rclcpp::Time now = this->now();

  visualization_msgs::msg::MarkerArray array;
  int id = 0;

  auto make_line_strip = [&](const std::vector<geometry_msgs::msg::Point> &pts, int id_local) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = now;
    m.ns = "inflated_polygons";
    m.id = id_local;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;
    m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.0f; m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration(0, 0);
    m.points = pts;
    return m;
  };

  auto make_cylinder = [&](double x, double y, double r, int id_local) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = now;
    m.ns = "inflated_circles";
    m.id = id_local;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.0;
    m.scale.x = 2.0 * r;
    m.scale.y = 2.0 * r;
    m.scale.z = 0.02;
    m.color.r = 0.1f; m.color.g = 0.7f; m.color.b = 1.0f; m.color.a = 0.6f;
    m.lifetime = rclcpp::Duration(0, 0);
    return m;
  };

  for (const auto &obstacle : msg.obstacles) {
    if (obstacle.radius == 0.0) {
      RCLCPP_INFO(this->get_logger(), "is a polygon!");
      const auto n = obstacle.polygon.points.size();
      if (n < 2) continue;
      
      // Create a copy of points to modify
      auto inflated_points = obstacle.polygon.points;
      
      double cx = 0.0, cy = 0.0;
      for (const auto &p : inflated_points) { cx += p.x; cy += p.y; }
      cx /= static_cast<double>(n);
      cy /= static_cast<double>(n);
      
      for (auto &p : inflated_points){
        //pitagora per calcolare la distanza dal centro
        double dx = p.x - cx, dy = p.y - cy;
        double len = std::sqrt(dx*dx + dy*dy);
        if (len > 1e-6) {
          double s = (len + inflation_value) / len; 
          p.x = cx + dx * s;
          p.y = cy + dy * s;
        } else {
          p.x = cx + inflation_value;
          p.y = cy;
        }
      }
      
      std::vector<geometry_msgs::msg::Point> pts;
      pts.reserve(n + 1);
      for (const auto &p : inflated_points) {
        geometry_msgs::msg::Point pt; 
        pt.x = p.x; pt.y = p.y; pt.z = 0.0; 
        pts.push_back(pt);
      }
      geometry_msgs::msg::Point first; 
      first.x = inflated_points.front().x;
      first.y = inflated_points.front().y; 
      first.z = 0.0; 
      pts.push_back(first);

      array.markers.push_back(make_line_strip(pts, id++));
    }
    else {
      RCLCPP_INFO(this->get_logger(), "is a circle");
      const double new_r = obstacle.radius + inflation_value;
      double cx, cy;
      if (!obstacle.polygon.points.empty()) {
        cx = obstacle.polygon.points[0].x;
        cy = obstacle.polygon.points[0].y;
      } else {
        cx = 0.0; cy = 0.0;
      }

      array.markers.push_back(make_cylinder(cx, cy, new_r, id++));
    }
  }
  // pub_markers_->publish(array);
  last_marker_array_ = array;
  have_obstacles_ = !last_marker_array_.markers.empty();
  RCLCPP_INFO(this->get_logger(), "Inflated obstacles cached: %zu markers", last_marker_array_.markers.size());
}

void ReMapping::set_borders(const geometry_msgs::msg::Polygon &msg)
{
  // Handle the borders of the arena
  RCLCPP_INFO(this->get_logger(), "Setting borders");
  
  const std::string frame_id = this->get_parameter("marker_frame").as_string();
  rclcpp::Time now = this->now();

  //compute centroid
  const auto &pts_in = msg.points;
  const size_t n = pts_in.size();

  double cx = 0.0, cy = 0.0;
  for (const auto &p : pts_in) { cx += p.x; cy += p.y; }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);

  std::vector<geometry_msgs::msg::Point> pts_out; pts_out.reserve(n + 1);
  const double eps = 1e-6;
  for (const auto &p : pts_in) {
    double dx = p.x - cx, dy = p.y - cy;
    double len = std::sqrt(dx*dx + dy*dy);
    geometry_msgs::msg::Point q;
    if (len > eps) {
      // scala verso il centro: (len - inflation_value)/len, clamp a > 0
      double k = std::max((len - inflation_value) / len, 0.0);
      q.x = cx + dx * k;
      q.y = cy + dy * k;
    } else {
      // se coincide col baricentro, lascia il punto lì
      q.x = cx; q.y = cy;
    }
    q.z = 0.0;
    pts_out.push_back(q);
  }
  // chiudi poligono
  pts_out.push_back(pts_out.front());

  // Marker contorno (LINE_STRIP)
  visualization_msgs::msg::Marker border;
  border.header.frame_id = frame_id;
  border.header.stamp = now;
  border.ns = "arena_inflated";
  border.id = 0;
  border.type = visualization_msgs::msg::Marker::LINE_STRIP;
  border.action = visualization_msgs::msg::Marker::ADD;
  border.pose.orientation.w = 1.0;
  border.scale.x = 0.05; // spessore linea
  border.color.r = 0.0f; border.color.g = 1.0f; border.color.b = 0.3f; border.color.a = 1.0f;
  border.lifetime = rclcpp::Duration(0, 0);
  border.points = pts_out;

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(border);
  // pub_arena_->publish(arr);

  last_marker_array_borders_ = arr;
  have_borders_ = !last_marker_array_borders_.markers.empty();
  RCLCPP_INFO(this->get_logger(), "Arena cached: %zu markers",
              last_marker_array_borders_.markers.size());

}

void ReMapping::set_gates(const geometry_msgs::msg::PoseArray &msg)
{
  // Handle the gates
  RCLCPP_INFO(this->get_logger(), "Setting gates");
  // TO DO: Implement logic to handle gates
}

void ReMapping::set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  RCLCPP_INFO(this->get_logger(), "Setting pos1");
}

void ReMapping::set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  RCLCPP_INFO(this->get_logger(), "Setting pos2");
}

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
