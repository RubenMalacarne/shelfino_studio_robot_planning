#include "planning_pkg/common.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <string>

#include <std_srvs/srv/trigger.hpp>

namespace
{
  struct Vec2
  {
    double x{0.0}, y{0.0};
    Vec2() = default;
    Vec2(double X, double Y) : x(X), y(Y) {}
    Vec2 operator+(const Vec2 &o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2 &o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
  };
  
  inline double dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }

  inline double cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }

  inline double norm(const Vec2 &a) { return std::hypot(a.x, a.y); }
  
  inline Vec2 normalize(const Vec2 &a)
  {
    double n = norm(a);
    return (n > 0) ? (a / n) : Vec2{0, 0};
  }
  
  inline Vec2 rot90L(const Vec2 &a) { return Vec2{-a.y, a.x}; }
  
  inline Vec2 rot90R(const Vec2 &a) { return Vec2{a.y, -a.x}; }
  
  inline double angleOf(const Vec2 &d) { return std::atan2(d.y, d.x); }
  
  inline double wrapPi(double a)
  {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  }
  
  bool nearlyEqual(const Vec2 &a, const Vec2 &b, double eps)
  {
    return std::hypot(a.x - b.x, a.y - b.y) <= eps;
  }

  geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

}

class SmoothPathNode : public rclcpp::Node
{
public:
  SmoothPathNode() : rclcpp::Node("smooth_path_node")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), planning_pkg::qos::qos_profile_custom1);

    turning_radius_ = declare_parameter<double>("turning_radius", 0.50);
    sample_step_ = declare_parameter<double>("sample_step", 0.05);
    min_segment_keep_ = declare_parameter<double>("min_segment_keep", 1e-4); 
    angle_eps_ = declare_parameter<double>("angle_eps", 1e-3);               // ~0.057°
    join_eps_ = declare_parameter<double>("join_eps", 1e-6);

    // Subscribers
    sub_path_in_ = create_subscription<nav_msgs::msg::Path>(
        "/path_pos1_to_gates", qos,
        std::bind(&SmoothPathNode::pathCallback, this, std::placeholders::_1));

    sub_path_in_2_ = create_subscription<nav_msgs::msg::Path>(
        "/path_pos2_to_gates", qos,
        std::bind(&SmoothPathNode::pathCallback2, this, std::placeholders::_1));

    // Publishers
    pub_path_out_path_ = create_publisher<nav_msgs::msg::Path>("/path_1_run", qos);
    pub_path_out_path_2_ = create_publisher<nav_msgs::msg::Path>("/path_2_run", qos);

    RCLCPP_INFO(get_logger(), "Fillet smoother: turning_radius=%.3f, sample_step=%.3f",
                turning_radius_, sample_step_);

    // Trigger service
    srv_trigger = this->create_service<std_srvs::srv::Trigger>(
      "/service_trigger_smoothing_path",   
      std::bind(&SmoothPathNode::callback_trigger, this, std::placeholders::_1, std::placeholders::_2)
    );

  }

private:

  void callback_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!latest_smoothed_path_1_.poses.empty())
    {
      pub_path_out_path_->publish(latest_smoothed_path_1_);
      RCLCPP_INFO(get_logger(), "Published /path_1_smoothed via trigger with %zu poses.", latest_smoothed_path_1_.poses.size());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "No path_1 available to publish via trigger.");
    }

    if (!latest_smoothed_path_2_.poses.empty())
    {
      pub_path_out_path_2_->publish(latest_smoothed_path_2_);
      RCLCPP_INFO(get_logger(), "Published /path_2_smoothed via trigger with %zu poses.", latest_smoothed_path_2_.poses.size());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "No path_2 available to publish via trigger.");
    }

    response->success = true;
    response->message = "Paths published successfully.";
    RCLCPP_INFO(get_logger(), "Trigger service called and paths published.");
  }

  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path::SharedPtr &msg)
  {
    nav_msgs::msg::Path out_path;
    out_path.header = msg->header;
    out_path.header.stamp = now();
    const double R = turning_radius_;

    if (msg->poses.size() < 2)
    {
      RCLCPP_WARN(get_logger(), "Input path has less than 2 poses.");
      return out_path;
    }

    std::vector<Vec2> pts;
    pts.reserve(msg->poses.size());
    for (const auto &ps : msg->poses)
    {
      pts.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }

    auto push_pose = [&](const Vec2 &p, double yaw)
    {
      if (!out_path.poses.empty())
      {
        const auto &last = out_path.poses.back().pose.position;
        if (std::hypot(last.x - p.x, last.y - p.y) < join_eps_)
          return;
      }
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out_path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = yawToQuat(yaw);
      out_path.poses.push_back(ps);
    };

    {
      Vec2 p0 = pts[0];
      Vec2 p1 = pts[1];
      Vec2 d01 = p1 - p0;
      double yaw0 = angleOf(d01);
      push_pose(p0, yaw0);
    }

    for (size_t i = 1; i + 1 < pts.size(); ++i)
    {
      Vec2 Pm = pts[i - 1];
      Vec2 P = pts[i];
      Vec2 Pp = pts[i + 1];

      Vec2 u = P - Pm;
      double L1 = norm(u);
      Vec2 v = Pp - P;
      double L2 = norm(v);

      if (L1 < 1e-12 || L2 < 1e-12)
      {
        double yawP = angleOf(v.x || v.y ? v : u);
        push_pose(P, yawP);
        continue;
      }

      Vec2 uhat = u / L1;
      Vec2 vhat = v / L2;

      double cosphi = std::clamp(dot(uhat, vhat), -1.0, 1.0);
      double sinphi = cross(uhat, vhat);
      double phi = std::atan2(sinphi, cosphi);

      if (std::abs(phi) < angle_eps_)
      {
        double yawP = angleOf(vhat);
        push_pose(P, yawP);
        continue;
      }

      double d_needed = R * std::tan(std::abs(phi) / 2.0);
      double d_max = std::max(0.0, std::min(L1, L2) - min_segment_keep_);
      double d = std::min(d_needed, d_max);

      if (d <= 0.0 || !std::isfinite(d))
      {
        double yawP = angleOf(vhat);
        push_pose(P, yawP);
        continue;
      }

      Vec2 E = P - uhat * d;
      Vec2 X = P + vhat * d;

      double s = (phi > 0.0) ? 1.0 : -1.0;

      Vec2 n_u = (s > 0.0) ? rot90L(uhat) : rot90R(uhat);
      Vec2 n_v = (s > 0.0) ? rot90L(vhat) : rot90R(vhat);

      Vec2 C1 = E + n_u * R;
      Vec2 C2 = X + n_v * R;
      Vec2 C = (C1 + C2) * 0.5;

      Vec2 lastP{out_path.poses.back().pose.position.x, out_path.poses.back().pose.position.y};
      Vec2 seg = E - lastP;
      double L = norm(seg);
      if (L > join_eps_)
      {
        Vec2 dir = seg / L;
        int n = std::max(1, (int)std::ceil(L / sample_step_));
        for (int k = 1; k <= n; ++k)
        {
          double t = (double)k / (double)n;
          Vec2 p = lastP + seg * t;
          double yaw = angleOf(dir);
          push_pose(p, yaw);
        }
      }
      else
      {
        push_pose(E, angleOf(uhat));
      }

      double angE = std::atan2(E.y - C.y, E.x - C.x);
      double angX = std::atan2(X.y - C.y, X.x - C.x);

      auto angDiffCCW = [](double a0, double a1)
      {
        double d = a1 - a0;
        while (d <= 0)
          d += 2 * M_PI;
        return d;
      };
      auto angDiffCW = [](double a0, double a1)
      {
        double d = a0 - a1;
        while (d <= 0)
          d += 2 * M_PI;
        return d;
      };

      double sweep = (s > 0.0) ? angDiffCCW(angE, angX) : angDiffCW(angE, angX);
      double arc_len = R * sweep;
      int n_arc = std::max(1, (int)std::ceil(arc_len / sample_step_));

      for (int k = 1; k <= n_arc; ++k)
      {
        double tau = (double)k / (double)n_arc;
        double ang = (s > 0.0) ? angE + sweep * tau : angE - sweep * tau;
        Vec2 p = {C.x + R * std::cos(ang), C.y + R * std::sin(ang)};
        double yaw = (s > 0.0) ? (ang + M_PI_2) : (ang - M_PI_2);
        yaw = wrapPi(yaw);
        push_pose(p, yaw);
      }
    }

    {
      Vec2 last_added{out_path.poses.back().pose.position.x, out_path.poses.back().pose.position.y};
      Vec2 Pend = pts.back();
      Vec2 seg = Pend - last_added;
      double L = norm(seg);
      if (L > join_eps_)
      {
        Vec2 dir = seg / L;
        int n = std::max(1, (int)std::ceil(L / sample_step_));
        for (int k = 1; k <= n; ++k)
        {
          double t = (double)k / (double)n;
          Vec2 p = last_added + seg * t;
          double yaw = angleOf(dir);
          push_pose(p, yaw);
        }
      }
      else
      {
        double yaw = (L > 0) ? angleOf(seg) : angleOf(pts.back() - *(pts.end() - 2));
        push_pose(Pend, yaw);
      }
    }

    return out_path;
  }
  
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    auto out_path = smoothPath(msg);
    latest_smoothed_path_1_ = out_path;
    RCLCPP_INFO(get_logger(), "Smoothed path_1 ready with %zu poses.", out_path.poses.size());
  }
  
  void pathCallback2(const nav_msgs::msg::Path::SharedPtr msg)
  {
    auto out_path = smoothPath(msg);
    latest_smoothed_path_2_ = out_path;
    RCLCPP_INFO(get_logger(), "Smoothed path_2 ready with %zu poses.", out_path.poses.size());
  }
  
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_in_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_in_2_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_out_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_out_path_2_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger;
  
  nav_msgs::msg::Path latest_smoothed_path_1_;
  nav_msgs::msg::Path latest_smoothed_path_2_;
  
  double turning_radius_{0.10};
  double sample_step_{0.05};
  double min_segment_keep_{1e-4};
  double angle_eps_{1e-3};
  double join_eps_{1e-6};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmoothPathNode>());
  rclcpp::shutdown();
  return 0;
}

// to launch the service smooth and send the path:  
// ros2 service call /service_trigger_smoothing_path std_srvs/srv/Trigger {}
