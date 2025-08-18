// fillet_path_node.cpp
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

namespace {
struct Vec2 {
  double x{0.0}, y{0.0};
  Vec2() = default;
  Vec2(double X, double Y): x(X), y(Y) {}
  Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
  Vec2 operator*(double s) const { return {x*s, y*s}; }
  Vec2 operator/(double s) const { return {x/s, y/s}; }
};

inline double dot(const Vec2& a, const Vec2& b){ return a.x*b.x + a.y*b.y; }
inline double cross(const Vec2& a, const Vec2& b){ return a.x*b.y - a.y*b.x; }
inline double norm(const Vec2& a){ return std::hypot(a.x, a.y); }
inline Vec2 normalize(const Vec2& a){ double n = norm(a); return (n>0)? (a/n) : Vec2{0,0}; }
inline Vec2 rot90L(const Vec2& a){ return Vec2{-a.y, a.x}; }  // rotazione +90°
inline Vec2 rot90R(const Vec2& a){ return Vec2{ a.y,-a.x}; }  // rotazione -90°

inline double angleOf(const Vec2& d){ return std::atan2(d.y, d.x); }

inline double wrapPi(double a){
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

bool nearlyEqual(const Vec2& a, const Vec2& b, double eps){
  return std::hypot(a.x-b.x, a.y-b.y) <= eps;
}

} // namespace

class FilletPathNode : public rclcpp::Node {
public:
  FilletPathNode() : rclcpp::Node("fillet_path_node")
  {
    turning_radius_ = declare_parameter<double>("turning_radius", 0.50);
    sample_step_    = declare_parameter<double>("sample_step", 0.05);
    min_segment_keep_ = declare_parameter<double>("min_segment_keep", 1e-4); // margine anti-degenerazione
    angle_eps_ = declare_parameter<double>("angle_eps", 1e-3);               // ~0.057°
    join_eps_  = declare_parameter<double>("join_eps", 1e-6);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::TransientLocal)
      .history(rclcpp::HistoryPolicy::KeepLast);

    pub_path_out_ = create_publisher<nav_msgs::msg::Path>("/path_1_smoothed", qos);
    pub_path_out_2_ = create_publisher<nav_msgs::msg::Path>("/path_2_smoothed", qos);

    sub_path_in_ = create_subscription<nav_msgs::msg::Path>(
      "/path_pos1_to_gates", qos,
      std::bind(&FilletPathNode::pathCallback, this, std::placeholders::_1));

    sub_path_in_2_ = create_subscription<nav_msgs::msg::Path>(
      "/path_pos2_to_gates", qos,
      std::bind(&FilletPathNode::pathCallback2, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Fillet smoother: turning_radius=%.3f, sample_step=%.3f",
                turning_radius_, sample_step_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Input path has less than 2 poses.");
      return;
    }

    // Estrai i punti 2D
    std::vector<Vec2> pts;
    pts.reserve(msg->poses.size());
    for (const auto& ps : msg->poses) {
      pts.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }

    nav_msgs::msg::Path out;
    out.header = msg->header;
    out.header.stamp = now();

    // Helper per push con deduplica
    auto push_pose = [&](const Vec2& p, double yaw){
      if (!out.poses.empty()) {
        const auto& last = out.poses.back().pose.position;
        if (std::hypot(last.x - p.x, last.y - p.y) < join_eps_) return;
      }
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = yawToQuat(yaw);
      out.poses.push_back(ps);
    };

    // Inizio: aggiungi il primo punto (orientazione = direzione al prossimo)
    {
      Vec2 p0 = pts[0];
      Vec2 p1 = pts[1];
      Vec2 d01 = p1 - p0;
      double yaw0 = angleOf(d01);
      push_pose(p0, yaw0);
    }

    const double R = turning_radius_;

    // Scorri le triple per inserire raccordi
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      Vec2 Pm = pts[i-1];
      Vec2 P  = pts[i];
      Vec2 Pp = pts[i+1];

      Vec2 u = P - Pm;  double L1 = norm(u);
      Vec2 v = Pp - P;  double L2 = norm(v);

      if (L1 < 1e-12 || L2 < 1e-12) {
        // Segmento degenerato, salta arco e vai dritto
        // Aggiungi direttamente P con yaw inferito dal tratto successivo
        double yawP = angleOf(v.x||v.y ? v : u);
        push_pose(P, yawP);
        continue;
      }

      Vec2 uhat = u / L1;
      Vec2 vhat = v / L2;

      // Angolo di svolta con segno (phi > 0 = svolta a sinistra)
      double cosphi = std::clamp(dot(uhat, vhat), -1.0, 1.0);
      double sinphi = cross(uhat, vhat); // segno della svolta
      double phi = std::atan2(sinphi, cosphi);

      // Se quasi collineare, niente arco: mantieni il vertice (smusseremo con sampling)
      if (std::abs(phi) < angle_eps_) {
        // congiungi Pm->P e P->Pp con tratto rettilineo continuo
        // per evitare sovracampionamento, aggiungiamo un solo punto al vertice
        double yawP = angleOf(vhat);
        push_pose(P, yawP);
        continue;
      }

      // Distanza di taglio d = R * tan(|phi|/2)
      double d_needed = R * std::tan(std::abs(phi) / 2.0);

      // Limita d se i segmenti sono corti
      double d_max = std::max(0.0, std::min(L1, L2) - min_segment_keep_);
      double d = std::min(d_needed, d_max);

      // Se non c'è spazio per l'arco, niente raccordo
      if (d <= 0.0 || !std::isfinite(d)) {
        double yawP = angleOf(vhat);
        push_pose(P, yawP);
        continue;
      }

      // Punti di tangenza (entry/exit) sul taglio
      Vec2 E = P - uhat * d; // punto sul primo segmento prima del vertice
      Vec2 X = P + vhat * d; // punto sul secondo segmento dopo il vertice

      // Verso della svolta: s = +1 sinistra, -1 destra
      double s = (phi > 0.0) ? 1.0 : -1.0;

      // Normali verso il centro dal lato interno della curva
      Vec2 n_u = (s > 0.0) ? rot90L(uhat) : rot90R(uhat);
      Vec2 n_v = (s > 0.0) ? rot90L(vhat) : rot90R(vhat);

      // Centri attesi dalle due tangenti (coincidono idealmente)
      Vec2 C1 = E + n_u * R;
      Vec2 C2 = X + n_v * R;
      Vec2 C  = (C1 + C2) * 0.5;

      // ---- Campiona il tratto rettilineo P_prev_out -> E ----
      // direzione lungo uhat dal LAST punto aggiunto (che dovrebbe già essere sul ramo precedente)
      {
        Vec2 lastP{out.poses.back().pose.position.x, out.poses.back().pose.position.y};
        Vec2 seg = E - lastP;
        double L = norm(seg);
        if (L > join_eps_) {
          Vec2 dir = (L>0)? seg/L : uhat;
          int n = std::max(1, (int)std::ceil(L / sample_step_));
          for (int k=1; k<=n; ++k) {
            double t = (double)k / (double)n; // (0,1]
            Vec2 p = lastP + seg * t;
            double yaw = angleOf(dir);
            push_pose(p, yaw);
          }
        } else {
          // se praticamente già su E, forziamo un punto su E con yaw uhat
          push_pose(E, angleOf(uhat));
        }
      }

      // ---- Campiona l'arco E -> X (raggio R, centro C) ----
      double angE = std::atan2(E.y - C.y, E.x - C.x);
      double angX = std::atan2(X.y - C.y, X.x - C.x);

      auto angDiffCCW = [](double a0, double a1){
        double d = a1 - a0;
        while (d <= 0) d += 2*M_PI;
        return d;
      };
      auto angDiffCW = [](double a0, double a1){
        double d = a0 - a1;
        while (d <= 0) d += 2*M_PI;
        return d;
      };

      double sweep = (s > 0.0) ? angDiffCCW(angE, angX) : angDiffCW(angE, angX);
      double arc_len = R * sweep;
      int n_arc = std::max(1, (int)std::ceil(arc_len / sample_step_));

      for (int k=1; k<=n_arc; ++k) {
        double tau = (double)k / (double)n_arc; // (0,1]
        double ang;
        if (s > 0.0) {
          ang = angE + sweep * tau; // CCW
        } else {
          ang = angE - sweep * tau; // CW
        }
        // punto sull'arco
        Vec2 p = { C.x + R * std::cos(ang), C.y + R * std::sin(ang) };
        // tangente: per sinistra yaw = ang + pi/2, per destra yaw = ang - pi/2
        double yaw = (s > 0.0) ? (ang + M_PI_2) : (ang - M_PI_2);
        yaw = wrapPi(yaw);
        push_pose(p, yaw);
      }

      // Nota: non aggiungiamo qui X esplicitamente perché è già incluso nel loop (k==n_arc).
      // Il prossimo ciclo partirà da X verso il prossimo raccordo.
    }

    // Chiudi con l'ultimo tratto rettilineo fino all'ultimo punto
    {
      Vec2 last_added{out.poses.back().pose.position.x, out.poses.back().pose.position.y};
      Vec2 Pend = pts.back();
      Vec2 seg = Pend - last_added;
      double L = norm(seg);
      if (L > join_eps_) {
        Vec2 dir = seg / L;
        int n = std::max(1, (int)std::ceil(L / sample_step_));
        for (int k=1; k<=n; ++k) {
          double t = (double)k / (double)n;
          Vec2 p = last_added + seg * t;
          double yaw = angleOf(dir);
          push_pose(p, yaw);
        }
      } else {
        // giusto per garantire l'ultimo punto
        double yaw = (L>0)? angleOf(seg) : angleOf(pts.back() - *(pts.end()-2));
        push_pose(Pend, yaw);
      }
    }

    pub_path_out_->publish(out);
    RCLCPP_INFO(get_logger(), "Published /path_1_smoothed with %zu poses.", out.poses.size());
  }

  void pathCallback2(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Input path 2 has less than 2 poses.");
      return;
    }

    // Estrai i punti 2D
    std::vector<Vec2> pts;
    pts.reserve(msg->poses.size());
    for (const auto& ps : msg->poses) {
      pts.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }

    nav_msgs::msg::Path out;
    out.header = msg->header;
    out.header.stamp = now();

    // Helper per push con deduplica
    auto push_pose = [&](const Vec2& p, double yaw){
      if (!out.poses.empty()) {
        const auto& last = out.poses.back().pose.position;
        if (std::hypot(last.x - p.x, last.y - p.y) < join_eps_) return;
      }
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = yawToQuat(yaw);
      out.poses.push_back(ps);
    };

    // Inizio: aggiungi il primo punto (orientazione = direzione al prossimo)
    {
      Vec2 p0 = pts[0];
      Vec2 p1 = pts[1];
      Vec2 d01 = p1 - p0;
      double yaw0 = angleOf(d01);
      push_pose(p0, yaw0);
    }

    const double R = turning_radius_;

    // Scorri le triple per inserire raccordi
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      Vec2 Pm = pts[i-1];
      Vec2 P  = pts[i];
      Vec2 Pp = pts[i+1];

      Vec2 u = P - Pm;  double L1 = norm(u);
      Vec2 v = Pp - P;  double L2 = norm(v);

      if (L1 < 1e-12 || L2 < 1e-12) {
        // Segmento degenerato, salta arco e vai dritto
        // Aggiungi direttamente P con yaw inferito dal tratto successivo
        double yawP = angleOf(v.x||v.y ? v : u);
        push_pose(P, yawP);
        continue;
      }

      Vec2 uhat = u / L1;
      Vec2 vhat = v / L2;

      // Angolo di svolta con segno (phi > 0 = svolta a sinistra)
      double cosphi = std::clamp(dot(uhat, vhat), -1.0, 1.0);
      double sinphi = cross(uhat, vhat); // segno della svolta
      double phi = std::atan2(sinphi, cosphi);

      // Se quasi collineare, niente arco: mantieni il vertice (smusseremo con sampling)
      if (std::abs(phi) < angle_eps_) {
        // congiungi Pm->P e P->Pp con tratto rettilineo continuo
        // per evitare sovracampionamento, aggiungiamo un solo punto al vertice
        double yawP = angleOf(vhat);
        push_pose(P, yawP);
        continue;
      }

      // Distanza di taglio d = R * tan(|phi|/2)
      double d_needed = R * std::tan(std::abs(phi) / 2.0);

      // Limita d se i segmenti sono corti
      double d_max = std::max(0.0, std::min(L1, L2) - min_segment_keep_);
      double d = std::min(d_needed, d_max);

      // Se non c'è spazio per l'arco, niente raccordo
      if (d <= 0.0 || !std::isfinite(d)) {
        double yawP = angleOf(vhat);
        push_pose(P, yawP);
        continue;
      }

      // Punti di tangenza (entry/exit) sul taglio
      Vec2 E = P - uhat * d; // punto sul primo segmento prima del vertice
      Vec2 X = P + vhat * d; // punto sul secondo segmento dopo il vertice

      // Verso della svolta: s = +1 sinistra, -1 destra
      double s = (phi > 0.0) ? 1.0 : -1.0;

      // Normali verso il centro dal lato interno della curva
      Vec2 n_u = (s > 0.0) ? rot90L(uhat) : rot90R(uhat);
      Vec2 n_v = (s > 0.0) ? rot90L(vhat) : rot90R(vhat);

      // Centri attesi dalle due tangenti (coincidono idealmente)
      Vec2 C1 = E + n_u * R;
      Vec2 C2 = X + n_v * R;
      Vec2 C  = (C1 + C2) * 0.5;

      // ---- Campiona il tratto rettilineo P_prev_out -> E ----
      // direzione lungo uhat dal LAST punto aggiunto (che dovrebbe già essere sul ramo precedente)
      {
        Vec2 lastP{out.poses.back().pose.position.x, out.poses.back().pose.position.y};
        Vec2 seg = E - lastP;
        double L = norm(seg);
        if (L > join_eps_) {
          Vec2 dir = (L>0)? seg/L : uhat;
          int n = std::max(1, (int)std::ceil(L / sample_step_));
          for (int k=1; k<=n; ++k) {
            double t = (double)k / (double)n; // (0,1]
            Vec2 p = lastP + seg * t;
            double yaw = angleOf(dir);
            push_pose(p, yaw);
          }
        } else {
          // se praticamente già su E, forziamo un punto su E con yaw uhat
          push_pose(E, angleOf(uhat));
        }
      }

      // ---- Campiona l'arco E -> X (raggio R, centro C) ----
      double angE = std::atan2(E.y - C.y, E.x - C.x);
      double angX = std::atan2(X.y - C.y, X.x - C.x);

      auto angDiffCCW = [](double a0, double a1){
        double d = a1 - a0;
        while (d <= 0) d += 2*M_PI;
        return d;
      };
      auto angDiffCW = [](double a0, double a1){
        double d = a0 - a1;
        while (d <= 0) d += 2*M_PI;
        return d;
      };

      double sweep = (s > 0.0) ? angDiffCCW(angE, angX) : angDiffCW(angE, angX);
      double arc_len = R * sweep;
      int n_arc = std::max(1, (int)std::ceil(arc_len / sample_step_));

      for (int k=1; k<=n_arc; ++k) {
        double tau = (double)k / (double)n_arc; // (0,1]
        double ang;
        if (s > 0.0) {
          ang = angE + sweep * tau; // CCW
        } else {
          ang = angE - sweep * tau; // CW
        }
        // punto sull'arco
        Vec2 p = { C.x + R * std::cos(ang), C.y + R * std::sin(ang) };
        // tangente: per sinistra yaw = ang + pi/2, per destra yaw = ang - pi/2
        double yaw = (s > 0.0) ? (ang + M_PI_2) : (ang - M_PI_2);
        yaw = wrapPi(yaw);
        push_pose(p, yaw);
      }

      // Nota: non aggiungiamo qui X esplicitamente perché è già incluso nel loop (k==n_arc).
      // Il prossimo ciclo partirà da X verso il prossimo raccordo.
    }

    // Chiudi con l'ultimo tratto rettilineo fino all'ultimo punto
    {
      Vec2 last_added{out.poses.back().pose.position.x, out.poses.back().pose.position.y};
      Vec2 Pend = pts.back();
      Vec2 seg = Pend - last_added;
      double L = norm(seg);
      if (L > join_eps_) {
        Vec2 dir = seg / L;
        int n = std::max(1, (int)std::ceil(L / sample_step_));
        for (int k=1; k<=n; ++k) {
          double t = (double)k / (double)n;
          Vec2 p = last_added + seg * t;
          double yaw = angleOf(dir);
          push_pose(p, yaw);
        }
      } else {
        // giusto per garantire l'ultimo punto
        double yaw = (L>0)? angleOf(seg) : angleOf(pts.back() - *(pts.end()-2));
        push_pose(Pend, yaw);
      }
    }

    pub_path_out_2_->publish(out);
    RCLCPP_INFO(get_logger(), "Published /path_2_smoothed with %zu poses.", out.poses.size());
  }

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_in_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_in_2_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    pub_path_out_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    pub_path_out_2_;

  // Params
  double turning_radius_{0.10};
  double sample_step_{0.05};
  double min_segment_keep_{1e-4};
  double angle_eps_{1e-3};
  double join_eps_{1e-6};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilletPathNode>());
  rclcpp::shutdown();
  return 0;
}
