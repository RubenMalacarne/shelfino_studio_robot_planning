#include "planning_pkg/prm_path.hpp"
#include "planning_pkg/common_function.hpp"

#include <algorithm>   // std::min, std::max, std::clamp
#include <cmath>       // std::sqrt
#include <limits>      // std::numeric_limits
#include <random>      // random_device, mt19937, uniform_real_distribution
#include <utility>     // std::move
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>


// ============================================================================
// planning_pkg::PrmPathGenerator
// ============================================================================
namespace planning_pkg
{

PrmPathGenerator::PrmPathGenerator(std::string default_frame_id,
                                   double default_step_size)
: default_frame_id_(std::move(default_frame_id)),
  default_step_size_(default_step_size)
{}

nav_msgs::msg::Path PrmPathGenerator::generate(
  const std::vector<std::pair<double,double>>& waypoints,
  const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
  const geometry_msgs::msg::Polygon& arena) const

{
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = default_frame_id_;

  if (waypoints.size() < 2) {
    return path;
  }
  const auto start = waypoints.front();
  const auto goal = waypoints.back();

  std::vector<geometry_msgs::msg::Point> temp_points = random_points;
  std::vector<std::vector<int>> temp_adj = knn_adj;
  
  // Aggiungi start point
  geometry_msgs::msg::Point start_point;
  start_point.x = start.first;
  start_point.y = start.second;
  start_point.z = 0.0;
  
  // Verifica che start sia valido
  if (!CommonFunction::point_is_valid(start.first, start.second, arena, obstacles, 0.15)) {
    RCLCPP_ERROR(rclcpp::get_logger("prm_path_generator"), 
                 "Punto di start non valido");
    return path;
  }
  
  temp_points.push_back(start_point);
  temp_adj.resize(temp_points.size());
  int start_node = static_cast<int>(temp_points.size() - 1);
  
  // Aggiungi goal point
  geometry_msgs::msg::Point goal_point;
  goal_point.x = goal.first;
  goal_point.y = goal.second;
  goal_point.z = 0.0;
  
  // Verifica che goal sia valido
  if (!CommonFunction::point_is_valid(goal.first, goal.second, arena, obstacles, 0.15)) {
    RCLCPP_ERROR(rclcpp::get_logger("prm_path_generator"), 
                 "Punto di goal non valido");
    return path;
  }
  
  temp_points.push_back(goal_point);
  temp_adj.resize(temp_points.size());
  int goal_node = static_cast<int>(temp_points.size() - 1);
  
  // Connetti start e goal ai nodi vicini validi
  const double max_connection_distance = 2.0; // massima distanza per connessioni
  const size_t max_connections = 10; // massimo numero di connessioni
  
  // Connetti start
  std::vector<std::pair<double, int>> start_distances;
  for (size_t i = 0; i < random_points.size(); ++i) {
    double dist = std::sqrt(CommonFunction::dist2(start_point, random_points[i]));
    if (dist <= max_connection_distance) {
      start_distances.emplace_back(dist, static_cast<int>(i));
    }
  }
  
  std::sort(start_distances.begin(), start_distances.end());
  size_t start_connections = std::min(max_connections, start_distances.size());
  
  for (size_t i = 0; i < start_connections; ++i) {
    int neighbor = start_distances[i].second;
    if (CommonFunction::segment_is_valid(start_point, random_points[neighbor], arena, obstacles, 0.15, 0.05)) {
      temp_adj[start_node].push_back(neighbor);
      temp_adj[neighbor].push_back(start_node);
    }
  }
  
  // Connetti goal
  std::vector<std::pair<double, int>> goal_distances;
  for (size_t i = 0; i < random_points.size(); ++i) {
    double dist = std::sqrt(CommonFunction::dist2(goal_point, random_points[i]));
    if (dist <= max_connection_distance) {
      goal_distances.emplace_back(dist, static_cast<int>(i));
    }
  }
  
  std::sort(goal_distances.begin(), goal_distances.end());
  size_t goal_connections = std::min(max_connections, goal_distances.size());
  
  for (size_t i = 0; i < goal_connections; ++i) {
    int neighbor = goal_distances[i].second;
    if (CommonFunction::segment_is_valid(goal_point, random_points[neighbor], arena, obstacles, 0.15, 0.05)) {
      temp_adj[goal_node].push_back(neighbor);
      temp_adj[neighbor].push_back(goal_node);
    }
  }
  
  // Verifica connessione diretta start-goal
  if (CommonFunction::segment_is_valid(start_point, goal_point, arena, obstacles, 0.15, 0.05)) {
    temp_adj[start_node].push_back(goal_node);
    temp_adj[goal_node].push_back(start_node);
  }

  // Esegui Dijkstra con il grafo temporaneo
  std::vector<int> path_indices = dijkstra_shortest_path_temp(start_node, goal_node, temp_points, temp_adj);
  
  if (path_indices.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("prm_path_generator"), 
                 "Nessun path trovato nel grafo PRM");
    return path;
  }

  // Converti gli indici in punti
  std::vector<geometry_msgs::msg::Point> original_path_points;
  for (int idx : path_indices) {
    if (idx >= 0 && static_cast<size_t>(idx) < temp_points.size()) {
      original_path_points.push_back(temp_points[idx]);
    }
  }

  std::vector<geometry_msgs::msg::Point> optimized_path_points = 
      CommonFunction::optimize_path_with_raycasting(original_path_points, obstacles, arena, 0.15, 0.05);

  for (const auto& point : optimized_path_points) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path.header;
      
      pose_stamped.pose.position = point;
      pose_stamped.pose.orientation.w = 1.0;
      
      path.poses.push_back(pose_stamped);
  }

  return path;
}

// Versione temporanea di Dijkstra che usa i punti e adiacenze temporanei
std::vector<int> PrmPathGenerator::dijkstra_shortest_path_temp(
  int start, int goal, 
  const std::vector<geometry_msgs::msg::Point>& points,
  const std::vector<std::vector<int>>& adjacency) const {
  
  if (start == goal) return {start};
  
  const size_t N = points.size();
  if (start < 0 || goal < 0 || static_cast<size_t>(start) >= N || static_cast<size_t>(goal) >= N) {
    return {};
  }

  std::vector<double> dist(N, std::numeric_limits<double>::infinity());
  std::vector<int> prev(N, -1);
  std::vector<bool> visited(N, false);
  
  dist[start] = 0.0;
  
  for (size_t count = 0; count < N; ++count) {
    int u = -1;
    for (size_t v = 0; v < N; ++v) {
      if (!visited[v] && (u == -1 || dist[v] < dist[u])) {
        u = static_cast<int>(v);
      }
    }
    
    if (u == -1 || dist[u] == std::numeric_limits<double>::infinity()) break;
    
    visited[u] = true;
    
    if (u == goal) break;
    
    for (int v : adjacency[u]) {
      if (!visited[v]) {
        double weight = std::sqrt(CommonFunction::dist2(points[u], points[v]));
        double alt = dist[u] + weight;
        if (alt < dist[v]) {
          dist[v] = alt;
          prev[v] = u;
        }
      }
    }
  }
  
  // Ricostruisci il path
  std::vector<int> path;
  for (int at = goal; at != -1; at = prev[at]) {
    path.push_back(at);
  }
  
  if (path.empty() || path.back() != start) {
    return {}; // Nessun path trovato
  }
  
  std::reverse(path.begin(), path.end());
  return path;
}

void PrmPathGenerator::sample_random_points(
  const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
  const geometry_msgs::msg::Polygon& arena,
  std::size_t count)
{
  random_points.clear();
  if (count == 0 || arena.points.empty()) {
    return;
  }

  double minx, miny, maxx, maxy;
  CommonFunction::compute_bbox(arena, minx, miny, maxx, maxy);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist_x(minx, maxx);
  std::uniform_real_distribution<double> dist_y(miny, maxy);

  const double clearance = 0.15; // m
  const std::size_t max_attempts = std::max<std::size_t>(count * 200, 5000);

  std::size_t attempts = 0;
  while (random_points.size() < count && attempts < max_attempts) {
    ++attempts;
    const double x = dist_x(gen);
    const double y = dist_y(gen);

    if (!CommonFunction::point_is_valid(x, y, arena, obstacles, clearance)) {
      continue;
    }

    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.0;
    random_points.push_back(std::move(p));
  }

  if (random_points.size() < count) {
    RCLCPP_WARN(rclcpp::get_logger("prm_path_generator"),
                "Richiesti %zu campioni, generati %zu (clearance %.2f m).",
                count, random_points.size(), clearance);
  }
}


void planning_pkg::PrmPathGenerator::build_knn_edges(
  const obstacles_msgs::msg::ObstacleArrayMsg& obstacles,
  const geometry_msgs::msg::Polygon& arena,
  std::size_t k_neighbors,
  double clearance,
  double sample_step)
{
  knn_adj.clear();

  const std::size_t N = random_points.size();
  knn_adj.resize(N);

  if (N < 2 || k_neighbors == 0) {
    return;
  }

  // Per ogni punto, trova i k vicini più prossimi (euclidei) e tieni l’arco se collision-free.
  // Nota: uso nth_element per evitare ordini completi inutili.
  std::vector<std::pair<double, int>> dists;
  dists.reserve(N - 1);

  for (std::size_t i = 0; i < N; ++i) {
    dists.clear();
    dists.reserve(N - 1);

    for (std::size_t j = 0; j < N; ++j) {
      if (i == j) continue;
      dists.emplace_back(CommonFunction::dist2(random_points[i], random_points[j]), static_cast<int>(j));
    }

    // seleziona i k vicini con distanza minore
    const std::size_t take = std::min(k_neighbors, dists.size());
    std::nth_element(dists.begin(), dists.begin() + take, dists.end(),
                     [](const auto& a, const auto& b){ return a.first < b.first; });

    for (std::size_t t = 0; t < take; ++t) {
      const int j = dists[t].second;

      // Per evitare duplicati, potresti imporre i<j qui e poi spingere su entrambi i lati.
      // Ma per semplicità controlliamo e aggiungiamo entrambe le direzioni se valido.
      if (CommonFunction::segment_is_valid(random_points[i], random_points[j],
                           arena, obstacles, clearance, sample_step))
      {
        knn_adj[i].push_back(j);
        knn_adj[j].push_back(static_cast<int>(i)); // grafo non orientato
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("prm_path_generator"),
              "Costruito grafo k-NN: %zu nodi, k=%zu (clearance=%.2f, step=%.2f).",
              N, k_neighbors, clearance, sample_step);
}

std::vector<std::pair<double,double>>
planning_pkg::PrmPathGenerator::indices_to_polyline(const std::vector<int>& idx_path) const
{
  std::vector<std::pair<double,double>> out;
  out.reserve(idx_path.size());
  for (int id : idx_path) {
    if (id < 0 || static_cast<std::size_t>(id) >= random_points.size()) continue;
    const auto& p = random_points[static_cast<std::size_t>(id)];
    out.emplace_back(p.x, p.y);
  }
  return out;
}


void PrmPathGenerator::set_default_frame_id(const std::string& fid)
{
  default_frame_id_ = fid;
}

void PrmPathGenerator::set_default_step_size(double s)
{
  default_step_size_ = s;
}

const std::string& PrmPathGenerator::default_frame_id() const
{
  return default_frame_id_;
}

double PrmPathGenerator::default_step_size() const
{
  return default_step_size_;
}

} // namespace planning_pkg
