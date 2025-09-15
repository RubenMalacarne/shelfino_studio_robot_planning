#include "planning_pkg/multiagent_astar.hpp"
#include "planning_pkg/common_function.hpp"
#include <algorithm> 
#include <cmath>     
#include <limits>    
#include <random>    
#include <utility>   
#include <vector>
#include <queue>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

namespace planning_pkg
{

MultiAgentAStar::MultiAgentAStar(std::string default_frame_id, double default_step_size)
    : default_frame_id_(std::move(default_frame_id)), default_step_size_(default_step_size)
{
    initialize_random_generator();
    last_stats_ = PlanningStats{};
}

void MultiAgentAStar::initialize_random_generator()
{
    std::random_device rd;
    rng_.seed(rd());
}

std::vector<AgentPath> MultiAgentAStar::plan_paths(
    const std::vector<Agent> &agents,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Reset statistics
    last_stats_ = PlanningStats{};
    last_stats_.all_paths_found = true;

    RCLCPP_INFO(rclcpp::get_logger("multiagent_astar"), 
                "Starting multi-agent A*CA* planning for %zu agents", agents.size());

    // Discretize environment
    std::vector<geometry_msgs::msg::Point> vertices = discretize_environment(obstacles, arena);
    if (vertices.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("multiagent_astar"), "Failed to discretize environment");
        return {};
    }

    // Build adjacency graph
    std::vector<std::vector<int>> adjacency = build_adjacency_graph(obstacles, arena);
    
    RCLCPP_INFO(rclcpp::get_logger("multiagent_astar"), 
                "Environment discretized into %zu vertices", vertices.size());

    // Find start and goal vertices for each agent
    std::vector<int> start_vertices(agents.size());
    std::vector<int> goal_vertices(agents.size());
    
    for (size_t i = 0; i < agents.size(); ++i)
    {
        start_vertices[i] = find_closest_vertex(agents[i].start, vertices);
        goal_vertices[i] = find_closest_vertex(agents[i].goal, vertices);
        
        if (start_vertices[i] == -1 || goal_vertices[i] == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("multiagent_astar"), 
                        "Failed to find start/goal vertices for agent %d", agents[i].id);
            last_stats_.all_paths_found = false;
        }
    }

    // Plan paths iteratively with conflict resolution
    std::vector<AgentPath> agent_paths(agents.size());
    std::vector<Constraint> all_constraints;
    
    for (size_t i = 0; i < agents.size(); ++i)
    {
        if (start_vertices[i] == -1 || goal_vertices[i] == -1)
        {
            agent_paths[i] = AgentPath(agents[i].id);
            continue;
        }

        // Plan path for agent i with current constraints
        agent_paths[i] = plan_single_agent_path(agents[i], all_constraints, obstacles, arena);
        
        if (!agent_paths[i].valid)
        {
            RCLCPP_WARN(rclcpp::get_logger("multiagent_astar"), 
                       "Failed to find path for agent %d", agents[i].id);
            last_stats_.all_paths_found = false;
            continue;
        }

        last_stats_.agents_with_paths.push_back(agents[i].id);

        // Detect conflicts with previously planned agents
        if (config_.use_conflict_avoidance)
        {
            std::vector<Conflict> conflicts = detect_conflicts(agent_paths);
            last_stats_.total_conflicts_detected += conflicts.size();

            // Resolve conflicts by adding constraints
            for (const auto &conflict : conflicts)
            {
                std::vector<Constraint> new_constraints = resolve_conflict(conflict);
                all_constraints.insert(all_constraints.end(), new_constraints.begin(), new_constraints.end());
                last_stats_.total_constraints_added += new_constraints.size();
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    last_stats_.total_planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    RCLCPP_INFO(rclcpp::get_logger("multiagent_astar"), 
                "Multi-agent planning completed in %.2f ms. Found paths for %zu/%zu agents",
                last_stats_.total_planning_time_ms, last_stats_.agents_with_paths.size(), agents.size());

    return agent_paths;
}

AgentPath MultiAgentAStar::plan_single_agent_path(
    const Agent &agent,
    const std::vector<Constraint> &constraints,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena)
{
    AgentPath path(agent.id);

    // Discretize environment
    std::vector<geometry_msgs::msg::Point> vertices = discretize_environment(obstacles, arena);
    if (vertices.empty())
    {
        return path;
    }

    // Build adjacency graph
    std::vector<std::vector<int>> adjacency = build_adjacency_graph(obstacles, arena);

    // Find start and goal vertices
    int start_vertex = find_closest_vertex(agent.start, vertices);
    int goal_vertex = find_closest_vertex(agent.goal, vertices);

    if (start_vertex == -1 || goal_vertex == -1)
    {
        return path;
    }

    // Run A* search
    std::vector<int> path_vertices = astar_search(start_vertex, goal_vertex, vertices, adjacency, constraints, agent.id);

    if (path_vertices.empty())
    {
        return path;
    }

    // Convert vertex path to waypoints
    for (int vertex : path_vertices)
    {
        path.waypoints.push_back(vertices[vertex]);
    }

    // Calculate timesteps
    path.timesteps.resize(path.waypoints.size());
    for (size_t i = 0; i < path.waypoints.size(); ++i)
    {
        path.timesteps[i] = static_cast<int>(i * config_.time_step);
    }

    // Calculate total cost
    path.total_cost = 0.0;
    for (size_t i = 1; i < path.waypoints.size(); ++i)
    {
        path.total_cost += distance(path.waypoints[i-1], path.waypoints[i]);
    }

    path.valid = true;
    return path;
}

std::vector<Conflict> MultiAgentAStar::detect_conflicts(const std::vector<AgentPath> &paths) const
{
    std::vector<Conflict> conflicts;

    for (size_t i = 0; i < paths.size(); ++i)
    {
        if (!paths[i].valid) continue;

        for (size_t j = i + 1; j < paths.size(); ++j)
        {
            if (!paths[j].valid) continue;

            // Check for vertex conflicts
            for (size_t t = 0; t < std::min(paths[i].timesteps.size(), paths[j].timesteps.size()); ++t)
            {
                if (paths[i].timesteps[t] == paths[j].timesteps[t])
                {
                    // Check if agents are at the same location
                    double dist = distance(paths[i].waypoints[t], paths[j].waypoints[t]);
                    if (dist < (paths[i].waypoints[t].x + paths[j].waypoints[t].x) * 0.1) // Simplified collision check
                    {
                        conflicts.emplace_back(ConflictType::VERTEX, paths[i].agent_id, paths[j].agent_id, 
                                             static_cast<int>(t), static_cast<int>(t));
                    }
                }
            }

            // Check for edge conflicts
            for (size_t t = 0; t < std::min(paths[i].timesteps.size() - 1, paths[j].timesteps.size() - 1); ++t)
            {
                if (paths[i].timesteps[t] == paths[j].timesteps[t] && 
                    paths[i].timesteps[t+1] == paths[j].timesteps[t+1])
                {
                    // Check if agents cross paths
                    double dist1 = distance(paths[i].waypoints[t], paths[j].waypoints[t+1]);
                    double dist2 = distance(paths[i].waypoints[t+1], paths[j].waypoints[t]);
                    
                    if (dist1 < 0.1 && dist2 < 0.1) // Simplified edge conflict detection
                    {
                        conflicts.emplace_back(ConflictType::EDGE, paths[i].agent_id, paths[j].agent_id, 
                                             static_cast<int>(t), static_cast<int>(t+1), static_cast<int>(t));
                    }
                }
            }
        }
    }

    return conflicts;
}

std::vector<Constraint> MultiAgentAStar::resolve_conflict(const Conflict &conflict) const
{
    std::vector<Constraint> constraints;

    if (conflict.type == ConflictType::VERTEX)
    {
        constraints = resolve_vertex_conflict(conflict);
    }
    else if (conflict.type == ConflictType::EDGE)
    {
        constraints = resolve_edge_conflict(conflict);
    }

    return constraints;
}

std::vector<Constraint> MultiAgentAStar::resolve_vertex_conflict(const Conflict &conflict) const
{
    std::vector<Constraint> constraints;
    
    // Add constraint for agent1 to not be at vertex at timestep
    constraints.emplace_back(conflict.agent1_id, conflict.vertex, conflict.timestep, false);
    
    // Add constraint for agent2 to not be at vertex at timestep
    constraints.emplace_back(conflict.agent2_id, conflict.vertex, conflict.timestep, false);
    
    return constraints;
}

std::vector<Constraint> MultiAgentAStar::resolve_edge_conflict(const Conflict &conflict) const
{
    std::vector<Constraint> constraints;
    
    // Add constraint for agent1 to not traverse edge at timestep
    constraints.emplace_back(conflict.agent1_id, conflict.edge_from, conflict.timestep, false);
    constraints.emplace_back(conflict.agent1_id, conflict.edge_to, conflict.timestep + 1, false);
    
    // Add constraint for agent2 to not traverse edge at timestep
    constraints.emplace_back(conflict.agent2_id, conflict.edge_from, conflict.timestep, false);
    constraints.emplace_back(conflict.agent2_id, conflict.edge_to, conflict.timestep + 1, false);
    
    return constraints;
}

std::vector<std::vector<int>> MultiAgentAStar::build_adjacency_graph(
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    std::vector<geometry_msgs::msg::Point> vertices = discretize_environment(obstacles, arena);
    std::vector<std::vector<int>> adjacency(vertices.size());

    // Connect nearby vertices
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        for (size_t j = i + 1; j < vertices.size(); ++j)
        {
            double dist = distance(vertices[i], vertices[j]);
            if (dist <= config_.max_speed * config_.time_step * 2.0) // Within reachable distance
            {
                if (is_collision_free(vertices[i], vertices[j], obstacles, arena))
                {
                    adjacency[i].push_back(static_cast<int>(j));
                    adjacency[j].push_back(static_cast<int>(i));
                }
            }
        }
    }

    return adjacency;
}

std::vector<geometry_msgs::msg::Point> MultiAgentAStar::discretize_environment(
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    std::vector<geometry_msgs::msg::Point> vertices;

    // Get arena bounds
    double min_x, min_y, max_x, max_y;
    CommonFunction::compute_bbox(arena, min_x, min_y, max_x, max_y);

    // Create grid of points
    double resolution = config_.max_speed * config_.time_step;
    for (double x = min_x; x <= max_x; x += resolution)
    {
        for (double y = min_y; y <= max_y; y += resolution)
        {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;

            if (is_point_valid(point, obstacles, arena))
            {
                vertices.push_back(point);
            }
        }
    }

    return vertices;
}

std::vector<int> MultiAgentAStar::astar_search(
    int start_vertex, int goal_vertex,
    const std::vector<geometry_msgs::msg::Point> &vertices,
    const std::vector<std::vector<int>> &adjacency,
    const std::vector<Constraint> &constraints,
    int agent_id) const
{
    if (start_vertex == goal_vertex)
        return {start_vertex};

    const size_t N = vertices.size();
    if (start_vertex < 0 || goal_vertex < 0 || 
        static_cast<size_t>(start_vertex) >= N || static_cast<size_t>(goal_vertex) >= N)
    {
        return {};
    }

    // Heuristic function
    auto heuristic = [&](int vertex) -> double {
        return distance(vertices[vertex], vertices[goal_vertex]);
    };

    // Priority queue: (f_cost, timestep, vertex_id)
    std::priority_queue<std::tuple<double, int, int>, 
                       std::vector<std::tuple<double, int, int>>, 
                       std::greater<std::tuple<double, int, int>>> open_set;

    // Data structures
    std::map<std::pair<int, int>, double> g_cost; // (vertex, timestep) -> cost
    std::map<std::pair<int, int>, int> prev_vertex; // (vertex, timestep) -> previous vertex
    std::map<std::pair<int, int>, int> prev_timestep; // (vertex, timestep) -> previous timestep
    std::set<std::pair<int, int>> closed_set;

    // Initialize start
    g_cost[{start_vertex, 0}] = 0.0;
    double f_cost = g_cost[{start_vertex, 0}] + heuristic(start_vertex);
    open_set.push({f_cost, 0, start_vertex});

    while (!open_set.empty())
    {
        auto [current_f, current_t, current_v] = open_set.top();
        open_set.pop();

        if (closed_set.count({current_v, current_t}))
            continue;

        closed_set.insert({current_v, current_t});

        if (current_v == goal_vertex)
        {
            // Reconstruct path
            std::vector<int> path;
            int v = current_v;
            int t = current_t;
            
            while (v != start_vertex || t != 0)
            {
                path.push_back(v);
                int prev_v = prev_vertex[{v, t}];
                int prev_t = prev_timestep[{v, t}];
                v = prev_v;
                t = prev_t;
            }
            path.push_back(start_vertex);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Expand neighbors
        for (int neighbor : adjacency[current_v])
        {
            int next_timestep = current_t + 1;
            
            // Check constraints
            if (!is_constraint_satisfied(agent_id, neighbor, next_timestep, constraints))
                continue;

            double tentative_g = g_cost[{current_v, current_t}] + 
                               distance(vertices[current_v], vertices[neighbor]);

            if (!g_cost.count({neighbor, next_timestep}) || 
                tentative_g < g_cost[{neighbor, next_timestep}])
            {
                g_cost[{neighbor, next_timestep}] = tentative_g;
                prev_vertex[{neighbor, next_timestep}] = current_v;
                prev_timestep[{neighbor, next_timestep}] = current_t;
                
                double f = tentative_g + heuristic(neighbor);
                open_set.push({f, next_timestep, neighbor});
            }
        }
    }

    return {}; // No path found
}

bool MultiAgentAStar::is_constraint_satisfied(
    int agent_id, int vertex, int timestep,
    const std::vector<Constraint> &constraints) const
{
    for (const auto &constraint : constraints)
    {
        if (constraint.agent_id == agent_id && 
            constraint.vertex == vertex && 
            constraint.timestep == timestep)
        {
            return constraint.is_positive; // Must be at vertex if positive constraint
        }
    }
    return true; // No constraint violated
}

bool MultiAgentAStar::is_collision_free(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::segment_is_valid(from, to, arena, obstacles, 
                                          config_.collision_radius, config_.time_step);
}

bool MultiAgentAStar::is_point_valid(
    const geometry_msgs::msg::Point &point,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::point_is_valid(point.x, point.y, arena, obstacles, config_.collision_radius);
}

double MultiAgentAStar::heuristic(
    const geometry_msgs::msg::Point &current,
    const geometry_msgs::msg::Point &goal) const
{
    return distance(current, goal);
}

double MultiAgentAStar::distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const
{
    return std::sqrt(CommonFunction::dist2(a, b));
}

bool MultiAgentAStar::is_inside_arena(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::point_in_polygon(arena, point.x, point.y);
}

int MultiAgentAStar::find_closest_vertex(
    const geometry_msgs::msg::Point &point,
    const std::vector<geometry_msgs::msg::Point> &vertices) const
{
    if (vertices.empty()) return -1;

    int closest = 0;
    double min_dist = distance(point, vertices[0]);

    for (size_t i = 1; i < vertices.size(); ++i)
    {
        double dist = distance(point, vertices[i]);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest = static_cast<int>(i);
        }
    }

    return closest;
}


nav_msgs::msg::Path MultiAgentAStar::agent_path_to_nav_path(const AgentPath &agent_path) const
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = default_frame_id_;

    for (const auto &waypoint : agent_path.waypoints)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position = waypoint;
        pose_stamped.pose.orientation.w = 1.0;
        path.poses.push_back(pose_stamped);
    }

    return path;
}

std::vector<nav_msgs::msg::Path> MultiAgentAStar::agent_paths_to_nav_paths(
    const std::vector<AgentPath> &agent_paths) const
{
    std::vector<nav_msgs::msg::Path> nav_paths;
    
    for (const auto &agent_path : agent_paths)
    {
        if (agent_path.valid)
        {
            nav_paths.push_back(agent_path_to_nav_path(agent_path));
        }
    }
    
    return nav_paths;
}

MultiAgentAStar::PlanningStats MultiAgentAStar::get_last_planning_stats() const
{
    return last_stats_;
}

void MultiAgentAStar::set_default_frame_id(const std::string &fid)
{
    default_frame_id_ = fid;
}

void MultiAgentAStar::set_default_step_size(double s)
{
    default_step_size_ = s;
}

void MultiAgentAStar::set_config(const AStarCAConfig &config)
{
    config_ = config;
}

const std::string &MultiAgentAStar::default_frame_id() const
{
    return default_frame_id_;
}

double MultiAgentAStar::default_step_size() const
{
    return default_step_size_;
}

const AStarCAConfig &MultiAgentAStar::get_config() const
{
    return config_;
}

} // namespace planning_pkg
