#pragma once
#include <string>
#include <vector>
#include <utility>
#include <random>
#include <cstddef> 
#include <map>
#include <cmath>
#include <optional>
#include <queue>
#include <functional>
#include <set>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

namespace planning_pkg
{
    // Agent configuration
    struct Agent
    {
        int id;
        geometry_msgs::msg::Point start;
        geometry_msgs::msg::Point goal;
        double radius; // Agent collision radius
        double max_speed;
        
        Agent(int agent_id, const geometry_msgs::msg::Point &start_pos, 
              const geometry_msgs::msg::Point &goal_pos, 
              double agent_radius = 0.2, double speed = 1.0)
            : id(agent_id), start(start_pos), goal(goal_pos), 
              radius(agent_radius), max_speed(speed) {}
    };

    // Conflict types
    enum class ConflictType
    {
        VERTEX,  // Two agents at same vertex at same time
        EDGE     // Two agents traverse same edge in opposite directions
    };

    // Conflict representation
    struct Conflict
    {
        ConflictType type;
        int agent1_id;
        int agent2_id;
        int vertex; // For vertex conflicts
        int edge_from, edge_to; // For edge conflicts
        int timestep; // When the conflict occurs
        
        Conflict(ConflictType t, int a1, int a2, int v, int t_step)
            : type(t), agent1_id(a1), agent2_id(a2), vertex(v), 
              edge_from(-1), edge_to(-1), timestep(t_step) {}
              
        Conflict(ConflictType t, int a1, int a2, int from, int to, int t_step)
            : type(t), agent1_id(a1), agent2_id(a2), vertex(-1), 
              edge_from(from), edge_to(to), timestep(t_step) {}
    };

    // Constraint for an agent
    struct Constraint
    {
        int agent_id;
        int vertex; // Constrained vertex
        int timestep; // Constrained timestep
        bool is_positive; // True for positive constraint (must be at vertex), false for negative (must not be at vertex)
        
        Constraint(int agent, int v, int t, bool positive = false)
            : agent_id(agent), vertex(v), timestep(t), is_positive(positive) {}
    };

    // A*CA* Configuration
    struct AStarCAConfig
    {
        double time_step = 0.1;           // Time discretization step
        int max_timesteps = 1000;         // Maximum planning horizon
        double collision_radius = 0.2;    // Default collision radius
        double max_speed = 1.0;           // Default max speed
        double goal_tolerance = 0.3;      // Distance tolerance to goal
        bool use_heuristic = true;        // Use heuristic for A* search
        bool use_conflict_avoidance = true; // Enable conflict avoidance
    };

    // Path for a single agent
    struct AgentPath
    {
        int agent_id;
        std::vector<geometry_msgs::msg::Point> waypoints;
        std::vector<int> timesteps; // Timestep for each waypoint
        double total_cost;
        bool valid;
        
        AgentPath() : agent_id(-1), total_cost(0.0), valid(false) {}
        AgentPath(int id) : agent_id(id), total_cost(0.0), valid(false) {}
    };

    // Multi-agent A* with Conflict Avoidance (A*CA*)
    class MultiAgentAStar
    {
    public:
        MultiAgentAStar(std::string default_frame_id, double default_step_size);
        MultiAgentAStar() : MultiAgentAStar("default_frame", 0.1) {}

        // Main planning function
        std::vector<AgentPath> plan_paths(
            const std::vector<Agent> &agents,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);

        // Configuration methods
        void set_default_frame_id(const std::string &fid);
        void set_default_step_size(double s);
        void set_config(const AStarCAConfig &config);
        const std::string &default_frame_id() const;
        double default_step_size() const;
        const AStarCAConfig &get_config() const;

        // Individual agent path planning
        AgentPath plan_single_agent_path(
            const Agent &agent,
            const std::vector<Constraint> &constraints,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);

        // Conflict detection and resolution
        std::vector<Conflict> detect_conflicts(
            const std::vector<AgentPath> &paths) const;

        std::vector<Constraint> resolve_conflict(
            const Conflict &conflict) const;

        // Graph construction and search
        std::vector<std::vector<int>> build_adjacency_graph(
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        std::vector<geometry_msgs::msg::Point> discretize_environment(
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        // A* search with constraints
        std::vector<int> astar_search(
            int start_vertex, int goal_vertex,
            const std::vector<geometry_msgs::msg::Point> &vertices,
            const std::vector<std::vector<int>> &adjacency,
            const std::vector<Constraint> &constraints,
            int agent_id) const;

        // Utility methods
        bool is_constraint_satisfied(
            int agent_id, int vertex, int timestep,
            const std::vector<Constraint> &constraints) const;

        bool is_collision_free(
            const geometry_msgs::msg::Point &from,
            const geometry_msgs::msg::Point &to,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        bool is_point_valid(
            const geometry_msgs::msg::Point &point,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        double heuristic(
            const geometry_msgs::msg::Point &current,
            const geometry_msgs::msg::Point &goal) const;

        // Convert to nav_msgs::Path format
        nav_msgs::msg::Path agent_path_to_nav_path(
            const AgentPath &agent_path) const;

        std::vector<nav_msgs::msg::Path> agent_paths_to_nav_paths(
            const std::vector<AgentPath> &agent_paths) const;

        // Statistics and debugging
        struct PlanningStats
        {
            int total_conflicts_detected;
            int total_constraints_added;
            double total_planning_time_ms;
            bool all_paths_found;
            std::vector<int> agents_with_paths;
        };

        PlanningStats get_last_planning_stats() const;

        // Helper method for finding closest vertex
        int find_closest_vertex(
            const geometry_msgs::msg::Point &point,
            const std::vector<geometry_msgs::msg::Point> &vertices) const;

    private:
        std::string default_frame_id_;
        double default_step_size_;
        AStarCAConfig config_;
        mutable PlanningStats last_stats_;
        mutable std::mt19937 rng_;

        // Helper methods
        double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const;
        bool is_inside_arena(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Polygon &arena) const;
        void initialize_random_generator();
        
        // Conflict resolution strategies
        std::vector<Constraint> resolve_vertex_conflict(const Conflict &conflict) const;
        std::vector<Constraint> resolve_edge_conflict(const Conflict &conflict) const;
        
        // Path validation
        bool validate_agent_path(const AgentPath &path, const Agent &agent) const;
    };

} // namespace planning_pkg
