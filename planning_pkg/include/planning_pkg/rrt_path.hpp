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
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

namespace planning_pkg
{
    // RRT Node structure
    struct RRTNode
    {
        geometry_msgs::msg::Point position;
        int parent_id;
        double cost; // Cost from root to this node
        int id;
        
        RRTNode() : parent_id(-1), cost(0.0), id(-1) {}
        RRTNode(double x, double y, int parent = -1, double node_cost = 0.0, int node_id = -1)
            : parent_id(parent), cost(node_cost), id(node_id)
        {
            position.x = x;
            position.y = y;
            position.z = 0.0;
        }
    };

    // RRT Configuration parameters
    struct RRTConfig
    {
        double step_size = 0.5;           // Maximum step size for RRT expansion
        double goal_tolerance = 0.3;      // Distance tolerance to consider goal reached
        int max_iterations = 10000;       // Maximum number of RRT iterations
        double goal_bias = 0.1;           // Probability of sampling goal directly
        double arena_margin = 0.1;        // Safety margin from arena boundaries
        double obstacle_clearance = 0.15; // Safety clearance from obstacles
        double sample_resolution = 0.05;  // Resolution for collision checking
    };

    class RRTPathGenerator
    {
    public:
        RRTPathGenerator(std::string default_frame_id, double default_step_size);
        RRTPathGenerator() : RRTPathGenerator("default_frame", 0.1) {}

        // Main path generation function
        nav_msgs::msg::Path generate(
            const std::vector<std::pair<double, double>> &waypoints,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);

        // Configuration methods
        void set_default_frame_id(const std::string &fid);
        void set_default_step_size(double s);
        void set_config(const RRTConfig &config);
        const std::string &default_frame_id() const;
        double default_step_size() const;
        const RRTConfig &get_config() const;

        // RRT algorithm methods
        std::vector<RRTNode> build_rrt_tree(
            const geometry_msgs::msg::Point &start,
            const geometry_msgs::msg::Point &goal,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena);

        // Utility methods
        geometry_msgs::msg::Point sample_random_point(
            const geometry_msgs::msg::Polygon &arena) const;
        
        geometry_msgs::msg::Point sample_goal_biased_point(
            const geometry_msgs::msg::Point &goal,
            const geometry_msgs::msg::Polygon &arena) const;

        int find_nearest_node(
            const geometry_msgs::msg::Point &target,
            const std::vector<RRTNode> &tree) const;

        geometry_msgs::msg::Point steer(
            const geometry_msgs::msg::Point &from,
            const geometry_msgs::msg::Point &to) const;

        bool is_collision_free(
            const geometry_msgs::msg::Point &from,
            const geometry_msgs::msg::Point &to,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        bool is_point_valid(
            const geometry_msgs::msg::Point &point,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        std::vector<int> extract_path(
            const std::vector<RRTNode> &tree,
            int goal_node_id) const;

        // Path optimization
        std::vector<geometry_msgs::msg::Point> optimize_path(
            const std::vector<geometry_msgs::msg::Point> &path,
            const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
            const geometry_msgs::msg::Polygon &arena) const;

        // Visualization support
        std::vector<geometry_msgs::msg::Point> get_tree_nodes(
            const std::vector<RRTNode> &tree) const;
        
        std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> get_tree_edges(
            const std::vector<RRTNode> &tree) const;

    private:
        std::string default_frame_id_;
        double default_step_size_;
        RRTConfig config_;
        mutable std::mt19937 rng_;

        // Helper methods
        double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const;
        bool is_inside_arena(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Polygon &arena) const;
        void initialize_random_generator();
    };

} // namespace planning_pkg
