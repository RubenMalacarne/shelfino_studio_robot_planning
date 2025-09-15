#include "planning_pkg/rrt_path.hpp"
#include "planning_pkg/common_function.hpp"
#include <algorithm> 
#include <cmath>     
#include <limits>    
#include <random>    
#include <utility>   
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

namespace planning_pkg
{

RRTPathGenerator::RRTPathGenerator(std::string default_frame_id, double default_step_size)
    : default_frame_id_(std::move(default_frame_id)), default_step_size_(default_step_size)
{
    initialize_random_generator();
}

void RRTPathGenerator::initialize_random_generator()
{
    std::random_device rd;
    rng_.seed(rd());
}

nav_msgs::msg::Path RRTPathGenerator::generate(
    const std::vector<std::pair<double, double>> &waypoints,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = default_frame_id_;

    if (waypoints.size() < 2)
    {
        RCLCPP_WARN(rclcpp::get_logger("rrt_path_generator"), "Not enough waypoints for path planning");
        return path;
    }

    const auto start = waypoints.front();
    const auto goal = waypoints.back();

    // Convert waypoints to geometry_msgs::Point
    geometry_msgs::msg::Point start_point;
    start_point.x = start.first;
    start_point.y = start.second;
    start_point.z = 0.0;

    geometry_msgs::msg::Point goal_point;
    goal_point.x = goal.first;
    goal_point.y = goal.second;
    goal_point.z = 0.0;

    // Validate start and goal points
    if (!is_point_valid(start_point, obstacles, arena))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rrt_path_generator"), "Start point is not valid");
        return path;
    }

    if (!is_point_valid(goal_point, obstacles, arena))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rrt_path_generator"), "Goal point is not valid");
        return path;
    }

    RCLCPP_INFO(rclcpp::get_logger("rrt_path_generator"), 
                "Starting RRT path planning from (%.2f, %.2f) to (%.2f, %.2f)",
                start_point.x, start_point.y, goal_point.x, goal_point.y);

    // Build RRT tree
    std::vector<RRTNode> tree = build_rrt_tree(start_point, goal_point, obstacles, arena);

    if (tree.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rrt_path_generator"), "Failed to build RRT tree");
        return path;
    }

    // Find goal node (closest to actual goal)
    int goal_node_id = -1;
    double min_goal_distance = std::numeric_limits<double>::infinity();
    
    for (const auto &node : tree)
    {
        double dist = distance(node.position, goal_point);
        if (dist < min_goal_distance)
        {
            min_goal_distance = dist;
            goal_node_id = node.id;
        }
    }

    if (goal_node_id == -1 || min_goal_distance > config_.goal_tolerance)
    {
        RCLCPP_WARN(rclcpp::get_logger("rrt_path_generator"), 
                   "Goal not reached within tolerance. Closest distance: %.3f", min_goal_distance);
    }

    // Extract path from tree
    std::vector<int> path_indices = extract_path(tree, goal_node_id);
    
    if (path_indices.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rrt_path_generator"), "Failed to extract path from tree");
        return path;
    }

    // Convert path indices to waypoints
    std::vector<geometry_msgs::msg::Point> path_points;
    for (int idx : path_indices)
    {
        if (idx >= 0 && static_cast<size_t>(idx) < tree.size())
        {
            path_points.push_back(tree[idx].position);
        }
    }

    // Add actual goal if not already included
    if (path_points.empty() || distance(path_points.back(), goal_point) > 1e-6)
    {
        path_points.push_back(goal_point);
    }

    // Optimize path
    std::vector<geometry_msgs::msg::Point> optimized_path = optimize_path(path_points, obstacles, arena);

    // Convert to nav_msgs::Path
    for (const auto &point : optimized_path)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position = point;
        pose_stamped.pose.orientation.w = 1.0;
        path.poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(rclcpp::get_logger("rrt_path_generator"), 
                "RRT path generated with %zu waypoints", path.poses.size());

    return path;
}

std::vector<RRTNode> RRTPathGenerator::build_rrt_tree(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &goal,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena)
{
    std::vector<RRTNode> tree;
    
    // Initialize tree with start node
    RRTNode start_node(start.x, start.y, -1, 0.0, 0);
    tree.push_back(start_node);

    std::uniform_real_distribution<double> goal_bias_dist(0.0, 1.0);
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration)
    {
        geometry_msgs::msg::Point random_point;
        
        // Goal biasing: sample goal with probability goal_bias
        if (goal_bias_dist(rng_) < config_.goal_bias)
        {
            random_point = sample_goal_biased_point(goal, arena);
        }
        else
        {
            random_point = sample_random_point(arena);
        }

        // Find nearest node in tree
        int nearest_id = find_nearest_node(random_point, tree);
        if (nearest_id == -1) continue;

        // Steer towards random point
        geometry_msgs::msg::Point new_point = steer(tree[nearest_id].position, random_point);

        // Check if path from nearest node to new point is collision-free
        if (is_collision_free(tree[nearest_id].position, new_point, obstacles, arena))
        {
            // Create new node
            double new_cost = tree[nearest_id].cost + distance(tree[nearest_id].position, new_point);
            RRTNode new_node(new_point.x, new_point.y, nearest_id, new_cost, static_cast<int>(tree.size()));
            tree.push_back(new_node);

            // Check if we reached the goal
            if (distance(new_point, goal) <= config_.goal_tolerance)
            {
                RCLCPP_INFO(rclcpp::get_logger("rrt_path_generator"), 
                           "Goal reached in %d iterations", iteration + 1);
                break;
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rrt_path_generator"), 
                "RRT tree built with %zu nodes", tree.size());

    return tree;
}

geometry_msgs::msg::Point RRTPathGenerator::sample_random_point(const geometry_msgs::msg::Polygon &arena) const
{
    // Get arena bounds
    double min_x, min_y, max_x, max_y;
    CommonFunction::compute_bbox(arena, min_x, min_y, max_x, max_y);
    
    // Add margin
    min_x += config_.arena_margin;
    min_y += config_.arena_margin;
    max_x -= config_.arena_margin;
    max_y -= config_.arena_margin;

    std::uniform_real_distribution<double> x_dist(min_x, max_x);
    std::uniform_real_distribution<double> y_dist(min_y, max_y);

    geometry_msgs::msg::Point point;
    point.x = x_dist(rng_);
    point.y = y_dist(rng_);
    point.z = 0.0;

    return point;
}

geometry_msgs::msg::Point RRTPathGenerator::sample_goal_biased_point(
    const geometry_msgs::msg::Point &goal,
    const geometry_msgs::msg::Polygon &arena) const
{
    // Sample around goal with some variance
    std::normal_distribution<double> x_dist(goal.x, 1.0);
    std::normal_distribution<double> y_dist(goal.y, 1.0);

    geometry_msgs::msg::Point point;
    point.x = x_dist(rng_);
    point.y = y_dist(rng_);
    point.z = 0.0;

    // Clamp to arena bounds
    double min_x, min_y, max_x, max_y;
    CommonFunction::compute_bbox(arena, min_x, min_y, max_x, max_y);
    
    point.x = std::clamp(point.x, min_x + config_.arena_margin, max_x - config_.arena_margin);
    point.y = std::clamp(point.y, min_y + config_.arena_margin, max_y - config_.arena_margin);

    return point;
}

int RRTPathGenerator::find_nearest_node(
    const geometry_msgs::msg::Point &target,
    const std::vector<RRTNode> &tree) const
{
    if (tree.empty()) return -1;

    int nearest_id = 0;
    double min_distance = distance(target, tree[0].position);

    for (size_t i = 1; i < tree.size(); ++i)
    {
        double dist = distance(target, tree[i].position);
        if (dist < min_distance)
        {
            min_distance = dist;
            nearest_id = static_cast<int>(i);
        }
    }

    return nearest_id;
}

geometry_msgs::msg::Point RRTPathGenerator::steer(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to) const
{
    double dist = distance(from, to);
    
    if (dist <= config_.step_size)
    {
        return to;
    }

    // Interpolate towards target
    double ratio = config_.step_size / dist;
    geometry_msgs::msg::Point result;
    result.x = from.x + ratio * (to.x - from.x);
    result.y = from.y + ratio * (to.y - from.y);
    result.z = 0.0;

    return result;
}

bool RRTPathGenerator::is_collision_free(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::segment_is_valid(from, to, arena, obstacles, 
                                          config_.obstacle_clearance, config_.sample_resolution);
}

bool RRTPathGenerator::is_point_valid(
    const geometry_msgs::msg::Point &point,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::point_is_valid(point.x, point.y, arena, obstacles, config_.obstacle_clearance);
}

std::vector<int> RRTPathGenerator::extract_path(
    const std::vector<RRTNode> &tree,
    int goal_node_id) const
{
    std::vector<int> path;
    
    if (goal_node_id < 0 || static_cast<size_t>(goal_node_id) >= tree.size())
    {
        return path;
    }

    // Trace back from goal to start
    int current_id = goal_node_id;
    while (current_id != -1)
    {
        path.push_back(current_id);
        current_id = tree[current_id].parent_id;
    }

    // Reverse to get start-to-goal path
    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<geometry_msgs::msg::Point> RRTPathGenerator::optimize_path(
    const std::vector<geometry_msgs::msg::Point> &path,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::optimize_path_with_raycasting(path, obstacles, arena, 
                                                       config_.obstacle_clearance, config_.sample_resolution);
}

std::vector<geometry_msgs::msg::Point> RRTPathGenerator::get_tree_nodes(
    const std::vector<RRTNode> &tree) const
{
    std::vector<geometry_msgs::msg::Point> nodes;
    for (const auto &node : tree)
    {
        nodes.push_back(node.position);
    }
    return nodes;
}

std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> RRTPathGenerator::get_tree_edges(
    const std::vector<RRTNode> &tree) const
{
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> edges;
    
    for (const auto &node : tree)
    {
        if (node.parent_id != -1)
        {
            edges.emplace_back(tree[node.parent_id].position, node.position);
        }
    }
    
    return edges;
}

double RRTPathGenerator::distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const
{
    return std::sqrt(CommonFunction::dist2(a, b));
}

bool RRTPathGenerator::is_inside_arena(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Polygon &arena) const
{
    return CommonFunction::point_in_polygon(arena, point.x, point.y);
}

void RRTPathGenerator::set_default_frame_id(const std::string &fid)
{
    default_frame_id_ = fid;
}

void RRTPathGenerator::set_default_step_size(double s)
{
    default_step_size_ = s;
}

void RRTPathGenerator::set_config(const RRTConfig &config)
{
    config_ = config;
}

const std::string &RRTPathGenerator::default_frame_id() const
{
    return default_frame_id_;
}

double RRTPathGenerator::default_step_size() const
{
    return default_step_size_;
}

const RRTConfig &RRTPathGenerator::get_config() const
{
    return config_;
}

} // namespace planning_pkg
