# RRT and Multi-Agent A* (A*CA*) Path Planning Implementation

This document describes the implementation of Rapidly-exploring Random Tree (RRT) and Multi-Agent A* with Conflict Avoidance (A*CA*) path planning algorithms for the planning_pkg package.

## Overview

The implementation provides two main path planning approaches:

1. **RRT (Rapidly-exploring Random Tree)**: A sampling-based algorithm for single-agent path planning
2. **Multi-Agent A* with Conflict Avoidance (A*CA*)**: A coordinated multi-agent path planning algorithm

## Components

### 1. RRT Path Planner (`rrt_path.hpp` / `rrt_path.cpp`)

The RRT implementation includes:

- **RRTNode Structure**: Represents nodes in the RRT tree with position, parent, cost, and ID
- **RRTConfig**: Configuration parameters for the RRT algorithm
- **RRTPathGenerator**: Main class implementing the RRT algorithm

#### Key Features:
- Goal biasing for faster convergence
- Collision checking with obstacles and arena boundaries
- Path optimization using raycasting
- Configurable parameters (step size, goal tolerance, max iterations, etc.)

#### Configuration Parameters:
```cpp
struct RRTConfig {
    double step_size = 0.5;           // Maximum step size for RRT expansion
    double goal_tolerance = 0.3;      // Distance tolerance to consider goal reached
    int max_iterations = 10000;       // Maximum number of RRT iterations
    double goal_bias = 0.1;           // Probability of sampling goal directly
    double arena_margin = 0.1;        // Safety margin from arena boundaries
    double obstacle_clearance = 0.15; // Safety clearance from obstacles
    double sample_resolution = 0.05;  // Resolution for collision checking
};
```

### 2. Multi-Agent A* with Conflict Avoidance (`multiagent_astar.hpp` / `multiagent_astar.cpp`)

The A*CA* implementation includes:

- **Agent Structure**: Represents individual agents with start/goal positions and properties
- **Conflict Detection**: Identifies vertex and edge conflicts between agents
- **Constraint Resolution**: Resolves conflicts by adding constraints
- **MultiAgentAStar**: Main class implementing the A*CA* algorithm

#### Key Features:
- Conflict detection between multiple agents
- Constraint-based conflict resolution
- Time-discretized planning
- Coordinated multi-agent path planning

#### Configuration Parameters:
```cpp
struct AStarCAConfig {
    double time_step = 0.1;           // Time discretization step
    int max_timesteps = 1000;         // Maximum planning horizon
    double collision_radius = 0.2;    // Default collision radius
    double max_speed = 1.0;           // Default max speed
    double goal_tolerance = 0.3;      // Distance tolerance to goal
    bool use_heuristic = true;        // Use heuristic for A* search
    bool use_conflict_avoidance = true; // Enable conflict avoidance
};
```

### 3. RRT Orchestrator (`path_orchestrator_rrt.cpp`)

The orchestrator integrates both planning approaches:

- **Individual RRT Planning**: Uses RRT for each agent independently
- **Multi-Agent A*CA* Planning**: Uses coordinated planning with conflict avoidance
- **Fallback Mechanism**: Falls back to individual RRT if multi-agent planning fails
- **Linear Path Optimization**: Uses linear paths when direct paths are feasible

## Usage

### Building the Package

The implementation is integrated into the existing CMakeLists.txt. Build the package using:

```bash
cd /home/gabrielevm/Projects/RMPTP/shelfino_studio_ws
colcon build --packages-select planning_pkg
```

### Running the RRT Orchestrator

#### Individual RRT Planning (Default)
```bash
ros2 launch planning_pkg rrt_executor.launch.py
```

#### Multi-Agent A*CA* Planning
```bash
ros2 launch planning_pkg rrt_executor.launch.py use_multiagent:=true
```

#### Custom Configuration
```bash
ros2 launch planning_pkg rrt_executor.launch.py \
    use_multiagent:=true \
    rrt_step_size:=0.3 \
    rrt_max_iterations:=8000 \
    astar_time_step:=0.05 \
    astar_collision_radius:=0.25
```

### Launch Parameters

- `use_multiagent`: Enable multi-agent A*CA* planning (default: false)
- `rrt_step_size`: RRT step size for tree expansion (default: 0.5)
- `rrt_max_iterations`: Maximum RRT iterations (default: 5000)
- `rrt_goal_bias`: RRT goal biasing probability (default: 0.1)
- `astar_time_step`: Multi-agent time discretization (default: 0.1)
- `astar_collision_radius`: Agent collision radius (default: 0.2)

## Algorithm Details

### RRT Algorithm

1. **Initialization**: Start with the initial position as the root node
2. **Sampling**: Sample random points in the free space (with goal biasing)
3. **Nearest Neighbor**: Find the nearest node in the tree to the sampled point
4. **Steering**: Create a new node by steering from the nearest node toward the sampled point
5. **Collision Checking**: Verify the path from nearest to new node is collision-free
6. **Tree Extension**: Add the new node to the tree if valid
7. **Goal Check**: Check if the goal is reached within tolerance
8. **Path Extraction**: Extract the path from start to goal through the tree

### Multi-Agent A*CA* Algorithm

1. **Environment Discretization**: Create a graph of reachable positions
2. **Individual Planning**: Plan initial paths for each agent using A*
3. **Conflict Detection**: Identify vertex and edge conflicts between agents
4. **Constraint Resolution**: Add constraints to resolve conflicts
5. **Replanning**: Replan paths for affected agents with new constraints
6. **Iteration**: Repeat until all conflicts are resolved or maximum iterations reached

## Integration with Existing System

The implementation integrates seamlessly with the existing planning framework:

- **Same Interface**: Uses the same ROS2 topics and message types
- **Compatible with Existing Components**: Works with existing obstacle detection, arena mapping, and visualization
- **Fallback Support**: Falls back to linear path planning when direct paths are feasible
- **Performance Monitoring**: Includes timing and statistics for performance analysis

## Performance Considerations

### RRT Performance
- **Convergence**: Goal biasing improves convergence speed
- **Path Quality**: Path optimization reduces unnecessary waypoints
- **Collision Checking**: Efficient collision checking using existing CommonFunction utilities

### Multi-Agent A*CA* Performance
- **Conflict Resolution**: Constraint-based approach ensures conflict-free paths
- **Scalability**: Performance depends on the number of agents and environment complexity
- **Time Complexity**: O(n²) for conflict detection, O(n log n) for A* search per agent

## Visualization

The implementation supports visualization through the existing RViz integration:

- **RRT Tree**: Can visualize the RRT tree structure
- **Agent Paths**: Individual agent paths are published as nav_msgs::Path
- **Conflict Visualization**: Conflicts can be visualized for debugging

## Future Enhancements

Potential improvements for the implementation:

1. **RRT* Integration**: Add optimal RRT* for better path quality
2. **Dynamic Obstacles**: Support for moving obstacles
3. **Real-time Replanning**: Online replanning capabilities
4. **Advanced Conflict Resolution**: More sophisticated conflict resolution strategies
5. **Performance Optimization**: Parallel processing for multi-agent planning

## Troubleshooting

### Common Issues

1. **No Path Found**: Check obstacle clearance and goal tolerance parameters
2. **Slow Convergence**: Adjust goal bias and step size parameters
3. **Multi-Agent Failures**: Reduce collision radius or increase time step
4. **Memory Issues**: Reduce max iterations or environment discretization resolution

### Debug Information

The implementation provides extensive logging:
- RRT tree construction progress
- Multi-agent conflict detection and resolution
- Path planning statistics and timing
- Fallback mechanism activation

## References

1. LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning.
2. Silver, D. (2005). Cooperative pathfinding.
3. Standley, T. (2010). Finding optimal solutions to cooperative pathfinding problems.
