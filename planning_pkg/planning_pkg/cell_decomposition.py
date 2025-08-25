#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon as MatplotlibPolygon
from matplotlib.path import Path
import numpy as np
from typing import List, Tuple, Dict, Optional
import math
import heapq

# ROS2 message imports
from obstacles_msgs.msg import ObstacleArrayMsg
from geometry_msgs.msg import Polygon as GeometryPolygon
from geometry_msgs.msg import Point32, PoseArray, PoseWithCovarianceStamped
from std_srvs.srv import Trigger

class CellDecomposition(Node):
    def __init__(self):
        super().__init__('cell_decomposition')
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Data storage
        self.arena_data = None
        self.obstacles_data = None
        self.gates_data = None
        self.pos1_data = None
        self.pos2_data = None
        
        # Subscribers
        self.sub_obstacles = self.create_subscription(
            ObstacleArrayMsg, '/inflated_obstacles', 
            self.callback_obstacles, qos)
        
        self.sub_arena = self.create_subscription(
            GeometryPolygon, '/inflated_arena',
            self.callback_arena, qos)
        
        self.sub_gates = self.create_subscription(
            PoseArray, '/published_gates',
            self.callback_gates, qos)
        
        self.sub_pos1 = self.create_subscription(
            PoseWithCovarianceStamped, '/published_pos1',
            self.callback_pos1, qos)
        
        self.sub_pos2 = self.create_subscription(
            PoseWithCovarianceStamped, '/published_pos2',
            self.callback_pos2, qos)
        
        # Service client
        self.trigger_client = self.create_client(Trigger, '/service_trigger_inflated')
        
        # Timer to call the service
        self.timer = self.create_timer(1.0, self.call_trigger_service)
        
        # Flag to prevent multiple plots
        self.plot_generated = False
        # Small geometry tolerance and outputs
        self._EPS = 1e-6
        self.cells: List[List[Tuple[float, float]]] = []
        self.cell_centroids: List[Tuple[float, float]] = []
        self.adjacency_graph: Dict[int, List[Tuple[int, float]]] = {}
        
    def call_trigger_service(self):
        if self.trigger_client.service_is_ready():
            self.timer.cancel()
            request = Trigger.Request()
            future = self.trigger_client.call_async(request)
            future.add_done_callback(self.trigger_response_callback)
        else:
            self.get_logger().info('Waiting for trigger service...')
    
    def trigger_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Trigger service successful: {response.message}')
            else:
                self.get_logger().warn(f'Trigger service failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    # --- CALLBACKS ---
    def callback_obstacles(self, msg):
        self.obstacles_data = msg
        self.get_logger().info(f'Received obstacles: {len(msg.obstacles)} obstacles')
        self.check_and_plot()
    
    def callback_arena(self, msg):
        self.arena_data = msg
        self.get_logger().info(f'Received arena: {len(msg.points)} points')
        self.check_and_plot()
    
    def callback_gates(self, msg):
        self.gates_data = msg
        self.get_logger().info(f'Received gates: {len(msg.poses)} gates')
        self.check_and_plot()
    
    def callback_pos1(self, msg):
        self.pos1_data = msg
        self.get_logger().info('Received position 1')
        self.check_and_plot()
    
    def callback_pos2(self, msg):
        self.pos2_data = msg
        self.get_logger().info('Received position 2')
        self.check_and_plot()
    
    def all_data_received(self):
        """Check if all necessary data has been received."""
        return self.arena_data and self.obstacles_data and self.gates_data and \
               self.pos1_data and self.pos2_data

    def check_and_plot(self):
        if self.all_data_received() and not self.plot_generated:
            self.plot_generated = True # Set flag to true to avoid re-plotting
            self.plot_map_with_decomposition()

    # --- HELPER FUNCTIONS FOR DECOMPOSITION ---

    def is_point_in_polygon(self, point: Tuple[float, float], polygon_vertices: List[Tuple[float, float]]) -> bool:
        """Checks if a point is inside a polygon using matplotlib.path."""
        if not polygon_vertices:
            return False
        # A small tolerance is added to contains_point to correctly handle points on the edge
        return Path(polygon_vertices).contains_point(point, radius=1e-9)

    def is_point_in_circle(self, point: Tuple[float, float], center: Tuple[float, float], radius: float) -> bool:
        """Checks if a point is inside a circle."""
        return np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2) < radius - 1e-9

    def get_vertical_line_segment_intersection(self, x_coord: float, p1: Tuple[float, float], p2: Tuple[float, float]) -> float | None:
        """Finds the y-intersection of a vertical line at x_coord with a line segment."""
        x1, y1 = p1
        x2, y2 = p2
        if x1 == x2: # Vertical segment
            return None
        if (x1 <= x_coord <= x2) or (x2 <= x_coord <= x1):
            y_intersect = y1 + (x_coord - x1) * (y2 - y1) / (x2 - x1)
            return y_intersect
        return None

    def get_vertical_line_circle_intersections(self, x_coord: float, center: Tuple[float, float], radius: float) -> List[float]:
        """Finds the y-intersections of a vertical line at x_coord with a circle."""
        cx, cy = center
        dx_sq = (x_coord - cx)**2
        if dx_sq > radius**2:
            return []
        dy = np.sqrt(radius**2 - dx_sq)
        return [cy - dy, cy + dy]

    # Robust polygon intersections with a vertical line, returning inside intervals
    def _vertical_intervals_from_polygon(self, x0: float, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        yints: List[float] = []
        n = len(pts)
        if n < 2:
            return []
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            dx = x2 - x1
            if abs(dx) < self._EPS:
                continue
            xmin, xmax = (x1, x2) if x1 <= x2 else (x2, x1)
            if (x0 > xmin + self._EPS) and (x0 < xmax - self._EPS):
                t = (x0 - x1) / dx
                y = y1 + t * (y2 - y1)
                yints.append(y)
        yints.sort()
        intervals: List[Tuple[float, float]] = []
        for i in range(0, len(yints) - 1, 2):
            intervals.append((yints[i], yints[i + 1]))
        return intervals

    def _vertical_intervals_from_circle(self, x0: float, center: Tuple[float, float], r: float) -> List[Tuple[float, float]]:
        cx, cy = center
        dx = x0 - cx
        if abs(dx) >= r - self._EPS:
            return []
        h = math.sqrt(max(r * r - dx * dx, 0.0))
        return [(cy - h, cy + h)]

    def _merge_intervals(self, intervals: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not intervals:
            return []
        intervals = sorted(intervals, key=lambda ab: ab[0])
        merged = [list(intervals[0])]
        for a, b in intervals[1:]:
            if a <= merged[-1][1] + self._EPS:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        return [(a, b) for a, b in merged]

    def _subtract_intervals(self, base: List[Tuple[float, float]], subtract: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        subtract = self._merge_intervals(subtract)
        out: List[Tuple[float, float]] = []
        for a0, a1 in base:
            cur = a0
            for b0, b1 in subtract:
                if b1 <= cur + self._EPS:
                    continue
                if b0 >= a1 - self._EPS:
                    break
                if b0 > cur + self._EPS:
                    out.append((cur, min(b0, a1)))
                cur = max(cur, b1)
                if cur >= a1 - self._EPS:
                    break
            if cur < a1 - self._EPS:
                out.append((cur, a1))
        return [(s, e) for (s, e) in out if e - s > 10 * self._EPS]

    def _arena_points(self) -> List[Tuple[float, float]]:
        return [] if not self.arena_data else [(p.x, p.y) for p in self.arena_data.points]

    def _obstacle_primitives(self) -> Tuple[List[List[Tuple[float, float]]], List[Tuple[Tuple[float, float], float]]]:
        polys: List[List[Tuple[float, float]]] = []
        circles: List[Tuple[Tuple[float, float], float]] = []
        if not self.obstacles_data:
            return polys, circles
        for obs in self.obstacles_data.obstacles:
            if obs.radius and obs.radius > 0.0:
                if obs.polygon.points:
                    c = obs.polygon.points[0]
                    circles.append(((c.x, c.y), float(obs.radius)))
            else:
                pts = [(p.x, p.y) for p in obs.polygon.points]
                if len(pts) >= 3:
                    polys.append(pts)
        return polys, circles

    def _free_vertical_intervals_at_x(self, x0: float, arena_pts: List[Tuple[float, float]],
                                      obs_polys: List[List[Tuple[float, float]]],
                                      circles: List[Tuple[Tuple[float, float], float]]) -> List[Tuple[float, float]]:
        arena_ints = self._vertical_intervals_from_polygon(x0, arena_pts)
        blocked: List[Tuple[float, float]] = []
        for poly in obs_polys:
            blocked.extend(self._vertical_intervals_from_polygon(x0, poly))
        for (c, r) in circles:
            blocked.extend(self._vertical_intervals_from_circle(x0, c, r))
        return self._subtract_intervals(arena_ints, blocked)

    def _unique_sorted(self, xs: List[float]) -> List[float]:
        xs = sorted(xs)
        out: List[float] = []
        for x in xs:
            if not out or abs(out[-1] - x) > 1e-6:
                out.append(x)
        return out

    def _polygon_area_centroid(self, poly: List[Tuple[float, float]]) -> Tuple[float, float]:
        n = len(poly)
        if n < 3:
            xs = [p[0] for p in poly]
            ys = [p[1] for p in poly]
            return (sum(xs) / len(xs), sum(ys) / len(ys))
        A = 0.0
        Cx = 0.0
        Cy = 0.0
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            cross = x1 * y2 - x2 * y1
            A += cross
            Cx += (x1 + x2) * cross
            Cy += (y1 + y2) * cross
        A *= 0.5
        if abs(A) < self._EPS:
            xs = [p[0] for p in poly]
            ys = [p[1] for p in poly]
            return (sum(xs) / len(xs), sum(ys) / len(ys))
        return (Cx / (6.0 * A), Cy / (6.0 * A))

    # --- CELL DECOMPOSITION LOGIC ---

    def compute_cells_and_centroids(self) -> List[Tuple[List[Tuple[float, float]], Tuple[float, float]]]:
        """Build vertical slabs and compute trapezoidal cells and centroids."""
        arena_pts = self._arena_points()
        if not arena_pts:
            return []
        obs_polys, circles = self._obstacle_primitives()

        # Collect critical x's (arena bounds, obstacle vertices, circle tangency x)
        xs_arena = [p[0] for p in arena_pts]
        xmin, xmax = min(xs_arena), max(xs_arena)
        x_crit: List[float] = [xmin, xmax]
        for poly in obs_polys:
            x_crit.extend([x for (x, _) in poly])
        for (c, r) in circles:
            x_crit.append(c[0] - r)
            x_crit.append(c[0] + r)
        xs = self._unique_sorted(x_crit)

        cells: List[List[Tuple[float, float]]] = []
        centroids: List[Tuple[float, float]] = []

        for i in range(len(xs) - 1):
            xL = xs[i] + 1e-7
            xR = xs[i + 1] - 1e-7
            if xR <= xL + self._EPS:
                continue
            freeL = self._free_vertical_intervals_at_x(xL, arena_pts, obs_polys, circles)
            freeR = self._free_vertical_intervals_at_x(xR, arena_pts, obs_polys, circles)

            # Pair intervals by index; if counts mismatch, try greedy overlap pairing
            pairs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
            if len(freeL) == len(freeR):
                for j in range(len(freeL)):
                    pairs.append((freeL[j], freeR[j]))
            else:
                j = 0
                usedR = set()
                for a0, a1 in freeL:
                    mid = (a0 + a1) / 2.0
                    # choose closest right interval by midpoint distance
                    best_k = None
                    best_d = float('inf')
                    for k, (b0, b1) in enumerate(freeR):
                        if k in usedR:
                            continue
                        midR = (b0 + b1) / 2.0
                        d = abs(mid - midR)
                        if d < best_d:
                            best_d = d
                            best_k = k
                    if best_k is not None:
                        pairs.append(((a0, a1), freeR[best_k]))
                        usedR.add(best_k)

            for (a0, a1), (b0, b1) in pairs:
                poly = [(xL, a0), (xL, a1), (xR, b1), (xR, b0)]
                cx, cy = self._polygon_area_centroid(poly)
                cells.append(poly)
                centroids.append((cx, cy))

        self.cells = cells
        self.cell_centroids = centroids
        return [(cells[k], centroids[k]) for k in range(len(cells))]

    def get_cell_centroids(self) -> List[Tuple[float, float]]:
        """Returns the list of centroids for all cells. Call after compute_cells_and_centroids/plot."""
        return list(self.cell_centroids)

    def _draw_vertical_segments_at_crit_x(self, ax):
        """Draws vertical free segments at each critical x (obstacle vertices and circle tangency)."""
        arena_pts = self._arena_points()
        if not arena_pts:
            return
        obs_polys, circles = self._obstacle_primitives()
        xs_arena = [p[0] for p in arena_pts]
        xmin, xmax = min(xs_arena), max(xs_arena)
        x_crit: List[float] = []
        for poly in obs_polys:
            x_crit.extend([x for (x, _) in poly])
        for (c, r) in circles:
            x_crit.append(c[0] - r)
            x_crit.append(c[0] + r)
        for xv in self._unique_sorted(x_crit):
            xq = xv + 1e-7
            free_ints = self._free_vertical_intervals_at_x(xq, arena_pts, obs_polys, circles)
            for (ya, yb) in free_ints:
                ax.plot([xv, xv], [ya, yb], color='black', linewidth=1.2, alpha=0.8)

    # --- CELL DECOMPOSITION LOGIC ---

    def perform_cell_decomposition(self, ax):
        """Keeps the projection lines from critical points (existing behavior)."""
        if not self.arena_data or not self.obstacles_data:
            return
        self.get_logger().info('Starting cell decomposition by projecting from critical points...')
        
        # Get arena points for boundary checks
        arena_points = [(p.x, p.y) for p in self.arena_data.points]
        
        # Step 1: Collect all critical (x, y) points from obstacles
        critical_points = []
        for obstacle in self.obstacles_data.obstacles:
            if obstacle.radius == 0.0: # Polygon
                for p in obstacle.polygon.points:
                    critical_points.append((p.x, p.y))
            else: # Circle
                center_pt = obstacle.polygon.points[0]
                center = (center_pt.x, center_pt.y)
                radius = obstacle.radius
                # Add left and right tangent points
                critical_points.append((center[0] - radius, center[1]))
                critical_points.append((center[0] + radius, center[1]))

        # Step 2: For each critical point, project lines up and down
        for x_crit, y_crit in critical_points:
            
            # Find all intersections at this x-coordinate
            all_intersections_y = []
            # Arena intersections
            for i in range(len(arena_points)):
                y_val = self.get_vertical_line_segment_intersection(x_crit, arena_points[i], arena_points[(i + 1) % len(arena_points)])
                if y_val is not None:
                    all_intersections_y.append(y_val)
            # Obstacle intersections
            for obs in self.obstacles_data.obstacles:
                if obs.radius == 0.0:
                    obs_pts = [(p.x, p.y) for p in obs.polygon.points]
                    for i in range(len(obs_pts)):
                        y_val = self.get_vertical_line_segment_intersection(x_crit, obs_pts[i], obs_pts[(i + 1) % len(obs_pts)])
                        if y_val is not None:
                            all_intersections_y.append(y_val)
                else:
                    center = (obs.polygon.points[0].x, obs.polygon.points[0].y)
                    y_vals = self.get_vertical_line_circle_intersections(x_crit, center, obs.radius)
                    all_intersections_y.extend(y_vals)

            # Step 3: Find the closest boundary above and below
            y_above = [y for y in all_intersections_y if y > y_crit + 1e-6]
            y_below = [y for y in all_intersections_y if y < y_crit - 1e-6]

            # Project upwards
            if y_above:
                y_upper_limit = min(y_above)
                ax.plot([x_crit, x_crit], [y_crit, y_upper_limit], color='gray', linestyle='--', linewidth=1, alpha=0.8)

            # Project downwards
            if y_below:
                y_lower_limit = max(y_below)
                ax.plot([x_crit, x_crit], [y_crit, y_lower_limit], color='gray', linestyle='--', linewidth=1, alpha=0.8)
                
        self.get_logger().info('Cell decomposition complete.')

    # --- GRAPH CONSTRUCTION AND PATH PLANNING ---

    def build_adjacency_graph(self):
        """Build enhanced adjacency graph between cell centroids with improved edge weights."""
        if not self.cell_centroids:
            return
        
        self.adjacency_graph = {i: [] for i in range(len(self.cell_centroids))}
        
        for i in range(len(self.cell_centroids)):
            for j in range(i + 1, len(self.cell_centroids)):
                if self.are_cells_adjacent(i, j):
                    # Enhanced edge weight calculation
                    distance = self.euclidean_distance(self.cell_centroids[i], self.cell_centroids[j])
                    
                    # Add penalty for direction changes (favor straight paths)
                    direction_penalty = self.calculate_direction_penalty(i, j)
                    
                    # Add penalty based on proximity to obstacles
                    obstacle_penalty = self.calculate_obstacle_proximity_penalty(i, j)
                    
                    # Final weight combines distance with penalties
                    total_weight = distance * (1.0 + direction_penalty + obstacle_penalty)
                    
                    self.adjacency_graph[i].append((j, total_weight))
                    self.adjacency_graph[j].append((i, total_weight))
        
        self.get_logger().info(f'Built enhanced adjacency graph with {len(self.adjacency_graph)} nodes')

    def calculate_direction_penalty(self, cell_i: int, cell_j: int) -> float:
        """Calculate penalty based on direction change to favor straighter paths."""
        # This is a simplified version - in a full implementation you'd consider
        # the previous direction in the path
        return 0.0  # For now, no direction penalty

    def calculate_obstacle_proximity_penalty(self, cell_i: int, cell_j: int) -> float:
        """Add penalty for paths that go close to obstacles."""
        midpoint = (
            (self.cell_centroids[cell_i][0] + self.cell_centroids[cell_j][0]) / 2,
            (self.cell_centroids[cell_i][1] + self.cell_centroids[cell_j][1]) / 2
        )
        
        min_obstacle_distance = float('inf')
        obs_polys, circles = self._obstacle_primitives()
        
        # Check distance to polygon obstacles
        for poly in obs_polys:
            for vertex in poly:
                dist = self.euclidean_distance(midpoint, vertex)
                min_obstacle_distance = min(min_obstacle_distance, dist)
        
        # Check distance to circular obstacles
        for center, radius in circles:
            # Distance to obstacle surface
            dist_to_center = self.euclidean_distance(midpoint, center)
            dist_to_surface = max(0, dist_to_center - radius)
            min_obstacle_distance = min(min_obstacle_distance, dist_to_surface)
        
        # Apply penalty - closer to obstacles = higher penalty
        if min_obstacle_distance < 1.0:  # Within 1 meter of obstacle
            penalty = max(0, (1.0 - min_obstacle_distance) * 0.5)  # Up to 50% penalty
            return penalty
        
        return 0.0

    def are_cells_adjacent(self, cell_i: int, cell_j: int) -> bool:
        """Check if two cells are adjacent by testing if the line between centroids is collision-free."""
        if cell_i >= len(self.cells) or cell_j >= len(self.cells):
            return False
        
        centroid_i = self.cell_centroids[cell_i]
        centroid_j = self.cell_centroids[cell_j]
        
        # Sample points along the line between centroids
        num_samples = 20
        for k in range(num_samples + 1):
            t = k / num_samples
            x = centroid_i[0] + t * (centroid_j[0] - centroid_i[0])
            y = centroid_i[1] + t * (centroid_j[1] - centroid_i[1])
            
            if not self.is_point_free((x, y)):
                return False
        
        return True

    def is_point_free(self, point: Tuple[float, float]) -> bool:
        """Check if a point is in free space (inside arena and not in obstacles)."""
        # Check if point is inside arena
        arena_pts = self._arena_points()
        if not arena_pts or not self.is_point_in_polygon(point, arena_pts):
            return False
        
        # Check if point is inside any obstacle
        obs_polys, circles = self._obstacle_primitives()
        
        for poly in obs_polys:
            if self.is_point_in_polygon(point, poly):
                return False
        
        for center, radius in circles:
            if self.is_point_in_circle(point, center, radius):
                return False
        
        return True

    def euclidean_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def find_closest_cell(self, point: Tuple[float, float]) -> int:
        """Find the closest cell centroid to a given point."""
        if not self.cell_centroids:
            return -1
        
        min_distance = float('inf')
        closest_cell = -1
        
        for i, centroid in enumerate(self.cell_centroids):
            distance = self.euclidean_distance(point, centroid)
            if distance < min_distance:
                min_distance = distance
                closest_cell = i
        
        return closest_cell

    def dijkstra_path(self, start_point: Tuple[float, float], goal_point: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Find optimized path from start_point to goal_point using Dijkstra's algorithm.
        Returns list of waypoints forming the path.
        """
        if not self.cell_centroids or not self.adjacency_graph:
            self.get_logger().warn('No cells or graph available for path planning')
            return []
        
        # Find closest cells to start and goal
        start_cell = self.find_closest_cell(start_point)
        goal_cell = self.find_closest_cell(goal_point)
        
        if start_cell == -1 or goal_cell == -1:
            self.get_logger().warn('Could not find valid start or goal cell')
            return []
        
        self.get_logger().info(f'Path planning: Start cell {start_cell}, Goal cell {goal_cell}')
        
        # If start and goal are in the same cell, check direct path
        if start_cell == goal_cell:
            if self.is_line_collision_free(start_point, goal_point):
                return [start_point, goal_point]
        
        # Enhanced Dijkstra's algorithm with A* heuristic
        distances = {i: float('inf') for i in range(len(self.cell_centroids))}
        distances[start_cell] = 0
        previous = {i: None for i in range(len(self.cell_centroids))}
        # Use A* heuristic: f(n) = g(n) + h(n)
        goal_centroid = self.cell_centroids[goal_cell]
        start_heuristic = self.euclidean_distance(self.cell_centroids[start_cell], goal_centroid)
        unvisited = [(start_heuristic, 0, start_cell)]  # (f_score, g_score, node)
        visited = set()
        
        while unvisited:
            f_score, g_score, current_cell = heapq.heappop(unvisited)
            
            if current_cell in visited:
                continue
            
            visited.add(current_cell)
            
            if current_cell == goal_cell:
                break
            
            # Check all neighbors
            for neighbor_cell, edge_weight in self.adjacency_graph[current_cell]:
                if neighbor_cell in visited:
                    continue
                
                new_g_score = g_score + edge_weight
                if new_g_score < distances[neighbor_cell]:
                    distances[neighbor_cell] = new_g_score
                    previous[neighbor_cell] = current_cell
                    # A* heuristic
                    h_score = self.euclidean_distance(self.cell_centroids[neighbor_cell], goal_centroid)
                    f_score = new_g_score + h_score
                    heapq.heappush(unvisited, (f_score, new_g_score, neighbor_cell))
        
        # Reconstruct path
        if distances[goal_cell] == float('inf'):
            self.get_logger().warn('No path found between start and goal')
            return []
        
        path_cells = []
        current = goal_cell
        while current is not None:
            path_cells.append(current)
            current = previous[current]
        
        path_cells.reverse()
        
        # Convert cell indices to centroids
        raw_path = [self.cell_centroids[cell_idx] for cell_idx in path_cells]
        
        # Optimize the path
        optimized_path = self.optimize_path([start_point] + raw_path + [goal_point])
        
        self.get_logger().info(f'Found path with {len(optimized_path)} waypoints (reduced from {len(raw_path) + 2})')
        return optimized_path

    def is_line_collision_free(self, p1: Tuple[float, float], p2: Tuple[float, float], num_samples: int = 50) -> bool:
        """Check if a straight line between two points is collision-free."""
        for i in range(num_samples + 1):
            t = i / num_samples
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self.is_point_free((x, y)):
                return False
        return True

    def optimize_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Optimize path using multiple techniques:
        1. Line-of-sight optimization (remove unnecessary waypoints)
        2. Corner cutting (smooth sharp turns)
        3. Redundant point removal
        """
        if len(path) <= 2:
            return path
        
        # Step 1: Line-of-sight optimization
        optimized = self.line_of_sight_optimization(path)
        
        # Step 2: Corner cutting for smoother paths
        optimized = self.corner_cutting_optimization(optimized)
        
        # Step 3: Remove redundant points
        optimized = self.remove_redundant_points(optimized)
        
        return optimized

    def line_of_sight_optimization(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Remove waypoints that can be bypassed with direct line-of-sight."""
        if len(path) <= 2:
            return path
        
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Find the farthest point we can reach directly
            farthest = i + 1
            for j in range(i + 2, len(path)):
                if self.is_line_collision_free(path[i], path[j]):
                    farthest = j
                else:
                    break
            
            optimized.append(path[farthest])
            i = farthest
        
        return optimized

    def corner_cutting_optimization(self, path: List[Tuple[float, float]], cut_distance: float = 0.3) -> List[Tuple[float, float]]:
        """Smooth sharp corners by cutting them."""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            p_prev = path[i - 1]
            p_curr = path[i]
            p_next = path[i + 1]
            
            # Calculate vectors
            v1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            
            # Normalize vectors
            len_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len_v1 > self._EPS and len_v2 > self._EPS:
                v1_norm = (v1[0] / len_v1, v1[1] / len_v1)
                v2_norm = (v2[0] / len_v2, v2[1] / len_v2)
                
                # Calculate angle between vectors
                dot_product = v1_norm[0] * v2_norm[0] + v1_norm[1] * v2_norm[1]
                dot_product = max(-1.0, min(1.0, dot_product))  # Clamp to avoid numerical errors
                angle = math.acos(dot_product)
                
                # If angle is sharp (< 150 degrees), try to cut the corner
                if angle < math.radians(150):
                    cut_dist = min(cut_distance, len_v1 * 0.3, len_v2 * 0.3)
                    
                    # Points for corner cutting
                    cut_point1 = (p_curr[0] - v1_norm[0] * cut_dist, p_curr[1] - v1_norm[1] * cut_dist)
                    cut_point2 = (p_curr[0] + v2_norm[0] * cut_dist, p_curr[1] + v2_norm[1] * cut_dist)
                    
                    # Check if cut is collision-free
                    if (self.is_point_free(cut_point1) and self.is_point_free(cut_point2) and 
                        self.is_line_collision_free(cut_point1, cut_point2)):
                        smoothed.extend([cut_point1, cut_point2])
                    else:
                        smoothed.append(p_curr)
                else:
                    smoothed.append(p_curr)
            else:
                smoothed.append(p_curr)
        
        smoothed.append(path[-1])
        return smoothed

    def remove_redundant_points(self, path: List[Tuple[float, float]], min_distance: float = 0.1) -> List[Tuple[float, float]]:
        """Remove points that are too close to each other."""
        if len(path) <= 2:
            return path
        
        filtered = [path[0]]
        
        for i in range(1, len(path)):
            distance = self.euclidean_distance(filtered[-1], path[i])
            if distance > min_distance or i == len(path) - 1:  # Always keep the last point
                filtered.append(path[i])
        
        return filtered

    def calculate_path_metrics(self, path: List[Tuple[float, float]]) -> Tuple[float, float, int]:
        """Calculate path metrics: total distance, average turn angle, number of waypoints."""
        if len(path) < 2:
            return 0.0, 0.0, len(path)
        
        # Total distance
        total_distance = 0.0
        for i in range(len(path) - 1):
            total_distance += self.euclidean_distance(path[i], path[i + 1])
        
        # Average turn angle
        total_turn_angle = 0.0
        num_turns = 0
        
        for i in range(1, len(path) - 1):
            p_prev = path[i - 1]
            p_curr = path[i]
            p_next = path[i + 1]
            
            # Calculate vectors
            v1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            
            # Calculate turn angle
            len_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len_v1 > self._EPS and len_v2 > self._EPS:
                v1_norm = (v1[0] / len_v1, v1[1] / len_v1)
                v2_norm = (v2[0] / len_v2, v2[1] / len_v2)
                
                dot_product = v1_norm[0] * v2_norm[0] + v1_norm[1] * v2_norm[1]
                dot_product = max(-1.0, min(1.0, dot_product))
                turn_angle = math.acos(dot_product)
                total_turn_angle += turn_angle
                num_turns += 1
        
        avg_turn_angle = total_turn_angle / num_turns if num_turns > 0 else 0.0
        
        return total_distance, math.degrees(avg_turn_angle), len(path)

    def plot_path(self, ax, path: List[Tuple[float, float]], color: str, label: str, linewidth: float = 3):
        """Plot a path on the given axes."""
        if len(path) < 2:
            return
        
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color=color, linewidth=linewidth, marker='o', markersize=6, 
                label=label, alpha=0.8, markerfacecolor='white', markeredgecolor=color)

    # --- PLOTTING ---
    
    def plot_map_with_decomposition(self):
        fig, ax = plt.subplots(figsize=(14, 12))
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title('Cell Decomposition with Dijkstra Path Planning', fontsize=16)
        
        # Plot arena boundary
        if self.arena_data:
            arena_points = [(p.x, p.y) for p in self.arena_data.points]
            arena_polygon = MatplotlibPolygon(arena_points, fill=True, facecolor='lightcyan', 
                                              edgecolor='blue', linewidth=3, label='Arena Boundary')
            ax.add_patch(arena_polygon)
        
        # Plot obstacles
        if self.obstacles_data:
            for i, obstacle in enumerate(self.obstacles_data.obstacles):
                if obstacle.radius == 0.0:  # Polygon obstacle
                    obs_points = [(p.x, p.y) for p in obstacle.polygon.points]
                    obs_polygon = MatplotlibPolygon(obs_points, facecolor='salmon', alpha=0.9,
                                                    edgecolor='darkred', linewidth=2)
                    ax.add_patch(obs_polygon)
                else:  # Circular obstacle
                    center = obstacle.polygon.points[0]
                    circle = patches.Circle((center.x, center.y), obstacle.radius,
                                            facecolor='salmon', alpha=0.9, edgecolor='darkred')
                    ax.add_patch(circle)

        # --- PERFORM DECOMPOSITION HERE ---
        self.perform_cell_decomposition(ax)
        # NEW: draw full vertical segments at critical x (stopping at objects/border)
        self._draw_vertical_segments_at_crit_x(ax)
        # NEW: compute cells and centroids and label them
        cells_and_centroids = self.compute_cells_and_centroids()
        for idx, (_, (cx, cy)) in enumerate(cells_and_centroids, start=1):
            ax.text(cx, cy, str(idx), ha='center', va='center', fontsize=10, color='k', 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
        if cells_and_centroids:
            self.get_logger().info(f'Computed {len(cells_and_centroids)} cells.')
            for i, (_, c) in enumerate(cells_and_centroids, start=1):
                self.get_logger().info(f'Cell {i} centroid: ({c[0]:.3f}, {c[1]:.3f})')
        
        # Build adjacency graph for path planning
        self.build_adjacency_graph()
        
        # Plot gates and find paths
        gate_positions = []
        if self.gates_data:
            for i, gate in enumerate(self.gates_data.poses):
                x, y = gate.position.x, gate.position.y
                gate_positions.append((x, y))
                quat = gate.orientation
                yaw = np.arctan2(2*(quat.w*quat.z + quat.x*quat.y), 1 - 2*(quat.y*quat.y + quat.z*quat.z))
                ax.plot(x, y, 'go', markersize=12, label='Gates' if i == 0 else "")
                dx, dy = 0.5 * np.cos(yaw), 0.5 * np.sin(yaw)
                ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='green', ec='green')
        
        # Get robot positions
        robot1_pos = robot2_pos = None
        if self.pos1_data:
            pos = self.pos1_data.pose.pose.position
            robot1_pos = (pos.x, pos.y)
            ax.plot(pos.x, pos.y, 'bo', markersize=12, label='Robot 1 Position', markeredgecolor='k')
        
        if self.pos2_data:
            pos = self.pos2_data.pose.pose.position
            robot2_pos = (pos.x, pos.y)
            ax.plot(pos.x, pos.y, 'mo', markersize=12, label='Robot 2 Position', markeredgecolor='k')
        
        # Generate optimized paths using Dijkstra if we have robot positions and gates
        if robot1_pos and gate_positions and self.cell_centroids:
            # Find path from robot 1 to closest gate
            closest_gate_dist = float('inf')
            closest_gate = None
            for gate_pos in gate_positions:
                dist = self.euclidean_distance(robot1_pos, gate_pos)
                if dist < closest_gate_dist:
                    closest_gate_dist = dist
                    closest_gate = gate_pos
            
            if closest_gate:
                path1 = self.dijkstra_path(robot1_pos, closest_gate)
                if path1:
                    self.plot_path(ax, path1, 'blue', 'Robot 1 Optimized Path', linewidth=4)
                    # Calculate and log path metrics
                    distance, avg_turn, waypoints = self.calculate_path_metrics(path1)
                    self.get_logger().info(f'Robot 1 optimized path: {waypoints} waypoints, '
                                         f'total distance: {distance:.3f}m, avg turn angle: {avg_turn:.1f}°')
        
        if robot2_pos and gate_positions and self.cell_centroids:
            # Find path from robot 2 to closest gate
            closest_gate_dist = float('inf')
            closest_gate = None
            for gate_pos in gate_positions:
                dist = self.euclidean_distance(robot2_pos, gate_pos)
                if dist < closest_gate_dist:
                    closest_gate_dist = dist
                    closest_gate = gate_pos
            
            if closest_gate:
                path2 = self.dijkstra_path(robot2_pos, closest_gate)
                if path2:
                    self.plot_path(ax, path2, 'magenta', 'Robot 2 Optimized Path', linewidth=4)
                    # Calculate and log path metrics
                    distance, avg_turn, waypoints = self.calculate_path_metrics(path2)
                    self.get_logger().info(f'Robot 2 optimized path: {waypoints} waypoints, '
                                         f'total distance: {distance:.3f}m, avg turn angle: {avg_turn:.1f}°')
        
        # Draw connections between adjacent cells for visualization
        if self.adjacency_graph and len(self.cell_centroids) > 1:
            for cell_i, neighbors in self.adjacency_graph.items():
                for neighbor_j, weight in neighbors:
                    if cell_i < neighbor_j:  # Draw each edge only once
                        c1 = self.cell_centroids[cell_i]
                        c2 = self.cell_centroids[neighbor_j]
                        ax.plot([c1[0], c2[0]], [c1[1], c2[1]], 'k-', alpha=0.3, linewidth=0.8)
        
        # Set axis limits
        if self.arena_data:
            x_coords = [p.x for p in self.arena_data.points]
            y_coords = [p.y for p in self.arena_data.points]
            margin = 1.0
            ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
            ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)
        
        ax.legend()
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        
        plt.tight_layout()
        plt.show()
        
        self.get_logger().info('Map with decomposition and paths plotted successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = CellDecomposition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()