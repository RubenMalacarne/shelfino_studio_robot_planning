#include "planning_pkg/comb_path.hpp"
#include "planning_pkg/common_function.hpp"
#include <algorithm> // std::min, std::max, std::clamp
#include <cmath>     // std::sqrt
#include <limits>    // std::numeric_limits
#include <random>    // random_device, mt19937, uniform_real_distribution
#include <utility>   // std::move
#include <vector>
#include <tuple>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>


// ============================================================================
// planning_pkg::CombPathGenerator
// ============================================================================
namespace planning_pkg
{

    CombPathGenerator::CombPathGenerator(std::string default_frame_id,
                                         double default_step_size)
        : default_frame_id_(std::move(default_frame_id)),
          default_step_size_(default_step_size)
    {
    }
    nav_msgs::msg::Path CombPathGenerator::generate(
        const std::vector<std::pair<double, double>> &waypoints,
        const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
        const geometry_msgs::msg::Polygon &arena)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = rclcpp::Clock().now();
        path.header.frame_id = default_frame_id_;

        if (waypoints.size() < 2)
        {
            return path;
        }

        const auto start = waypoints.front();
        const auto goal = waypoints.back();

        // Verifica che start e goal siano validi
        if (!CommonFunction::point_is_valid(start.first, start.second, arena, obstacles, 0.15))
        {
            RCLCPP_ERROR(rclcpp::get_logger("comb_path_generator"),
                         "Punto di start non valido");
            return path;
        }

        if (!CommonFunction::point_is_valid(goal.first, goal.second, arena, obstacles, 0.15))
        {
            RCLCPP_ERROR(rclcpp::get_logger("comb_path_generator"),
                         "Punto di goal non valido");
            return path;
        }

        // 1. Genera la lista di punti (vertici di ostacoli e arena)
        std::vector<std::pair<double, double>> pointslist = get_pointlist(obstacles, arena);

        RCLCPP_INFO(rclcpp::get_logger("comb_path_generator"),
                    "Generati %zu punti da ostacoli e arena", pointslist.size());

        // 2. Genera le linee orizzontali
        std::vector<HorizontalLine> horizontal_lines = set_vertical_line(pointslist, obstacles, arena);

        // 3. Genera i punti lungo le linee orizzontali
        std::vector<std::pair<double, double>> points_line;
        for (const auto &line : horizontal_lines)
        {
            auto line_points = set_point_in_vertical_line(line, 1.0);
            points_line.insert(points_line.end(), line_points.begin(), line_points.end());
        }

        // 4. Genera le celle tra le linee verticali
        std::vector<planning_pkg::Cell> cells = get_cells_btw_vlines(horizontal_lines);
        std::vector<std::pair<double, double>> points_centroids;
        for (const auto &cell : cells)
        {
            auto centroid = get_cell_centroid(cell);
            points_centroids.push_back(centroid);
        }

        RCLCPP_INFO(rclcpp::get_logger("comb_path_generator"),
                    "Generate %zu linee orizzontali, %zu punti, %zu centroidi",
                    horizontal_lines.size(), points_line.size(), points_centroids.size());

        // 5. Crea il grafo con archi
        std::vector<std::vector<int>> adjacency = get_arc(horizontal_lines, points_line, points_centroids);

        // 6. Converti i punti in formato geometry_msgs::msg::Point per Dijkstra
        std::vector<geometry_msgs::msg::Point> temp_points;

        // Aggiungi i punti delle linee
        for (const auto &point : points_line)
        {
            geometry_msgs::msg::Point p;
            p.x = point.first;
            p.y = point.second;
            p.z = 0.0;
            temp_points.push_back(p);
        }

        // Aggiungi i centroidi
        for (const auto &centroid : points_centroids)
        {
            geometry_msgs::msg::Point p;
            p.x = centroid.first;
            p.y = centroid.second;
            p.z = 0.0;
            temp_points.push_back(p);
        }

        // 7. Aggiungi start e goal al grafo temporaneo
        geometry_msgs::msg::Point start_point;
        start_point.x = start.first;
        start_point.y = start.second;
        start_point.z = 0.0;

        geometry_msgs::msg::Point goal_point;
        goal_point.x = goal.first;
        goal_point.y = goal.second;
        goal_point.z = 0.0;

        temp_points.push_back(start_point);
        temp_points.push_back(goal_point);

        // Ridimensiona la lista di adiacenza
        adjacency.resize(temp_points.size());

        int start_node = static_cast<int>(temp_points.size() - 2);
        int goal_node = static_cast<int>(temp_points.size() - 1);

        // 8. Connetti start e goal ai nodi più vicini
        const double max_connection_distance = 2.0;
        const size_t max_connections = 10;

        // Connetti start
        std::vector<std::pair<double, int>> start_distances;
        for (size_t i = 0; i < temp_points.size() - 2; ++i)
        {
            double dx = start_point.x - temp_points[i].x;
            double dy = start_point.y - temp_points[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= max_connection_distance)
            {
                start_distances.emplace_back(dist, static_cast<int>(i));
            }
        }

        std::sort(start_distances.begin(), start_distances.end());
        size_t start_connections = std::min(max_connections, start_distances.size());

        for (size_t i = 0; i < start_connections; ++i)
        {
            int neighbor = start_distances[i].second;
            if (CommonFunction::point_is_valid((start_point.x + temp_points[neighbor].x) / 2.0,
                               (start_point.y + temp_points[neighbor].y) / 2.0,
                               arena, obstacles, 0.1))
            {
                adjacency[start_node].push_back(neighbor);
                adjacency[neighbor].push_back(start_node);
            }
        }

        // Connetti goal
        std::vector<std::pair<double, int>> goal_distances;
        for (size_t i = 0; i < temp_points.size() - 2; ++i)
        {
            double dx = goal_point.x - temp_points[i].x;
            double dy = goal_point.y - temp_points[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= max_connection_distance)
            {
                goal_distances.emplace_back(dist, static_cast<int>(i));
            }
        }

        std::sort(goal_distances.begin(), goal_distances.end());
        size_t goal_connections = std::min(max_connections, goal_distances.size());

        for (size_t i = 0; i < goal_connections; ++i)
        {
            int neighbor = goal_distances[i].second;
            if (CommonFunction::point_is_valid((goal_point.x + temp_points[neighbor].x) / 2.0,
                               (goal_point.y + temp_points[neighbor].y) / 2.0,
                               arena, obstacles, 0.1))
            {
                adjacency[goal_node].push_back(neighbor);
                adjacency[neighbor].push_back(goal_node);
            }
        }

        // Verifica connessione diretta start-goal
        double dx = goal_point.x - start_point.x;
        double dy = goal_point.y - start_point.y;
        double direct_dist = std::sqrt(dx * dx + dy * dy);
        if (direct_dist <= max_connection_distance &&
            CommonFunction::point_is_valid((start_point.x + goal_point.x) / 2.0,
                           (start_point.y + goal_point.y) / 2.0,
                           arena, obstacles, 0.1))
        {
            adjacency[start_node].push_back(goal_node);
            adjacency[goal_node].push_back(start_node);
        }

        // 9. Esegui Dijkstra
        std::vector<int> path_indices = dijkstra_shortest_path_temp(start_node, goal_node, temp_points, adjacency);

        if (path_indices.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("comb_path_generator"),
                         "Nessun path trovato nel grafo Comb");
            return path;
        }

        std::vector<geometry_msgs::msg::Point> original_path_points;
        for (int idx : path_indices)
        {
            if (idx >= 0 && static_cast<size_t>(idx) < temp_points.size())
            {
                original_path_points.push_back(temp_points[idx]);
            }
        }

        std::vector<geometry_msgs::msg::Point> optimized_path_points =
            CommonFunction::optimize_path_with_raycasting(original_path_points, obstacles, arena, 0.15, 0.05);

        for (const auto &point : optimized_path_points)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path.header;

            pose_stamped.pose.position = point;
            pose_stamped.pose.orientation.w = 1.0;

            path.poses.push_back(pose_stamped);
        }
        // // 10. Converti il risultato in nav_msgs::msg::Path
        // for (int idx : path_indices)
        // {
        //     if (idx >= 0 && static_cast<size_t>(idx) < temp_points.size())
        //     {
        //         geometry_msgs::msg::PoseStamped pose_stamped;
        //         pose_stamped.header = path.header;

        //         pose_stamped.pose.position = temp_points[idx];
        //         pose_stamped.pose.orientation.w = 1.0;

        //         path.poses.push_back(pose_stamped);
        //     }
        // }

    RCLCPP_INFO(rclcpp::get_logger("comb_path_generator"),
                "Path generato con %zu waypoints", path.poses.size());

    return path;
}

std::vector<std::pair<double, double>> CombPathGenerator::get_pointlist(
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    std::vector<std::pair<double, double>> points;

    // Aggiungi tutti i vertici dell'arena
    for (const auto &point : arena.points)
    {
        points.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
    }

    // Aggiungi tutti i vertici degli ostacoli
    for (const auto &obstacle : obstacles.obstacles)
    {
        if (obstacle.radius > 0.0)
        {
            // Per ostacoli circolari, aggiungi il centro e alcuni punti sulla circonferenza
            double center_x = static_cast<double>(obstacle.polygon.points[0].x);
            double center_y = static_cast<double>(obstacle.polygon.points[0].y);

            // Aggiungi il centro
            points.emplace_back(center_x, center_y);

            // Aggiungi punti cardinali della circonferenza
            points.emplace_back(center_x + obstacle.radius, center_y); // Est
            points.emplace_back(center_x - obstacle.radius, center_y); // Ovest
            points.emplace_back(center_x, center_y + obstacle.radius); // Nord
            points.emplace_back(center_x, center_y - obstacle.radius); // Sud
        }
        else
        {
            // Per ostacoli poligonali, aggiungi tutti i vertici
            for (const auto &point : obstacle.polygon.points)
            {
                points.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
            }
        }
    }

    // Rimuovi duplicati (opzionale, ma utile)
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());

    return points;
}

std::vector<HorizontalLine> CombPathGenerator::set_vertical_line(
    const std::vector<std::pair<double, double>> &points,
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena) const
{
    std::vector<HorizontalLine> horizontal_lines;

    // Per ogni punto, crea una linea orizzontale passante per quel punto
    for (const auto &point : points)
    {
        double y = point.second;

        // Trova i limiti dell'arena per questa coordinata y
        std::vector<std::pair<double, double>> arena_segments = get_intersection_points(y, obstacles, arena);

        // Per ogni segmento valido dell'arena, crea una linea orizzontale
        // escludendo le parti che intersecano gli ostacoli
        for (const auto &segment : arena_segments)
        {
            double x_start = segment.first;
            double x_end = segment.second;

            // Raccogli tutte le intersezioni con gli ostacoli in questo segmento
            std::vector<std::pair<double, double>> obstacle_intervals;

            for (const auto &obstacle : obstacles.obstacles)
            {
                if (obstacle.radius > 0.0)
                {
                    // Ostacolo circolare
                    double center_x = static_cast<double>(obstacle.polygon.points[0].x);
                    double center_y = static_cast<double>(obstacle.polygon.points[0].y);

                    auto intersections = CommonFunction::line_circle_intersection(y, center_x, center_y, obstacle.radius);
                    if (intersections.size() == 2)
                    {
                        double x1 = std::min(intersections[0], intersections[1]);
                        double x2 = std::max(intersections[0], intersections[1]);

                        // Clamp alle bounds del segmento dell'arena
                        x1 = std::max(x1, x_start);
                        x2 = std::min(x2, x_end);

                        if (x1 < x2)
                        {
                            obstacle_intervals.emplace_back(x1, x2);
                        }
                    }
                }
                else
                {
                    // Ostacolo poligonale - trova tutte le intersezioni e determina gli intervalli occupati
                    const auto &poly = obstacle.polygon;
                    std::vector<double> poly_intersections;

                    for (size_t i = 0; i < poly.points.size(); ++i)
                    {
                        size_t j = (i + 1) % poly.points.size();
                        double x1 = static_cast<double>(poly.points[i].x);
                        double y1 = static_cast<double>(poly.points[i].y);
                        double x2 = static_cast<double>(poly.points[j].x);
                        double y2 = static_cast<double>(poly.points[j].y);

                        auto intersections = CommonFunction::line_segment_intersection(y, x1, y1, x2, y2);
                        for (double x : intersections)
                        {
                            if (x >= x_start && x <= x_end)
                            {
                                poly_intersections.push_back(x);
                            }
                        }
                    }

                    // Ordina le intersezioni
                    std::sort(poly_intersections.begin(), poly_intersections.end());
                    poly_intersections.erase(
                        std::unique(poly_intersections.begin(), poly_intersections.end()),
                        poly_intersections.end());

                    // Per poligoni, le intersezioni a coppie definiscono gli intervalli occupati
                    for (size_t i = 0; i < poly_intersections.size(); i += 2)
                    {
                        if (i + 1 < poly_intersections.size())
                        {
                            double interval_start = poly_intersections[i];
                            double interval_end = poly_intersections[i + 1];

                            // Verifica che l'intervallo sia effettivamente dentro l'ostacolo
                            double mid_x = (interval_start + interval_end) / 2.0;
                            if (point_in_polygon(poly, mid_x, y))
                            {
                                obstacle_intervals.emplace_back(interval_start, interval_end);
                            }
                        }
                    }
                }
            }

            // Ordina gli intervalli degli ostacoli per coordinate x
            std::sort(obstacle_intervals.begin(), obstacle_intervals.end());

            // Unisci intervalli sovrapposti
            std::vector<std::pair<double, double>> merged_intervals;
            for (const auto &interval : obstacle_intervals)
            {
                if (merged_intervals.empty() || merged_intervals.back().second < interval.first)
                {
                    merged_intervals.push_back(interval);
                }
                else
                {
                    merged_intervals.back().second = std::max(merged_intervals.back().second, interval.second);
                }
            }

            // Genera segmenti liberi da ostacoli
            double current_x = x_start;
            for (const auto &blocked_interval : merged_intervals)
            {
                // Aggiungi segmento libero prima dell'ostacolo
                if (current_x < blocked_interval.first)
                {
                    double free_end = blocked_interval.first;
                    if (free_end - current_x > 1e-6) // Evita segmenti troppo piccoli
                    {
                        horizontal_lines.emplace_back(y, current_x, free_end);
                    }
                }
                // Salta l'ostacolo
                current_x = std::max(current_x, blocked_interval.second);
            }

            // Aggiungi l'ultimo segmento libero se c'è spazio
            if (current_x < x_end)
            {
                if (x_end - current_x > 1e-6) // Evita segmenti troppo piccoli
                {
                    horizontal_lines.emplace_back(y, current_x, x_end);
                }
            }
        }
    }

    return horizontal_lines;
}

std::vector<std::pair<double, double>> CombPathGenerator::set_point_in_vertical_line(
    const HorizontalLine &horizontal_line, double offset) const
{
    std::vector<std::pair<double, double>> points;

    double y = horizontal_line.y;
    double x_start = horizontal_line.x_start;
    double x_end = horizontal_line.x_end;

    // Assicurati che x_start sia minore di x_end
    if (x_start > x_end)
    {
        std::swap(x_start, x_end);
    }

    double line_length = x_end - x_start;

    // Se la linea è troppo corta per contenere punti con l'offset specificato
    if (line_length < offset)
    {
        // Aggiungi solo il punto centrale
        double mid_x = (x_start + x_end) / 2.0;
        points.emplace_back(mid_x, y);
        return points;
    }

    // Calcola il numero di punti che possono essere posizionati
    int num_points = static_cast<int>(line_length / offset) + 1;

    // Genera i punti lungo la linea
    for (int i = 0; i < num_points; ++i)
    {
        double x = x_start + i * offset;

        // Assicurati che il punto non superi x_end
        if (x <= x_end)
        {
            points.emplace_back(x, y);
        }
        else
        {
            break;
        }
    }

    // Se l'ultimo punto non coincide esattamente con x_end, aggiungilo
    if (!points.empty() && std::abs(points.back().first - x_end) > 1e-6)
    {
        points.emplace_back(x_end, y);
    }

    return points;
}

// get_cells_btw_vlines
std::vector<Cell> CombPathGenerator::get_cells_btw_vlines(
    const std::vector<HorizontalLine> &horizontal_lines) const
{
    std::vector<Cell> cells;

    if (horizontal_lines.empty())
    {
        return cells;
    }

    // Raggruppa le linee orizzontali per coordinata y
    std::map<double, std::vector<HorizontalLine>> lines_by_y;
    for (const auto &line : horizontal_lines)
    {
        lines_by_y[line.y].push_back(line);
    }

    // Ordina le coordinate y
    std::vector<double> y_coords;
    for (const auto &pair : lines_by_y)
    {
        y_coords.push_back(pair.first);
    }
    std::sort(y_coords.begin(), y_coords.end());

    // Genera celle tra linee verticali consecutive
    for (size_t i = 0; i < y_coords.size() - 1; ++i)
    {
        double y1 = y_coords[i];
        double y2 = y_coords[i + 1];

        const auto &lines1 = lines_by_y[y1];
        const auto &lines2 = lines_by_y[y2];

        // Per ogni segmento della linea inferiore (y1)
        for (const auto &line1 : lines1)
        {
            // Trova segmenti sovrapposti nella linea superiore (y2)
            for (const auto &line2 : lines2)
            {
                // Calcola l'intersezione orizzontale tra i due segmenti
                double x_start = std::max(line1.x_start, line2.x_start);
                double x_end = std::min(line1.x_end, line2.x_end);

                // Se c'è sovrapposizione, crea una cella
                if (x_start < x_end && (x_end - x_start) > 1e-6)
                {
                    Cell cell;
                    cell.x_min = x_start;
                    cell.x_max = x_end;
                    cell.y_min = y1;
                    cell.y_max = y2;
                    cell.center_x = (x_start + x_end) / 2.0;
                    cell.center_y = (y1 + y2) / 2.0;
                    cell.width = x_end - x_start;
                    cell.height = y2 - y1;

                    cells.push_back(cell);
                }
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("comb_path_generator"),
                "Generate %zu celle da %zu linee orizzontali",
                cells.size(), horizontal_lines.size());

    return cells;
}

// get centroid by cell
std::pair<double, double> CombPathGenerator::get_cell_centroid(const Cell &cell)
{
    double centroid_x = (cell.x_min + cell.x_max) / 2.0;
    double centroid_y = (cell.y_min + cell.y_max) / 2.0;
    return {centroid_x, centroid_y};
}

std::vector<std::pair<double, double>> CombPathGenerator::get_intersection_points(
    double y,
    const obstacles_msgs::msg::ObstacleArrayMsg & /*obstacles*/,
    const geometry_msgs::msg::Polygon &arena) const
{
    std::vector<double> intersections;

    // Trova intersezioni con i bordi dell'arena
    for (size_t i = 0; i < arena.points.size(); ++i)
    {
        size_t j = (i + 1) % arena.points.size();
        double x1 = static_cast<double>(arena.points[i].x);
        double y1 = static_cast<double>(arena.points[i].y);
        double x2 = static_cast<double>(arena.points[j].x);
        double y2 = static_cast<double>(arena.points[j].y);

        auto line_intersections = CommonFunction::line_segment_intersection(y, x1, y1, x2, y2);
        intersections.insert(intersections.end(), line_intersections.begin(), line_intersections.end());
    }

    // Ordina e rimuovi duplicati
    std::sort(intersections.begin(), intersections.end());
    intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());

    // Crea coppie di segmenti (assumendo che l'arena sia un poligono convesso)
    std::vector<std::pair<double, double>> segments;
    for (size_t i = 0; i < intersections.size(); i += 2)
    {
        if (i + 1 < intersections.size())
        {
            // Verifica che il punto medio del segmento sia dentro l'arena
            double mid_x = (intersections[i] + intersections[i + 1]) / 2.0;
            if (point_in_polygon(arena, mid_x, y))
            {
                segments.emplace_back(intersections[i], intersections[i + 1]);
            }
        }
    }

    return segments;
}

// Nuovo metodo per ottenere solo le linee orizzontali
std::vector<HorizontalLine> CombPathGenerator::get_horizontal_lines(
    const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
    const geometry_msgs::msg::Polygon &arena)
{
    // 1. Crea lista di punti (vertici di ostacoli e arena)
    std::vector<std::pair<double, double>> points = get_pointlist(obstacles, arena);

    // 2. Genera le linee orizzontali per ogni punto
    return set_vertical_line(points, obstacles, arena);
}

std::vector<std::vector<int>> CombPathGenerator::get_arc(
    std::vector<HorizontalLine> horizontal_lines,
    std::vector<std::pair<double, double>> points,
    std::vector<std::pair<double, double>> points_centroids)
{
    std::vector<std::vector<int>> adjacency_list(points.size() + points_centroids.size());

    // Crea un mapping per identificare rapidamente i punti
    std::map<std::pair<double, double>, int> point_to_index;

    // Indicizza i punti sulle linee orizzontali
    for (size_t i = 0; i < points.size(); ++i)
    {
        point_to_index[points[i]] = static_cast<int>(i);
    }

    // Indicizza i centroidi (offset dopo i punti delle linee)
    for (size_t i = 0; i < points_centroids.size(); ++i)
    {
        point_to_index[points_centroids[i]] = static_cast<int>(points.size() + i);
    }

    // Raggruppa le linee orizzontali per coordinata y
    std::map<double, std::vector<HorizontalLine>> lines_by_y;
    for (const auto &line : horizontal_lines)
    {
        lines_by_y[line.y].push_back(line);
    }

    // Ottieni le coordinate y ordinate
    std::vector<double> y_coords;
    for (const auto &pair : lines_by_y)
    {
        y_coords.push_back(pair.first);
    }
    std::sort(y_coords.begin(), y_coords.end());

    // Per ogni coppia di linee orizzontali consecutive, trova le connessioni
    for (size_t i = 0; i < y_coords.size() - 1; ++i)
    {
        double y1 = y_coords[i];     // linea inferiore
        double y2 = y_coords[i + 1]; // linea superiore

        const auto &lines1 = lines_by_y[y1];
        const auto &lines2 = lines_by_y[y2];

        // Per ogni segmento della linea inferiore
        for (const auto &line1 : lines1)
        {
            // Trova segmenti sovrapposti nella linea superiore
            for (const auto &line2 : lines2)
            {
                // Calcola l'intersezione orizzontale tra i due segmenti
                double x_start = std::max(line1.x_start, line2.x_start);
                double x_end = std::min(line1.x_end, line2.x_end);

                // Se c'è sovrapposizione, questa rappresenta una cella
                if (x_start < x_end && (x_end - x_start) > 1e-6)
                {
                    // Calcola il centroide della cella
                    double centroid_x = (x_start + x_end) / 2.0;
                    double centroid_y = (y1 + y2) / 2.0;
                    std::pair<double, double> centroid = {centroid_x, centroid_y};

                    // Trova l'indice del centroide
                    auto centroid_it = point_to_index.find(centroid);
                    int centroid_idx = -1;

                    if (centroid_it != point_to_index.end())
                    {
                        centroid_idx = centroid_it->second;
                    }
                    else
                    {
                        // Se non troviamo esattamente il centroide, cerchiamo il più vicino
                        double min_dist = std::numeric_limits<double>::infinity();

                        for (size_t c = 0; c < points_centroids.size(); ++c)
                        {
                            double dx = points_centroids[c].first - centroid_x;
                            double dy = points_centroids[c].second - centroid_y;
                            double dist = std::sqrt(dx * dx + dy * dy);

                            if (dist < min_dist && dist < 1e-3)
                            { // tolleranza piccola
                                min_dist = dist;
                                centroid_idx = static_cast<int>(points.size() + c);
                            }
                        }

                        if (centroid_idx == -1)
                            continue; // Nessun centroide trovato
                    }

                    // Trova tutti i punti sulla linea inferiore (y1) che sono nella cella
                    for (const auto &point : points)
                    {
                        if (std::abs(point.second - y1) < 1e-6 && // stesso y
                            point.first >= x_start && point.first <= x_end)
                        { // dentro la cella in x

                            auto point_it = point_to_index.find(point);
                            if (point_it != point_to_index.end())
                            {
                                int point_idx = point_it->second;

                                // Collega punto al centroide (bidirezionale)
                                adjacency_list[point_idx].push_back(centroid_idx);
                                adjacency_list[centroid_idx].push_back(point_idx);
                            }
                        }
                    }

                    // Trova tutti i punti sulla linea superiore (y2) che sono nella cella
                    for (const auto &point : points)
                    {
                        if (std::abs(point.second - y2) < 1e-6 && // stesso y
                            point.first >= x_start && point.first <= x_end)
                        { // dentro la cella in x

                            auto point_it = point_to_index.find(point);
                            if (point_it != point_to_index.end())
                            {
                                int point_idx = point_it->second;

                                // Collega punto al centroide (bidirezionale)
                                adjacency_list[point_idx].push_back(centroid_idx);
                                adjacency_list[centroid_idx].push_back(point_idx);
                            }
                        }
                    }
                }
            }
        }
    }

    // Rimuovi duplicati dalla lista di adiacenza
    for (auto &adj : adjacency_list)
    {
        std::sort(adj.begin(), adj.end());
        adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
    }

    RCLCPP_INFO(rclcpp::get_logger("comb_path_generator"),
                "Generato grafo con %zu nodi (%zu punti + %zu centroidi)",
                adjacency_list.size(), points.size(), points_centroids.size());

    return adjacency_list;
}

// Versione temporanea di Dijkstra che usa i punti e adiacenze temporanei
std::vector<int> CombPathGenerator::dijkstra_shortest_path_temp(
    int start, int goal,
    const std::vector<geometry_msgs::msg::Point> &points,
    const std::vector<std::vector<int>> &adjacency) const
{

    if (start == goal)
        return {start};

    const size_t N = points.size();
    if (start < 0 || goal < 0 || static_cast<size_t>(start) >= N || static_cast<size_t>(goal) >= N)
    {
        return {};
    }

    std::vector<double> dist(N, std::numeric_limits<double>::infinity());
    std::vector<int> prev(N, -1);
    std::vector<bool> visited(N, false);

    dist[start] = 0.0;

    for (size_t count = 0; count < N; ++count)
    {
        int u = -1;
        for (size_t v = 0; v < N; ++v)
        {
            if (!visited[v] && (u == -1 || dist[v] < dist[u]))
            {
                u = static_cast<int>(v);
            }
        }

        if (u == -1 || dist[u] == std::numeric_limits<double>::infinity())
            break;

        visited[u] = true;

        if (u == goal)
            break;

        for (int v : adjacency[u])
        {
            if (!visited[v])
            {
                double weight = std::sqrt(CommonFunction::dist2(points[u], points[v]));
                double alt = dist[u] + weight;
                if (alt < dist[v])
                {
                    dist[v] = alt;
                    prev[v] = u;
                }
            }
        }
    }

    // Ricostruisci il path
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at])
    {
        path.push_back(at);
    }

    if (path.empty() || path.back() != start)
    {
        return {}; // Nessun path trovato
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool CombPathGenerator::point_in_polygon(const geometry_msgs::msg::Polygon &poly, double x, double y) const
{
    return CommonFunction::point_in_polygon(poly, x, y);
}

void CombPathGenerator::set_default_frame_id(const std::string &fid)
{
    default_frame_id_ = fid;
}

void CombPathGenerator::set_default_step_size(double s)
{
    default_step_size_ = s;
}

const std::string &CombPathGenerator::default_frame_id()
{
    return default_frame_id_;
}

double CombPathGenerator::default_step_size()
{
    return default_step_size_;
}

} // namespace planning_pkg