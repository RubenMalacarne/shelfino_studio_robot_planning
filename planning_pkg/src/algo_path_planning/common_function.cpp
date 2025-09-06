#include "planning_pkg/common_function.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace planning_pkg
{
    void CommonFunction::compute_bbox(const geometry_msgs::msg::Polygon &poly,
                                      double &minx, double &miny,
                                      double &maxx, double &maxy)
    {
        if (poly.points.empty())
        {
            minx = miny = maxx = maxy = 0.0;
            return;
        }
        minx = maxx = poly.points.front().x;
        miny = maxy = poly.points.front().y;
        for (const auto &p : poly.points)
        {
            minx = std::min(minx, static_cast<double>(p.x));
            miny = std::min(miny, static_cast<double>(p.y));
            maxx = std::max(maxx, static_cast<double>(p.x));
            maxy = std::max(maxy, static_cast<double>(p.y));
        }
    }

    bool CommonFunction::point_in_polygon(const geometry_msgs::msg::Polygon &poly, double x, double y)
    {
        bool inside = false;
        const auto n = poly.points.size();
        if (n < 3)
            return false;

        for (std::size_t i = 0, j = n - 1; i < n; j = i++)
        {
            const auto &pi = poly.points[i];
            const auto &pj = poly.points[j];
            const bool intersect =
                ((static_cast<double>(pi.y) > y) != (static_cast<double>(pj.y) > y)) &&
                (x < (static_cast<double>(pj.x) - static_cast<double>(pi.x)) *
                             (y - static_cast<double>(pi.y)) /
                             ((static_cast<double>(pj.y) - static_cast<double>(pi.y)) + 1e-12) +
                         static_cast<double>(pi.x));
            if (intersect)
                inside = !inside;
        }
        return inside;
    }

    inline double sqr(double v) { return v * v; }

    double CommonFunction::point_to_segment_distance(double px, double py,
                                                     double ax, double ay,
                                                     double bx, double by)
    {
        const double vx = bx - ax;
        const double vy = by - ay;
        const double wx = px - ax;
        const double wy = py - ay;

        const double c1 = vx * wx + vy * wy;
        if (c1 <= 0.0)
        {
            return std::sqrt(sqr(px - ax) + sqr(py - ay));
        }
        const double c2 = vx * vx + vy * vy;
        if (c2 <= 1e-15)
        {
            // segmento quasi nullo
            return std::sqrt(sqr(px - ax) + sqr(py - ay));
        }
        const double t = std::clamp(c1 / c2, 0.0, 1.0);
        const double projx = ax + t * vx;
        const double projy = ay + t * vy;
        return std::sqrt(sqr(px - projx) + sqr(py - projy));
    }

    double CommonFunction::distance_to_polygon_edges(const geometry_msgs::msg::Polygon &poly,
                                                     double x, double y)
    {
        const auto n = poly.points.size();
        if (n < 2)
            return std::numeric_limits<double>::infinity();

        double dmin = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++)
        {
            const auto &pi = poly.points[i];
            const auto &pj = poly.points[j];
            const double d = point_to_segment_distance(
                x, y,
                static_cast<double>(pj.x), static_cast<double>(pj.y),
                static_cast<double>(pi.x), static_cast<double>(pi.y));
            if (d < dmin)
                dmin = d;
        }
        return dmin;
    }

    // Controllo validità con clearance rispetto ad arena e ostacoli
    bool CommonFunction::point_is_valid(double x, double y,
                                        const geometry_msgs::msg::Polygon &arena,
                                        const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                                        double clearance)
    {
        // dentro l'arena
        if (!point_in_polygon(arena, x, y))
            return false;

        // distanza dal bordo arena
        if (clearance > 0.0)
        {
            const double d_arena = distance_to_polygon_edges(arena, x, y);
            if (d_arena < clearance)
                return false;
        }

        // fuori da ogni ostacolo + clearance dai bordi degli ostacoli
        for (const auto &obs : obstacles.obstacles)
        {
            // Gestione ostacoli circolari
            if (obs.radius > 0.0)
            {
                const double dx = x - obs.polygon.points[0].x; // centro del cerchio
                const double dy = y - obs.polygon.points[0].y;
                const double distance_to_center = std::sqrt(dx * dx + dy * dy);

                // Punto dentro il cerchio
                if (distance_to_center <= obs.radius)
                    return false;

                // Clearance dal bordo del cerchio
                if (clearance > 0.0)
                {
                    const double distance_to_edge = distance_to_center - obs.radius;
                    if (distance_to_edge < clearance)
                        return false;
                }
            }
            // Gestione ostacoli poligonali
            else
            {
                const geometry_msgs::msg::Polygon &opoly = obs.polygon;
                if (point_in_polygon(opoly, x, y))
                    return false;

                if (clearance > 0.0)
                {
                    const double d_obs = distance_to_polygon_edges(opoly, x, y);
                    if (d_obs < clearance)
                        return false;
                }
            }
        }
        return true;
    }

    double CommonFunction::dist2(const geometry_msgs::msg::Point &a,
                                 const geometry_msgs::msg::Point &b)
    {
        const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
        const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
        return dx * dx + dy * dy;
    }

    // Calcola l'intersezione tra una linea orizzontale e un segmento
    std::vector<double> CommonFunction::line_segment_intersection(double y, double x1, double y1, double x2, double y2)
    {
        std::vector<double> intersections;

        // Controlla se la linea orizzontale interseca il segmento
        if ((y1 <= y && y <= y2) || (y2 <= y && y <= y1))
        {
            if (std::abs(y2 - y1) < 1e-12) // Segmento orizzontale
            {
                if (std::abs(y - y1) < 1e-12) // Stesso y
                {
                    intersections.push_back(std::min(x1, x2));
                    intersections.push_back(std::max(x1, x2));
                }
            }
            else
            {
                // Calcola l'intersezione x = x1 + (x2-x1) * (y-y1)/(y2-y1)
                double x_intersection = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
                intersections.push_back(x_intersection);
            }
        }
        return intersections;
    }

    // Calcola l'intersezione tra una linea orizzontale e un cerchio
    std::vector<double> CommonFunction::line_circle_intersection(double y, double center_x, double center_y, double radius)
    {
        std::vector<double> intersections;

        double dy = y - center_y;
        if (std::abs(dy) <= radius)
        {
            double dx = std::sqrt(radius * radius - dy * dy);
            intersections.push_back(center_x - dx);
            intersections.push_back(center_x + dx);
        }
        return intersections;
    }

    bool CommonFunction::segment_is_valid(const geometry_msgs::msg::Point &a,
                                          const geometry_msgs::msg::Point &b,
                                          const geometry_msgs::msg::Polygon &arena,
                                          const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
                                          double clearance,
                                          double sample_step)
    {
        // Clamp passo
        if (sample_step < 0.01)
            sample_step = 0.01;

        const double len = std::sqrt(dist2(a, b));
        // almeno 2 campioni (estremi compresi)
        const int samples = std::max(2, static_cast<int>(std::ceil(len / sample_step)) + 1);

        for (int i = 0; i < samples; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
            const double x = a.x + t * (b.x - a.x);
            const double y = a.y + t * (b.y - a.y);

            if (!point_is_valid(x, y, arena, obstacles, clearance))
            {
                return false;
            }
        }
        return true;
    }

    std::vector<geometry_msgs::msg::Point> CommonFunction::optimize_path_with_raycasting(
        const std::vector<geometry_msgs::msg::Point> &original_path,
        const obstacles_msgs::msg::ObstacleArrayMsg &obstacles,
        const geometry_msgs::msg::Polygon &arena,
        double clearance,
        double sample_step)
    {
        if (original_path.size() <= 2)
        {
            return original_path; // Non c'è niente da ottimizzare
        }

        std::vector<geometry_msgs::msg::Point> optimized_path;
        optimized_path.push_back(original_path[0]); // Aggiungi sempre il punto di start

        size_t current_index = 0;

        while (current_index < original_path.size() - 1)
        {
            size_t furthest_reachable = current_index;

            // Cerca il punto più lontano raggiungibile direttamente dal punto corrente
            for (size_t test_index = current_index + 1; test_index < original_path.size(); ++test_index)
            {
                if (segment_is_valid(original_path[current_index],
                                     original_path[test_index],
                                     arena, obstacles, clearance, sample_step))
                {
                    furthest_reachable = test_index;
                }
                else
                {
                    break; // Se questo punto non è raggiungibile, i successivi probabilmente non lo saranno
                }
            }

            // Se non possiamo andare oltre il prossimo punto, aggiungi il prossimo punto
            if (furthest_reachable == current_index)
            {
                // Caso di emergenza: non possiamo nemmeno raggiungere il prossimo punto
                // Questo non dovrebbe succedere se il path originale è valido
                // RCLCPP_WARN(rclcpp::get_logger("prm_path_generator"),
                //             "Impossibile raggiungere il punto successivo durante l'ottimizzazione");
                furthest_reachable = current_index + 1;
            }

            // Se abbiamo raggiunto il goal, aggiungilo e termina
            if (furthest_reachable == original_path.size() - 1)
            {
                if (optimized_path.back().x != original_path.back().x ||
                    optimized_path.back().y != original_path.back().y)
                {
                    optimized_path.push_back(original_path.back());
                }
                break;
            }

            // Altrimenti, aggiungi il punto più lontano raggiungibile (se diverso dal corrente)
            if (furthest_reachable > current_index)
            {
                optimized_path.push_back(original_path[furthest_reachable]);
                current_index = furthest_reachable;
            }
            else
            {
                // Fallback: avanza di un punto
                current_index++;
                if (current_index < original_path.size())
                {
                    optimized_path.push_back(original_path[current_index]);
                }
            }
        }

        // Assicurati che il goal sia sempre presente
        if (optimized_path.empty() ||
            (optimized_path.back().x != original_path.back().x ||
             optimized_path.back().y != original_path.back().y))
        {
            optimized_path.push_back(original_path.back());
        }

        // RCLCPP_INFO(rclcpp::get_logger("prm_path_generator"),
        //             "Path ottimizzato: da %zu punti a %zu punti",
        //             original_path.size(), optimized_path.size());

        return optimized_path;
    }
}