#include "planning_pkg/visualizer_rviz.hpp"

namespace planning_pkg
{
    VisualizationUtils::VisualizationUtils(rclcpp::Node *node) : node_(node)
    {
        const auto qos_markers = rclcpp::QoS(rclcpp::KeepLast(10), planning_pkg::qos::qos_profile_markers);

        pub_points_markers = node_->create_publisher<visualization_msgs::msg::MarkerArray>("points_markers", qos_markers);
        pub_cells_markers = node_->create_publisher<visualization_msgs::msg::MarkerArray>("cells_markers", qos_markers);
        pub_vertical_line = node_->create_publisher<visualization_msgs::msg::MarkerArray>("vertical_lines", qos_markers);
        pub_arcs_markers = node_->create_publisher<visualization_msgs::msg::MarkerArray>("arcs_markers", qos_markers);
        pub_random_points_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_random_points", qos_markers);
        pub_knn_edges_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_knn_edges", qos_markers);
    }
        void VisualizationUtils::vis_points(const std::vector<std::pair<double, double>> &points)
        {
            if (points.empty())
            {
                RCLCPP_WARN(node_->get_logger(), "No points to visualize");
                return;
            }

            visualization_msgs::msg::MarkerArray marker_array;

            for (size_t i = 0; i < points.size(); ++i)
            {
                const auto &point = points[i];

                visualization_msgs::msg::Marker point_marker;
                point_marker.header.frame_id = "map";
                point_marker.header.stamp = node_->get_clock()->now();
                point_marker.ns = "pointlist_markers";
                point_marker.id = static_cast<int>(i);
                point_marker.type = visualization_msgs::msg::Marker::SPHERE;
                point_marker.action = visualization_msgs::msg::Marker::ADD;

                // Posizione del punto
                point_marker.pose.position.x = point.first;
                point_marker.pose.position.y = point.second;
                point_marker.pose.position.z = 0.1;
                point_marker.pose.orientation.w = 1.0;

                // Scala del marker
                point_marker.scale.x = 0.15;
                point_marker.scale.y = 0.15;
                point_marker.scale.z = 0.15;

                // Colore del marker (rosso)
                point_marker.color.r = 1.0;
                point_marker.color.g = 0.0;
                point_marker.color.b = 0.0;
                point_marker.color.a = 0.8;

                point_marker.lifetime = rclcpp::Duration::from_seconds(0);
                marker_array.markers.push_back(point_marker);
            }

            pub_points_markers->publish(marker_array);
            RCLCPP_INFO(node_->get_logger(), "Published %zu point markers for visualization",
                        marker_array.markers.size());
        }

        void VisualizationUtils::vis_cells(const std::vector<planning_pkg::Cell> &cells)
        {
            if (cells.empty())
            {
                RCLCPP_WARN(node_->get_logger(), "No cells to visualize");
                return;
            }

            visualization_msgs::msg::MarkerArray marker_array;

            for (size_t i = 0; i < cells.size(); ++i)
            {
                const auto &cell = cells[i];

                visualization_msgs::msg::Marker cell_marker;
                cell_marker.header.frame_id = "map";
                cell_marker.header.stamp = node_->get_clock()->now();
                cell_marker.ns = "cells_markers";
                cell_marker.id = static_cast<int>(i);
                cell_marker.type = visualization_msgs::msg::Marker::CUBE;
                cell_marker.action = visualization_msgs::msg::Marker::ADD;

                // Posizione del centro della cella
                cell_marker.pose.position.x = cell.center_x;
                cell_marker.pose.position.y = cell.center_y;
                cell_marker.pose.position.z = 0.05;
                cell_marker.pose.orientation.w = 1.0;

                // Scala del marker
                cell_marker.scale.x = cell.width;
                cell_marker.scale.y = cell.height;
                cell_marker.scale.z = 0.1;

                // Colore del marker (blu trasparente)
                cell_marker.color.r = 0.0;
                cell_marker.color.g = 0.0;
                cell_marker.color.b = 1.0;
                cell_marker.color.a = 0.3;

                cell_marker.lifetime = rclcpp::Duration::from_seconds(0);
                marker_array.markers.push_back(cell_marker);
            }

            pub_cells_markers->publish(marker_array);
            RCLCPP_INFO(node_->get_logger(), "Published %zu cell markers for visualization",
                        marker_array.markers.size());
        }

        void VisualizationUtils::vis_line()
        {
            if (vertical_lines_data_.empty())
            {
                RCLCPP_WARN(node_->get_logger(), "No lines to visualize");
                return;
            }

            visualization_msgs::msg::MarkerArray marker_array;

            for (size_t i = 0; i < vertical_lines_data_.size(); ++i)
            {
                const auto &line_data = vertical_lines_data_[i];

                visualization_msgs::msg::Marker line_marker;
                line_marker.header.frame_id = "map";
                line_marker.header.stamp = node_->get_clock()->now();
                line_marker.ns = "horizontal_lines";
                line_marker.id = static_cast<int>(i);
                line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::msg::Marker::ADD;

                line_marker.pose.orientation.w = 1.0;
                line_marker.scale.x = 0.05;
                line_marker.color.r = 0.0;
                line_marker.color.g = 1.0;
                line_marker.color.b = 0.0;
                line_marker.color.a = 0.8;

                geometry_msgs::msg::Point start_point;
                start_point.x = std::get<2>(line_data).first;
                start_point.y = std::get<2>(line_data).second;
                start_point.z = std::get<1>(line_data);

                geometry_msgs::msg::Point end_point;
                end_point.x = std::get<3>(line_data).first;
                end_point.y = std::get<3>(line_data).second;
                end_point.z = std::get<1>(line_data);

                line_marker.points.push_back(start_point);
                line_marker.points.push_back(end_point);

                marker_array.markers.push_back(line_marker);
            }

            pub_vertical_line->publish(marker_array);
            RCLCPP_INFO(node_->get_logger(), "Published %zu line markers for visualization",
                        marker_array.markers.size());
        }

        void VisualizationUtils::vis_arcs(const std::vector<std::vector<int>> &arc_list,
                                          const std::vector<std::pair<double, double>> &points_line,
                                          const std::vector<std::pair<double, double>> &points_centroids)
        {
            if (arc_list.empty())
            {
                RCLCPP_WARN(node_->get_logger(), "No arcs to visualize");
                return;
            }

            visualization_msgs::msg::MarkerArray marker_array;

            std::vector<std::pair<double, double>> all_points;
            all_points.insert(all_points.end(), points_line.begin(), points_line.end());
            all_points.insert(all_points.end(), points_centroids.begin(), points_centroids.end());

            int marker_id = 0;

            for (size_t i = 0; i < arc_list.size(); ++i)
            {
                if (i >= all_points.size())
                    continue;

                const auto &connections = arc_list[i];
                const auto &start_point = all_points[i];

                for (int connected_node : connections)
                {
                    if (connected_node >= static_cast<int>(all_points.size()) || connected_node < 0)
                        continue;

                    const auto &end_point = all_points[connected_node];

                    visualization_msgs::msg::Marker arc_marker;
                    arc_marker.header.frame_id = "map";
                    arc_marker.header.stamp = node_->get_clock()->now();
                    arc_marker.ns = "graph_arcs";
                    arc_marker.id = marker_id++;
                    arc_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    arc_marker.action = visualization_msgs::msg::Marker::ADD;

                    arc_marker.pose.orientation.w = 1.0;
                    arc_marker.scale.x = 0.02;

                    if (i < points_line.size() && connected_node >= static_cast<int>(points_line.size()))
                    {
                        arc_marker.color.r = 1.0;
                        arc_marker.color.g = 0.0;
                        arc_marker.color.b = 0.0;
                        arc_marker.color.a = 0.6;
                    }
                    else if (i >= points_line.size() && connected_node < static_cast<int>(points_line.size()))
                    {
                        arc_marker.color.r = 1.0;
                        arc_marker.color.g = 0.5;
                        arc_marker.color.b = 0.0;
                        arc_marker.color.a = 0.6;
                    }
                    else
                    {
                        arc_marker.color.r = 0.5;
                        arc_marker.color.g = 0.0;
                        arc_marker.color.b = 1.0;
                        arc_marker.color.a = 0.6;
                    }

                    geometry_msgs::msg::Point start_geom_point;
                    start_geom_point.x = start_point.first;
                    start_geom_point.y = start_point.second;
                    start_geom_point.z = 0.05;

                    geometry_msgs::msg::Point end_geom_point;
                    end_geom_point.x = end_point.first;
                    end_geom_point.y = end_point.second;
                    end_geom_point.z = 0.05;

                    arc_marker.points.push_back(start_geom_point);
                    arc_marker.points.push_back(end_geom_point);

                    arc_marker.lifetime = rclcpp::Duration::from_seconds(0);
                    marker_array.markers.push_back(arc_marker);
                }
            }

            pub_arcs_markers->publish(marker_array);
            RCLCPP_INFO(node_->get_logger(), "Published %d arc markers for visualization", marker_id);
        }

        void VisualizationUtils::vis_random_points_markers(const std::vector<geometry_msgs::msg::Point> &random_points)
        {
            visualization_msgs::msg::MarkerArray marker_array;

            for (size_t i = 0; i < random_points.size(); ++i)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = node_->get_clock()->now();
                marker.ns = "prm_random_points";
                marker.id = static_cast<int>(i);
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position = random_points[i];
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;

                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;

                marker_array.markers.push_back(marker);
            }

            try
            {
                pub_random_points_->publish(marker_array);
                RCLCPP_INFO(node_->get_logger(), "Published %zu random points markers", marker_array.markers.size());
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(node_->get_logger(), "Failed to publish random points markers: %s", e.what());
            }
        }

        void VisualizationUtils::vis_knn_edges_markers(const std::vector<geometry_msgs::msg::Point> &random_points,
                                                       const std::vector<std::vector<int>> &knn_adj)
        {
            visualization_msgs::msg::MarkerArray marker_array;

            for (size_t i = 0; i < knn_adj.size(); ++i)
            {
                for (size_t j = 0; j < knn_adj[i].size(); ++j)
                {
                    int neighbor_idx = knn_adj[i][j];

                    if (static_cast<int>(i) >= neighbor_idx)
                        continue;

                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = node_->get_clock()->now();
                    marker.ns = "prm_knn_edges";
                    marker.id = static_cast<int>(i * 1000 + j); // Unique ID
                    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    marker.action = visualization_msgs::msg::Marker::ADD;

                    geometry_msgs::msg::Point start_point = random_points[i];
                    geometry_msgs::msg::Point end_point = random_points[neighbor_idx];

                    marker.points.push_back(start_point);
                    marker.points.push_back(end_point);

                    marker.scale.x = 0.02;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.6;
                    marker_array.markers.push_back(marker);
                }
            }

            try
            {
                pub_knn_edges_->publish(marker_array);
                RCLCPP_INFO(node_->get_logger(), "Published %zu k-NN edge markers", marker_array.markers.size());
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(node_->get_logger(), "Failed to publish k-NN edges markers: %s", e.what());
            }
        }
    
} // namespace planning_pkg