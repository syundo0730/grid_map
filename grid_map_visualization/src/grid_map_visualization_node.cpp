/*
 * grid_map_visualization_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_visualization/GridMapVisualization.hpp>

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_visualization");
  auto viz = std::make_shared<grid_map_visualization::GridMapVisualization>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
