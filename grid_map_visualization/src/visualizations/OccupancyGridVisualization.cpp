/*
 * OccupancyGridVisualization.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/OccupancyGridVisualization.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace grid_map_visualization {

OccupancyGridVisualization::OccupancyGridVisualization(rclcpp::Node::SharedPtr node, const std::string& name)
: VisualizationBase(node, name),
  dataMin_(0.0),
  dataMax_(1.0)
{
}

OccupancyGridVisualization::~OccupancyGridVisualization()
{
}

bool OccupancyGridVisualization::readParameters()
{
  if (!getParam("layer", layer_)) {
    RCLCPP_ERROR(node_->get_logger(), "OccupancyGridVisualization with name '%s' did not find a 'layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("data_min", dataMin_)) {
    RCLCPP_ERROR(node_->get_logger(), "OccupancyGridVisualization with name '%s' did not find a 'data_min' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("data_max", dataMax_)) {
    RCLCPP_ERROR(node_->get_logger(), "OccupancyGridVisualization with name '%s' did not find a 'data_max' parameter.", name_.c_str());
    return false;
  }

  return true;
}

bool OccupancyGridVisualization::initialize()
{
  publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(name_, custom_qos_);
  return true;
}

bool OccupancyGridVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  if (!map.exists(layer_)) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "OccupancyGridVisualization::visualize: No grid map layer with name '" << layer_ << "' found.");
    return false;
  }
  nav_msgs::msg::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, layer_, dataMin_, dataMax_, occupancyGrid);
  publisher_->publish(occupancyGrid);
  return true;
}

} /* namespace */
