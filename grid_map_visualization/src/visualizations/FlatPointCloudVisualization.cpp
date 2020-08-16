/*
 * FlatPointCloudVisualization.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_visualization/visualizations/FlatPointCloudVisualization.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace grid_map_visualization {

FlatPointCloudVisualization::FlatPointCloudVisualization(rclcpp::Node::SharedPtr node, const std::string& name)
    : VisualizationBase(node, name),
      height_(0.0)
{
}

FlatPointCloudVisualization::~FlatPointCloudVisualization()
{
}

bool FlatPointCloudVisualization::readParameters()
{
  height_ = 0.0;
  if (!getParam("height", height_)) {
    RCLCPP_INFO(node_->get_logger(), "FlatPointCloudVisualization with name '%s' did not find a 'height' parameter. Using default.", name_.c_str());
  }

  return true;
}

bool FlatPointCloudVisualization::initialize()
{
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_, custom_qos_);
  return true;
}

bool FlatPointCloudVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  sensor_msgs::msg::PointCloud2 pointCloud;

  grid_map::GridMap mapCopy(map);
  mapCopy.add("flat", height_);
  grid_map::GridMapRosConverter::toPointCloud(mapCopy, "flat", pointCloud);

  publisher_->publish(pointCloud);
  return true;
}

} /* namespace */
