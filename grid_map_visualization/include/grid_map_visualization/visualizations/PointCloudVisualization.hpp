/*
 * PointCloudVisualization.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace grid_map_visualization {

/*!
 * Visualization of the grid map as a point cloud.
 */
class PointCloudVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param node the ROS node handle.
   * @param name the name of the visualization.
   */
  PointCloudVisualization(rclcpp::Node::SharedPtr node, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~PointCloudVisualization();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap& map);

 private:
  //! ROS publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  //! Type that is transformed to points.
  std::string layer_;
};

} /* namespace */
