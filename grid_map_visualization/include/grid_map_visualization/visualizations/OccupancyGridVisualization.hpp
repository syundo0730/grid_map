/*
 * PointCloudOccupancyGrid.hpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace grid_map_visualization {

class OccupancyGridVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param node the ROS node handle.
   * @param name the name of the visualization.
   */
  OccupancyGridVisualization(rclcpp::Node::SharedPtr node, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~OccupancyGridVisualization();

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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

  //! Type that is transformed to the occupancy grid.
  std::string layer_;

  //! Minimum and maximum value of the grid map data (used to normalize the cell data in [min, max]).
  float dataMin_, dataMax_;
};

} /* namespace */
