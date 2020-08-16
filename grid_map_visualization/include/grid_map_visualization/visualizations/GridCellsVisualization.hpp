/*
 * GridCellsVisualization.hpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/grid_cells.hpp>

namespace grid_map_visualization {

class GridCellsVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param node the ROS node handle.
   * @param name the name of the visualization.
   */
  GridCellsVisualization(rclcpp::Node::SharedPtr node, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~GridCellsVisualization();

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
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher_;

  //! Type that is transformed to the occupancy grid.
  std::string layer_;

  //! Values that are between lower and upper threshold are shown.
  float lowerThreshold_, upperThreshold_;
};

} /* namespace */
