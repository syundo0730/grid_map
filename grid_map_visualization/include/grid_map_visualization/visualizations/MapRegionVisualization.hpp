/*
 * MapRegionVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace grid_map_visualization {

/*!
 * Visualization of the region of the grid map as border line.
 */
class MapRegionVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param node the ROS node handle.
   * @param name the name of the visualization.
   */
  MapRegionVisualization(rclcpp::Node::SharedPtr node, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~MapRegionVisualization();

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;

  //! Marker to be published.
  visualization_msgs::msg::Marker marker_;

  //! Number of vertices of the map region visualization.
  const unsigned int nVertices_;

  //! Color of the map region visualization.
  std_msgs::msg::ColorRGBA color_;

  //! Line width of the map region marker [m].
  double lineWidth_;

};

} /* namespace */
