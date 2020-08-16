/*
 * VisualizationBase.hpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_core/GridMap.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"

namespace grid_map_visualization {

class VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param node the ROS node handle.
   * @param name the name of the visualization.
   */
  VisualizationBase(rclcpp::Node::SharedPtr node, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~VisualizationBase();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  virtual bool readParameters() = 0;

  /*!
   * Initialization.
   */
  virtual bool initialize() = 0;

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  virtual bool visualize(const grid_map::GridMap& map) = 0;

  /*!
   * Checks if visualization is active (if somebody has actually subscribed).
   * @return true if active, false otherwise.
   */
  bool isActive() const;

 protected:

  /*!
   * Get a visualization parameter as a string.
   * @param[in] name the name of the parameter
   * @param[out] value the string to set with the value.
   * @return true if parameter was found, false otherwise.
   */
  template<typename T>
  bool getParam(const std::string& name, T &value)
  {
    auto param_name = name_ + ".params." + name;
    node_->declare_parameter(param_name);
    return node_->get_parameter(param_name, value);
  }

  //! QoS for visualization
  rclcpp::QoS custom_qos_;

  //! ROS nodehandle.
  rclcpp::Node::SharedPtr node_;

  //! Name of the visualization.
  std::string name_;
};

} /* namespace */
