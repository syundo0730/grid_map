/*
 * VisualizationFactory.hpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <vector>
#include <string>
#include <memory>

namespace grid_map_visualization {

class VisualizationFactory
{
 public:
  VisualizationFactory(rclcpp::Node::SharedPtr node);
  virtual ~VisualizationFactory();

  bool isValidType(const std::string& type);
  std::shared_ptr<VisualizationBase> getInstance(const std::string& type, const std::string& name);

 private:
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> types_;
};

} /* namespace */
