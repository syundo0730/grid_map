/*
 * VisualizationBase.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization {

VisualizationBase::VisualizationBase(rclcpp::Node::SharedPtr node, const std::string& name)
    : custom_qos_(10),
      node_(node),
      name_(name)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  // if (publisher_.getNumSubscribers() > 0) return true;
  // return false;
  return true;
}

} /* namespace */
