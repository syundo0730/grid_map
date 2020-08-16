/*
 * GridMapVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/GridMapVisualization.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;

using std::placeholders::_1;

namespace grid_map_visualization {

GridMapVisualization::GridMapVisualization(rclcpp::Node::SharedPtr node)
    : node_(node),
      factory_(node),
      isSubscribed_(false)
{
  RCLCPP_INFO(node_->get_logger(), "Grid map visualization node started.");
  readParameters();
  activityCheckTimer_ = node_->create_wall_timer(activityCheckDuration_, std::bind(&GridMapVisualization::updateSubscriptionCallback, this));
  initialize();
}

GridMapVisualization::~GridMapVisualization()
{
}

bool GridMapVisualization::readParameters()
{
  node_->declare_parameter("grid_map_topic", std::string("/grid_map"));
  node_->get_parameter("grid_map_topic", mapTopic_);

  node_->declare_parameter("activity_check_rate", 2.0);
  double activityCheckRate = node_->get_parameter("activity_check_rate").as_double();
  activityCheckDuration_ = std::chrono::milliseconds(int(1000 / activityCheckRate));

  // // Configure the visualizations from a configuration stored on the parameter server.
  // std::string config;
  // declare_parameter(visualizationsParameter_);
  // if (!get_parameter(visualizationsParameter_, config)) {
  //   RCLCPP_WARN(node_->get_logger(),
  //       "Could not load the visualizations configuration from parameter %s,are you sure it"
  //       "was pushed to the parameter server? Assuming that you meant to leave it empty.",
  //       visualizationsParameter_);
  //   return false;
  // }

  // // Verify proper naming and structure,
  // if (config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
  //   RCLCPP_ERROR(node_->get_logger(), "%s: The visualization specification must be a list, but it is of XmlRpcType %d",
  //             visualizationsParameter_, config.getType());
  //   RCLCPP_ERROR(node_->get_logger(), "The XML passed in is formatted as follows:\n %s", config.toXml().c_str());
  //   return false;
  // }

  // // Iterate over all visualizations (may be just one),
  // for (unsigned int i = 0; i < config.size(); ++i) {
  //   if (config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
  //     RCLCPP_ERROR(node_->get_logger(), "%s: Visualizations must be specified as maps, but they are XmlRpcType:%d",
  //               visualizationsParameter_.c_str(), config[i].getType());
  //     return false;
  //   } else if (!config[i].hasMember("type")) {
  //     RCLCPP_ERROR(node_->get_logger(), "%s: Could not add a visualization because no type was given",
  //               visualizationsParameter_.c_str());
  //     return false;
  //   } else if (!config[i].hasMember("name")) {
  //     RCLCPP_ERROR(node_->get_logger(), "%s: Could not add a visualization because no name was given",
  //               visualizationsParameter_.c_str());
  //     return false;
  //   } else {
  //     //Check for name collisions within the list itself.
  //     for (int j = i + 1; j < config.size(); ++j) {
  //       if (config[j].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
  //         RCLCPP_ERROR(node_->get_logger(), "%s: Visualizations must be specified as maps, but they are XmlRpcType:%d",
  //                   visualizationsParameter_.c_str(), config[j].getType());
  //         return false;
  //       }

  //       if (!config[j].hasMember("name")
  //           || config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString
  //           || config[j]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
  //         RCLCPP_ERROR(node_->get_logger(), "%s: Visualizations names must be strings, but they are XmlRpcTypes:%d and %d",
  //                   visualizationsParameter_.c_str(), config[i].getType(), config[j].getType());
  //         return false;
  //       }

  //       std::string namei = config[i]["name"];
  //       std::string namej = config[j]["name"];
  //       if (namei == namej) {
  //         RCLCPP_ERROR(node_->get_logger(), "%s: A visualization with the name '%s' already exists.",
  //                   visualizationsParameter_.c_str(), namei.c_str());
  //         return false;
  //       }
  //     }
  //   }

  //   // Make sure the visualization has a valid type.
  //   if (!factory_.isValidType(config[i]["type"])) {
  //     RCLCPP_ERROR(node_->get_logger(), "Could not find visualization of type '%s'.", std::string(config[i]["type"]).c_str());
  //     return false;
  //   }
  // }

  // for (int i = 0; i < config.size(); ++i) {
  //   std::string type = config[i]["type"];
  //   std::string name = config[i]["name"];
  //   auto visualization = factory_.getInstance(type, name);
  //   visualization->readParameters(config[i]);
  //   visualizations_.push_back(visualization);
  //   RCLCPP_INFO(node_->get_logger(), "%s: Configured visualization of type '%s' with name '%s'.",
  //            visualizationsParameter_.c_str(), type.c_str(), name.c_str());
  // }

  auto config = {
    std::make_pair("point_cloud", "elevation_points"),
    std::make_pair("vectors", "surface_normals"),
  };
  for (auto v: config) {
    auto visualization = factory_.getInstance(v.first, v.second);
    visualization->readParameters();
    visualizations_.push_back(visualization);
  }
  return true;
}

bool GridMapVisualization::initialize()
{
  for (auto& visualization : visualizations_) {
    visualization->initialize();
  }
  updateSubscriptionCallback();
  RCLCPP_INFO(node_->get_logger(), "Grid map visualization initialized.");
  return true;
}

void GridMapVisualization::updateSubscriptionCallback()
{
  bool isActive = false;

  for (auto& visualization : visualizations_) {
    if (visualization->isActive()) {
      isActive = true;
      break;
    }
  }

  if (!isSubscribed_ && isActive) {
    mapSubscriber_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(mapTopic_, 10, std::bind(&GridMapVisualization::callback, this, _1));
    isSubscribed_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "Subscribed to grid map at '%s'.", mapTopic_.c_str());
  }
  if (isSubscribed_ && !isActive) {
    mapSubscriber_.reset();
    isSubscribed_ = false;
    RCLCPP_DEBUG(node_->get_logger(), "Cancelled subscription to grid map.");
  }
}

void GridMapVisualization::callback(const std::shared_ptr<grid_map_msgs::msg::GridMap> message)
{
  RCLCPP_DEBUG(node_->get_logger(), "Grid map visualization received a map (timestamp %f) for visualization.");
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*message, map);

  for (auto& visualization : visualizations_) {
    visualization->visualize(map);
  }
}

} /* namespace */
