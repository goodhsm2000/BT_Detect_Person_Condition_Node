// made by HSM

#include <string>

#include "nav2_behavior_tree/plugins/condition/detect_person_condition.hpp"

namespace nav2_behavior_tree
{

DetectPersonCondition::DetectPersonCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  subscription_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/yolo/detections",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DetectPersonCondition::detectionCallback, this, std::placeholders::_1));
}

BT::NodeStatus DetectPersonCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (person_detected_) 
  {
    //person_detected_ = false;
    RCLCPP_INFO(node_->get_logger(), "Person detected");
    return BT::NodeStatus::FAILURE;
  } 
  else
    RCLCPP_INFO(node_->get_logger(), "Person didn't detected"); 
    return BT::NodeStatus::SUCCESS;
}

void DetectPersonCondition::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& detection : msg->detections) {
    for (const auto& hypothesis : detection.results) {
      if (hypothesis.id == "person") 
        person_detected_ = true;
      else
        person_detected_ = false;
      return;
    }
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DetectPersonCondition>("DetectPerson");
}
