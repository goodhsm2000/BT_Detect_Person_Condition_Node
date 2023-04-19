// Made by HSM

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECT_PERSON_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECT_PERSON_CONDITION_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp" // rclcpp/rclcpp.hpp를 사용하여 ROS 시스템을 사용할 수 있게 됩니다. 
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class DetectPersonCondition : public BT::ConditionNode // BT::ConditionNOde 클래스를 상속하는 DetectPersonCondition 클래스를 선언합니다. 
{
public:
  DetectPersonCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  DetectPersonCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  bool person_detected_ = false;
  std::mutex mutex_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CONDITION__DETECT_PERSON_CONDITION_HPP_
