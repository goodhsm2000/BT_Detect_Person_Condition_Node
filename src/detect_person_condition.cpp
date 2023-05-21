#include <string>

#include "nav2_behavior_tree/plugins/condition/detect_person_condition.hpp"

namespace nav2_behavior_tree
{

DetectPersonCondition::DetectPersonCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node"); // Behavior Tree의 구성(Config)인 블랙보드에서 "node"라는 키(Key)에 해당하는 데이터를 가져음
  subscription_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>( // "/yolo/detections" topic의 메세지가 도착할 때마다 detectionCallback 함수를 실행시키는 subscriber
    "/yolo/detections",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DetectPersonCondition::detectionCallback, this, std::placeholders::_1));
}

BT::NodeStatus DetectPersonCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (person_detected_) 
  {
    RCLCPP_INFO(node_->get_logger(), "Person detected");
    return BT::NodeStatus::FAILURE;
  } 
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Person didn't detected"); 
    return BT::NodeStatus::SUCCESS;
  }
}

void DetectPersonCondition::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->detections.empty())
    person_detected_ = false;
  for (const auto& detection : msg->detections) { // 아무런 객체도 검출되지 않으면 이 코드조차 실행이 안됨
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

