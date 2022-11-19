#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/goal_transformer_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/time.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;

GoalTransformer::GoalTransformer(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

inline BT::NodeStatus GoalTransformer::tick()
{
  geometry_msgs::msg::PoseStamped goal;

  getInput("input_goal", goal);

  geometry_msgs::msg::PoseStamped msg_transformed;

  std::string publishing_frame;
  getInput("publishing_frame", publishing_frame);

  RCLCPP_INFO(
    node_->get_logger(), "Transforming goal from %s to %s", goal.header.frame_id.c_str(), publishing_frame.c_str());  

  RCLCPP_INFO(node_->get_logger(), "Goal recieved timestamp: %ds, %dns", goal.header.stamp.sec, goal.header.stamp.nanosec);
  tf_->lookupTransform(publishing_frame, goal.header.frame_id, tf2::TimePointZero, 3000ms);
  RCLCPP_INFO(
    node_->get_logger(), "Looked up transform");  
  nav2_util::transformPoseInTargetFrame(goal, msg_transformed, *tf_, publishing_frame, 1.0);
  msg_transformed.header.stamp = node_->now();
  RCLCPP_INFO(
    node_->get_logger(), "Transformed pose"); 

  setOutput("output_goal", msg_transformed);
  return child_node_->executeTick();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalTransformer>("GoalTransformer");
}
