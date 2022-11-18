#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_TRANSFORMER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_TRANSFORMER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class GoalTransformer : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalTransformer
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalTransformer(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::InputPort <std::string> ("publishing_frame", "map", "Frame to convert goal into before publishing"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "output_goal",
        "Received Goal by subscription"),
        
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_TRANSFORMER_NODE_HPP_
