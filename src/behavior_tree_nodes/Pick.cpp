// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>

#include "gb_manipulation/behavior_tree_nodes/Pick.hpp"

#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/content.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_manipulation
{

Pick::Pick(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node_);

  pick_pub_ = node_->create_publisher<moveit_msgs::msg::Grasp>("/moveit/pick", 1);
  result_sub_ = node_->create_subscription<moveit_msgs::msg::MoveItErrorCodes>(
    "/moveit/result",
    1,
    std::bind(&Pick::resultCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  pick_action_sent_ = false;
}

void
Pick::halt()
{
  std::cout << "Pick halt" << std::endl;
}

void 
Pick::resultCallback(const moveit_msgs::msg::MoveItErrorCodes::SharedPtr msg)
{
  result_ = msg->val;
}

geometry_msgs::msg::PoseStamped 
Pick::getObjectTF(std::string id)
{
  // read graph to take the tf
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> tf_edges;
  tf_edges = graph_->get_edges("world", id,  ros2_knowledge_graph_msgs::msg::Content::STATICTF);
  geometry_msgs::msg::PoseStamped object_pose;
  
  for (auto edge : tf_edges)
  {    
    try {
      // Check if the transform is available
      auto tf = tf_buffer_->lookupTransform("base_footprint", id, tf2::TimePointZero);
      object_pose.pose.position.x = tf.transform.translation.x;
      object_pose.pose.position.y = tf.transform.translation.y;
      object_pose.pose.position.z = tf.transform.translation.z;
      object_pose.pose.orientation = tf.transform.rotation;
      object_pose.header.frame_id = "base_footprint";
      object_pose.header.stamp = node_->now();
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "%s", e.what());
    }
  }

  return object_pose;
}

BT::NodeStatus
Pick::tick()
{
  if (!pick_action_sent_)
  {
    std::string goal;
    getInput<std::string>("goal", goal);
    RCLCPP_INFO(node_->get_logger(), "Picking a %s", goal.c_str());
    moveit_msgs::msg::Grasp msg;
    msg.id = goal;
    msg.grasp_pose = getObjectTF(goal);
    pick_pub_->publish(msg);
    pick_action_sent_ = true;
  }

  if (result_ == 0)
  {
    return BT::NodeStatus::RUNNING;
  }
  else if (result_ == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    result_ = 0;
    RCLCPP_INFO(node_->get_logger(), "Success!");
    return BT::NodeStatus::SUCCESS;
  } 
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Pick error: MoveItErrorCodes[%i]", result_);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace gb_manipulation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_manipulation::Pick>("Pick");
}
