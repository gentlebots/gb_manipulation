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
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace std::chrono_literals;

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
  result_ = 0;
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
  RCLCPP_INFO(node_->get_logger(), "---------- resultCallback: %i", result_);
}

geometry_msgs::msg::PoseStamped 
Pick::pose2BaseFootprint(geometry_msgs::msg::PoseStamped input)
{
  tf2::Transform bf2frame, frame2obj, bf2Object;
  frame2obj.setOrigin({input.pose.position.x,
                      input.pose.position.y,
                      input.pose.position.z});
  frame2obj.setRotation({input.pose.orientation.x,
                        input.pose.orientation.y,
                        input.pose.orientation.z,
                        input.pose.orientation.w});

  geometry_msgs::msg::PoseStamped object_pose;
  try {
    // Check if the transform is available
    auto tf = tf_buffer_->lookupTransform(
      input.header.frame_id, "base_footprint", tf2::TimePointZero);
    
    tf2::fromMsg(tf.transform, bf2frame);
    bf2Object = bf2frame * frame2obj;
    
    object_pose.pose.position.x = bf2Object.getOrigin().x();
    object_pose.pose.position.y = bf2Object.getOrigin().y();
    object_pose.pose.position.z = bf2Object.getOrigin().z();
    object_pose.pose.orientation.x = bf2Object.getRotation().x();
    object_pose.pose.orientation.y = bf2Object.getRotation().y();
    object_pose.pose.orientation.z = bf2Object.getRotation().z();
    object_pose.pose.orientation.w = bf2Object.getRotation().w();
    object_pose.header.frame_id = "base_footprint";
    object_pose.header.stamp = node_->now();
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(node_->get_logger(), "%s", e.what());
  }

  return object_pose;
}

BT::NodeStatus
Pick::tick()
{
  if (!pick_action_sent_)
  {
    std::string object_id;
    getInput<std::string>("object_id", object_id);
    geometry_msgs::msg::PoseStamped object_pose;
    getInput<geometry_msgs::msg::PoseStamped>("object_pose", object_pose);
    RCLCPP_INFO(node_->get_logger(), "Picking a %s", object_id.c_str());
    moveit_msgs::msg::Grasp msg;
    msg.id = object_id;
    msg.grasp_pose = pose2BaseFootprint(object_pose);
    pick_pub_->publish(msg);
    pick_action_sent_ = true;
  }

  if (node_->now() > (timer_ + 30s))
  {
    result_ = 99999;
    RCLCPP_ERROR(node_->get_logger(), "Timeout reached. Pick error: MoveItErrorCodes[%i]. Jumping to the next step!", result_);
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
    return BT::NodeStatus::SUCCESS;
  }  
}

}  // namespace gb_manipulation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_manipulation::Pick>("Pick");
}
