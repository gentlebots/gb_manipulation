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

#include "gb_manipulation/behavior_tree_nodes/PlaceSimple.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_manipulation
{

PlaceSimple::PlaceSimple(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("place_action_comms");
  place_pub_ = node_->create_publisher<moveit_msgs::msg::PlaceLocation>("/moveit/place", 1);
  result_sub_ = node_->create_subscription<moveit_msgs::msg::MoveItErrorCodes>(
    "/moveit/result",
    1,
    std::bind(&PlaceSimple::resultCallback, this, std::placeholders::_1));
    
  rclcpp::Node::SharedPtr bt_node;
  config().blackboard->get("node", bt_node);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  place_action_sent_ = false;
}

void
PlaceSimple::halt()
{
  std::cout << "PlaceSimple halt" << std::endl;
}

void 
PlaceSimple::resultCallback(const moveit_msgs::msg::MoveItErrorCodes::SharedPtr msg)
{
  result_ = msg->val;
}

double getYaw(geometry_msgs::msg::Quaternion mQ) {
    
  tf2::Quaternion tQ(
      mQ.x,
      mQ.y,
      mQ.z,
      mQ.w);
  tf2::Matrix3x3 m(tQ);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}


geometry_msgs::msg::PoseStamped PlaceSimple::pose2BaseFootprint(geometry_msgs::msg::PoseStamped input)
{
  tf2::Transform frame2bf, obj2frame, bf2Object;
  obj2frame.setOrigin({input.pose.position.x,
                      input.pose.position.y,
                      input.pose.position.z});
  obj2frame.setRotation({input.pose.orientation.x,
                        input.pose.orientation.y,
                        input.pose.orientation.z,
                        input.pose.orientation.w});

  geometry_msgs::msg::PoseStamped object_pose;
  try {
    // Check if the transform is available
    auto tf = tf_buffer_->lookupTransform(
      input.header.frame_id, "base_footprint", tf2::TimePointZero);
    
    tf2::fromMsg(tf.transform, frame2bf);
    bf2Object =  obj2frame * frame2bf;
    
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
PlaceSimple::tick()
{
  if (!place_action_sent_)
  {
    geometry_msgs::msg::PoseStamped goal;
    getInput<geometry_msgs::msg::PoseStamped>("goal", goal);    
    RCLCPP_INFO(node_->get_logger(), "Placing to:  x:%f  y:%f  z:%f yaw:%f", 
      goal.pose.position.x, 
      goal.pose.position.y, 
      goal.pose.position.z, 
      getYaw(goal.pose.orientation));
		
    moveit_msgs::msg::PlaceLocation msg;
    msg.place_pose = pose2BaseFootprint(goal);
    place_pub_->publish(msg);
    result_ = 0;
    place_action_sent_ = true;
    
  }

  rclcpp::spin_some(node_);

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
    RCLCPP_ERROR(node_->get_logger(), "PlaceSimple error: MoveItErrorCodes[%i]", result_);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace gb_manipulation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_manipulation::PlaceSimple>("PlaceSimple");
}
