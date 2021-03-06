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

#include "gb_manipulation/behavior_tree_nodes/Place.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_manipulation
{

Place::Place(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("place_action_comms");
  place_pub_ = node_->create_publisher<moveit_msgs::msg::PlaceLocation>("/moveit/place", 1);
  result_sub_ = node_->create_subscription<moveit_msgs::msg::MoveItErrorCodes>(
    "/moveit/result",
    1,
    std::bind(&Place::resultCallback, this, std::placeholders::_1));
    
  rclcpp::Node::SharedPtr bt_node;
  config().blackboard->get("node", bt_node);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!bt_node->has_parameter("place_locations")) {
    bt_node->declare_parameter("place_locations");
  }

  if (!bt_node->has_parameter("place_locations_coords")) {
    bt_node->declare_parameter("place_locations_coords");
  }

  if (bt_node->has_parameter("place_locations")) {
    std::vector<std::string> place_names;

    bt_node->get_parameter_or("place_locations", place_names, {});

    for (auto & place : place_names) {
      if (!bt_node->has_parameter("place_locations_coords." + place)) {
        bt_node->declare_parameter("place_locations_coords." + place);
      }

      std::vector<double> coords;
      if (bt_node->get_parameter_or("place_locations_coords." + place, coords, {})) {
        geometry_msgs::msg::Point point;
        point.x = coords[0];
        point.y = coords[1];
        point.z = coords[2];
        places_[place] = point;
      } else {
        std::cerr << "No coordinate configured for waypoint [" << place << "]" << std::endl;
      }
    }
  }

  place_action_sent_ = false;
}

void
Place::halt()
{
  std::cout << "Place halt" << std::endl;
}

void 
Place::resultCallback(const moveit_msgs::msg::MoveItErrorCodes::SharedPtr msg)
{
  result_ = msg->val;
}

bool
Place::getPlacePos(std::string id, geometry_msgs::msg::PoseStamped & pose)
{
  if (places_.find(id) != places_.end()) 
  {
    auto pos = places_[id];
    try {
      // Check if the transform is available
      auto tf = tf_buffer_->lookupTransform("base_footprint", "map", tf2::TimePointZero, tf2::durationFromSec(2.0));
      tf2::Transform bf2map, map2placePose, bf2placePose;
      tf2::fromMsg(tf.transform, bf2map);
      map2placePose.setOrigin({pos.x, pos.y, pos.z});
      bf2placePose = bf2map * map2placePose;

      pose.header.frame_id = "base_footprint";
      pose.pose.position.x = bf2placePose.getOrigin().x();
      pose.pose.position.y = bf2placePose.getOrigin().y();
      pose.pose.position.z = bf2placePose.getOrigin().z();
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      pose.header.stamp = node_->now();

    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "%s", e.what());
      return false;
    }
  }
  else
  {
    std::cerr << "No coordinate for waypoint [" << id << "]" << std::endl;
    return false;
  }
  
  return true;
}

BT::NodeStatus
Place::tick()
{
  if (!place_action_sent_)
  {
    std::string goal;
    getInput<std::string>("goal", goal);
    RCLCPP_INFO(node_->get_logger(), "Placing to %s", goal.c_str());
    moveit_msgs::msg::PlaceLocation msg;
    msg.id = goal;
    if (getPlacePos(goal, msg.place_pose))
    {
      place_pub_->publish(msg);
      result_ = 0;
      place_action_sent_ = true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Place error, pose not finded");
    }
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
    RCLCPP_ERROR(node_->get_logger(), "Place error: MoveItErrorCodes[%i]", result_);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace gb_manipulation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_manipulation::Place>("Place");
}
