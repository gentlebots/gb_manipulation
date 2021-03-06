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

#ifndef GB_MANIPULATION__BEHAVIOR_TREE_NODES__PLACE_HPP_
#define GB_MANIPULATION__BEHAVIOR_TREE_NODES__PLACE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "moveit_msgs/msg/place_location.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace gb_manipulation
{

class Place : public BT::ActionNodeBase
{
public:
  explicit Place(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();
  bool getPlacePos(std::string id, geometry_msgs::msg::PoseStamped & pose);
  void resultCallback(const moveit_msgs::msg::MoveItErrorCodes::SharedPtr msg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::PlaceLocation>::SharedPtr place_pub_;
  rclcpp::Subscription<moveit_msgs::msg::MoveItErrorCodes>::SharedPtr result_sub_;
  std::map<std::string, geometry_msgs::msg::Point> places_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  int result_;
  bool place_action_sent_;
};

}  // namespace gb_manipulation

#endif  // GB_MANIPULATION__BEHAVIOR_TREE_NODES__PLACE_HPP_
