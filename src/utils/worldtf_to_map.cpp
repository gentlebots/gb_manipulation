// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include <random>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


class WorldTF2Map : public rclcpp::Node
{
public:
  WorldTF2Map()
  : rclcpp::Node("worldtf_to_map")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void step()
  {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "sz_5", tf2::TimePointZero);    
      RCLCPP_INFO(get_logger(),
      "map -> object [%f, %f, %f]",
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
    }
    
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorldTF2Map>();
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
  }

  rclcpp::shutdown();

  return 0;
}
