// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "composition/listener_component.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

std::string StripSlash(const std::string &in)
{
  std::string out = in;
  if ((!in.empty()) && (in[0] == '/'))
    out.erase(0, 1);
  return out;
}

// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
Listener::Listener(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("listener", "", options)
{
  declare_parameter<std::string>("topic_name", "/chatter");
  get_parameter<std::string>("topic_name", topic_);
  topic_ = StripSlash(topic_);
}

nav2_util::CallbackReturn
  Listener::on_configure(const rclcpp_lifecycle::State &) {
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
  auto callback =
    [](std_msgs::msg::String::ConstSharedPtr msg) -> void
    {
      std::cout << "heard " << msg->data.c_str() << std::endl;
      // RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
      // std::flush(std::cout);
    };

  // Create a subscription to the "chatter" topic which can be matched with one or more
  // compatible ROS publishers.
  // Note that not all publishers on the same topic with the same type will be compatible:
  // they must have compatible Quality of Service policies.
  sub_ = create_subscription<std_msgs::msg::String>(topic_, 10, callback);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
  Listener::on_activate(const rclcpp_lifecycle::State &) {
  // sub_->on_activate();
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
  Listener::on_deactivate(const rclcpp_lifecycle::State &) {
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener)
