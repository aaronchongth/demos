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

#ifndef COMPOSITION__LISTENER_COMPONENT_HPP_
#define COMPOSITION__LISTENER_COMPONENT_HPP_

#include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include <string>

namespace composition
{

class Listener : public nav2_util::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options);

	nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
	nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string topic_;
};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_
