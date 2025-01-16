// Copyright 2021 Kenji Brameld
// Copyright 2025 Reilly Fox
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

#include "naosoccer_pos_action/naosoccer_pos_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <utility>
#include <memory>

namespace naosoccer_pos_action_node
{

NaosoccerPosActionNode::NaosoccerPosActionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"NaosoccerPosActionNode", options}
{
  sub_action_req_ = this->create_subscription<std_msgs::msg::String>(
    "action_req", 1,
    std::bind(&NaosoccerPosActionNode::action_req_callback, this, std::placeholders::_1));
  pub_action_feedback_ = this->create_publisher<std_msgs::msg::String>("action_feedback", 10);
  pub_action_finished_ = this->create_publisher<std_msgs::msg::String>("action_finished", 10);

  RCLCPP_INFO(this->get_logger(), "NaosoccerPosActionNode initialized");

  // Client
  client_ptr_ = rclcpp_action::create_client<PosAction>(this, "naosoccer_pos_action");

}

NaosoccerPosActionNode::~NaosoccerPosActionNode() {}

void NaosoccerPosActionNode::action_req_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "I heard: '%s'", msg->data.c_str());

  std::string action_name = msg->data;
  send_goal(action_name);
}

}  // namespace naosoccer_pos_action_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(naosoccer_pos_action_node::NaosoccerPosActionNode)
