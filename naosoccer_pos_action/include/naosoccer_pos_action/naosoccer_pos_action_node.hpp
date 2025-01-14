// Copyright 2023 Kenji Brameld
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

#ifndef NAOSOCCER_POS_ACTION__NAOSOCCER_POS_ACTION_NODE_HPP_
#define NAOSOCCER_POS_ACTION__NAOSOCCER_POS_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "naosoccer_pos_action/key_frame.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "naosoccer_pos_action_interfaces/action/action.hpp"


namespace naosoccer_pos_action
{

class NaosoccerPosActionNode : public rclcpp::Node
{
public:
  explicit NaosoccerPosActionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~NaosoccerPosActionNode();

private:

};

}  // namespace naosoccer_pos_action

#endif  // NAOSOCCER_POS_ACTION__NAOSOCCER_POS_ACTION_NODE_HPP_
