// Copyright 2024 Antonio Bono
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

#ifndef NAOSOCCER_POS_SERVER__NAOSOCCER_POS_ACTION_CLIENT_HPP_
#define NAOSOCCER_POS_SERVER__NAOSOCCER_POS_ACTION_CLIENT_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"

#include "naosoccer_pos_interfaces/action/pos_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = boost::filesystem;

namespace naosoccer_pos_action_client_ns
{

class NaosoccerPosActionClient : public rclcpp::Node
{
public:
  using PosAction = naosoccer_pos_interfaces::action::PosAction;
  using ClientGoalHandlePosAction = rclcpp_action::ClientGoalHandle<PosAction>;

  explicit NaosoccerPosActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  virtual ~NaosoccerPosActionClient();

private:
  void send_goal(std::string& action_name);
  void action_req_callback(const std_msgs::msg::String::SharedPtr msg);
  void goal_response_callback(
	  const ClientGoalHandlePosAction::SharedPtr& goal_handle);
  void feedback_callback(ClientGoalHandlePosAction::SharedPtr,
						 const std::shared_ptr<const PosAction::Feedback> feedback);
  void
  result_callback(const ClientGoalHandlePosAction::WrappedResult& result);

  rclcpp_action::Client<PosAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_action_req_;

};	// NaosoccerPosActionClient

}  // namespace naosoccer_pos_action_client_ns

#endif	// NAOSOCCER_POS_SERVER__NAOSOCCER_POS_ACTION_CLIENT_HPP_
