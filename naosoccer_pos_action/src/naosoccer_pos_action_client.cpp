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

#include "naosoccer_pos_action/naosoccer_pos_action_node.hpp"

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "naosoccer_pos_action_interfaces/action/pos_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = boost::filesystem;

namespace naosoccer_pos_action_node
{

void NaosoccerPosActionNode::send_goal(std::string & action_name)
{
  using namespace std::placeholders;

  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting 5 seconds");
    rclcpp::shutdown();
  }

  auto goal_msg = PosAction::Goal();
  goal_msg.action_name = action_name;

  auto send_goal_options = rclcpp_action::Client<PosAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&NaosoccerPosActionNode::goal_response_callback, this, _1);

  // send_goal_options.feedback_callback =
  //   std::bind(&NaosoccerPosActionNode::feedback_callback, this, _1, _2);

  send_goal_options.result_callback = std::bind(&NaosoccerPosActionNode::result_callback, this, _1);

  RCLCPP_INFO(
    this->get_logger(), ("Sending goal request for pos file:  " + action_name + ".pos").c_str());

  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NaosoccerPosActionNode::goal_response_callback(const ClientGoalHandlePosAction::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

/*
void NaosoccerPosActionNode::feedback_callback(ClientGoalHandlePosAction::SharedPtr,
                                           const std::shared_ptr<const PosAction::Feedback> feedback)
{
  // TODO
}
*/

void NaosoccerPosActionNode::result_callback(const ClientGoalHandlePosAction::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Joints posisitions regulary played.");
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), " nao pos Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), " nao pos Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), " nao pos Unknown result code");
      return;
  }
}

void NaosoccerPosActionNode::send_kick_cancel()
{
  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Sending cancel request for kick...");

  // Cancel the ongoing goal
  client_ptr_->async_cancel_all_goals(
    std::bind(&NaosoccerPosActionNode::kick_cancel_response_callback, this, std::placeholders::_1)
  );
}

void NaosoccerPosActionNode::kick_cancel_response_callback(
  const rclcpp_action::Client<PosAction>::CancelResponse::SharedPtr & response)
{
  if (response->return_code == rclcpp_action::Client<PosAction>::CancelResponse::ERROR_NONE) {
    RCLCPP_INFO(this->get_logger(), "Cancel request accepted by server");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Cancel request was rejected by server");
  }
}

}  // namespace naosoccer_pos_action_node
