// Copyright 2024 Antonio Bono
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

#include "nao_pos_server/nao_pos_action_client.hpp"

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
#include "nao_pos_interfaces/action/pos_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = boost::filesystem;

namespace nao_pos_action_client_ns
{

using PosAction = nao_pos_interfaces::action::PosPlay;
using GoalHandlePosAction = rclcpp_action::ClientGoalHandle<PosAction>;

NaoPosActionClient::NaoPosActionClient(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_pos_action_client_node", options}
{
  using namespace std::placeholders;
  this->client_ptr_ = rclcpp_action::create_client<PosAction>(this, "nao_pos_action");

  /*this->timer_ = this->create_wall_timer(
                   std::chrono::milliseconds(500),
                   std::bind(&NaoPosActionClient::send_goal, this));
  */

  this->sub_action_req_ = this->create_subscription<std_msgs::msg::String>(
    "action_req", 10, std::bind(&NaoPosActionClient::action_req_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "NaoPosActionClient initialized");
}

NaoPosActionClient::~NaoPosActionClient() {}

void NaoPosActionClient::action_req_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "I heard: '%s'", msg->data.c_str());

  std::string action_name = msg->data;
  this->send_goal(action_name);
}

void NaoPosActionClient::send_goal(std::string & action_name)
{
  using namespace std::placeholders;

  // this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = PosAction::Goal();
  goal_msg.action_name = action_name;

  auto send_goal_options = rclcpp_action::Client<PosAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&NaoPosActionClient::goal_response_callback, this, _1);

  // send_goal_options.feedback_callback =
  //   std::bind(&NaoPosActionClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback = std::bind(&NaoPosActionClient::result_callback, this, _1);

  RCLCPP_INFO(
    this->get_logger(), ("Sending goal request for pos file:  " + action_name + ".pos").c_str());

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NaoPosActionClient::goal_response_callback(const GoalHandlePosAction::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

/*
void NaoPosActionClient::feedback_callback(GoalHandlePosAction::SharedPtr,
                                           const std::shared_ptr<const PosAction::Feedback> feedback)
{
  // TODO
}
*/

void NaoPosActionClient::result_callback(const GoalHandlePosAction::WrappedResult & result)
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

}  // namespace nao_pos_action_client_ns

RCLCPP_COMPONENTS_REGISTER_NODE(nao_pos_action_client_ns::NaoPosActionClient)