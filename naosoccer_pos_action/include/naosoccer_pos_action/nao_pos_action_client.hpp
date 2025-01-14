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

#ifndef NAO_POS_SERVER__NAO_POS_ACTION_CLIENT_HPP_
#define NAO_POS_SERVER__NAO_POS_ACTION_CLIENT_HPP_

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

#include "nao_pos_interfaces/action/pos_play.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = boost::filesystem;

namespace nao_pos_action_client_ns
{

class NaoPosActionClient : public rclcpp::Node
{
public:
  explicit NaoPosActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  virtual ~NaoPosActionClient();

private:
  void send_goal(std::string& action_name);
  void action_req_callback(const std_msgs::msg::String::SharedPtr msg);
  void goal_response_callback(
	  const rclcpp_action::ClientGoalHandle<nao_pos_interfaces::action::PosPlay>::SharedPtr& goal_handle);
  void feedback_callback(rclcpp_action::ClientGoalHandle<nao_pos_interfaces::action::PosPlay>::SharedPtr,
						 const std::shared_ptr<const nao_pos_interfaces::action::PosPlay::Feedback> feedback);
  void
  result_callback(const rclcpp_action::ClientGoalHandle<nao_pos_interfaces::action::PosPlay>::WrappedResult& result);

  rclcpp_action::Client<nao_pos_interfaces::action::PosPlay>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_action_req_;

};	// NaoPosActionClient

}  // namespace nao_pos_action_client_ns

#endif	// NAO_POS_SERVER__NAO_POS_ACTION_CLIENT_HPP_
