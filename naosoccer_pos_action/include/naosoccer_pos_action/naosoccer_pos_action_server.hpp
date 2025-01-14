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

#ifndef NAOSOCCER_POS_SERVER__NAOSOCCER_POS_ACTION_SERVER_HPP_
#define NAOSOCCER_POS_SERVER__NAOSOCCER_POS_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"

#include "naosoccer_pos_interfaces/action/pos_action.hpp"
#include "naosoccer_pos_action/key_frame.hpp"

namespace naosoccer_pos_action_server_ns
{

class NaosoccerPosActionServer : public rclcpp::Node
{
public:
  using PosAction = naosoccer_pos_action_interfaces::action::PosAction;

  explicit NaosoccerPosActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  virtual ~NaosoccerPosActionServer();

private:
  bool canParsePosFolder(std::string& folder_path);
  bool canParsePosFile(std::string& file_path);
  void readPosFile(std::string& file_path);
  std::string getFullFilePath(std::string& filename);
  std::vector<std::string> readLines(std::ifstream& ifstream);
  float findElem(const std::vector<uint8_t>& indexes, const std::vector<float>& data, uint8_t joint);
  const KeyFrame& findPreviousKeyFrame(int time_ms);
  const KeyFrame& findNextKeyFrame(int time_ms);
  bool posFinished(int time_ms);

  void calculateEffectorJoints(nao_lola_sensor_msgs::msg::JointPositions& sensor_joints);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const PosAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PosAction>> goal_handle);
  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<PosAction>> goal_handle);

  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr pub_joint_positions_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr pub_joint_stiffnesses_;

  rclcpp_action::Server<PosAction>::SharedPtr action_server_;

  std::string pos_folder_;
  bool file_successfully_read_ = false;
  std::vector<KeyFrame> key_frames_;
  std::atomic<bool> pos_in_action_;
  bool firstTickSinceActionStarted_ = true;
  std::unique_ptr<KeyFrame> key_frame_start_;
  rclcpp::Time initial_time_;
  std::vector<uint8_t> selected_joints_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<PosAction>> goal_handle_;

  std::mutex mutex_;
};

}  // namespace naosoccer_pos_action_server_ns

#endif  // NAOSOCCER_POS_SERVER__NAOSOCCER_POS_SERVER_HPP_
