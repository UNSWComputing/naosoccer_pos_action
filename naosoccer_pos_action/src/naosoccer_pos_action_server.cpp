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

#include "naosoccer_pos_action/naosoccer_pos_action_server.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "indexes.hpp"
#include "parser.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = boost::filesystem;

namespace naosoccer_pos_action_server
{

NaosoccerPosActionServer::NaosoccerPosActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node{"naosoccer_pos_action_server_node", options}, pos_in_action_(false)
{
  this->declare_parameter<std::string>("pos_folder", "pos/");
  this->declare_parameter<bool>("parse_on_initialise", true);

  pub_joint_positions_ = create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "/effectors/joint_positions", rclcpp::SensorDataQoS());
  pub_joint_stiffnesses_ = create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
    "/effectors/joint_stiffnesses", rclcpp::SensorDataQoS());

  sub_joint_states_ = create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
    "/sensors/joint_positions", rclcpp::SensorDataQoS(),
    [this](nao_lola_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
      if (pos_in_action_) {
        calculateEffectorJoints(*sensor_joints);
      }
    });

  action_server_ = rclcpp_action::create_server<PosAction>(
    this, "naosoccer_pos_action",
    std::bind(
      &NaosoccerPosActionServer::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&NaosoccerPosActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&NaosoccerPosActionServer::handle_accepted, this, std::placeholders::_1));

  this->get_parameter<std::string>("pos_folder", pos_folder_);
  if (!pos_folder_.empty() && pos_folder_.back() == '/') {
    pos_folder_.pop_back();
  }
  pos_folder_.push_back('/');
  bool parse_on_initialise;
  this->get_parameter<bool>("parse_on_initialise", parse_on_initialise);
  if (parse_on_initialise) {
    if (canParsePosFolder(pos_folder_)) {
      RCLCPP_INFO(this->get_logger(), "Successfully parsed pos folder");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse pos folder");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Skipping parse on initialise");
  }

  RCLCPP_INFO(this->get_logger(), "naosoccer_pos_action_server_node initialized");
}

NaosoccerPosActionServer::~NaosoccerPosActionServer() {}

bool NaosoccerPosActionServer::canParsePosFolder(std::string & folder_path)
{
  if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid folder path: %s", folder_path.c_str());
    return false;
  }
  bool success = true;
  try {
    for (const auto & entry : fs::recursive_directory_iterator(folder_path)) {
      if (fs::is_regular_file(entry)) {
        std::string filePath = entry.path().string();
        bool isValid = canParsePosFile(filePath);
        if (isValid) {
          RCLCPP_DEBUG(this->get_logger(), ("Valid pos file: " + filePath).c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), ("Invalid pos file: " + filePath).c_str());
          success = false;
        }
      }
    }
  } catch (const fs::filesystem_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Filesystem error %s", e.what());
    success = false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    success = false;
  }
  return success;
}

bool NaosoccerPosActionServer::canParsePosFile(std::string & file_path)
{
  std::ifstream ifstream(file_path);
  if (ifstream.is_open()) {
    auto lines = readLines(ifstream);
    auto parseResult = parser::parse(lines);
    if (parseResult.successful) {
      return true;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), ("Could not open file:  " + file_path).c_str());
  }
  return false;
}

void NaosoccerPosActionServer::readPosFile(std::string & file_path)
{
  std::ifstream ifstream(file_path);
  if (ifstream.is_open()) {
    RCLCPP_DEBUG(this->get_logger(), ("Pos file succesfully loaded from " + file_path).c_str());
    file_successfully_read_ = true;
    auto lines = readLines(ifstream);
    auto parseResult = parser::parse(lines);
    file_successfully_read_ = parseResult.successful;
    key_frames_ = parseResult.keyFrames;
  } else {
    RCLCPP_ERROR(this->get_logger(), ("Could not open file:  " + file_path).c_str());
    file_successfully_read_ = false;
  }
}

std::string NaosoccerPosActionServer::getFullFilePath(std::string & filename)
{
  // Support absolute paths too
  if (fs::path(pos_folder_).is_absolute()) {
    return (fs::path(pos_folder_) / filename).string();
  }
  std::string file = pos_folder_ + filename;
  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("naosoccer_pos_action");

  fs::path dir_path(package_share_directory);
  fs::path file_path(file);
  fs::path full_path = dir_path / file_path;
  return full_path.string();
}

std::vector<std::string> NaosoccerPosActionServer::readLines(std::ifstream & ifstream)
{
  std::vector<std::string> ret;

  while (!ifstream.eof()) {
    std::string line;
    std::getline(ifstream, line);
    ret.push_back(line);
  }

  return ret;
}

float NaosoccerPosActionServer::findElem(
  const std::vector<uint8_t> & indexes, const std::vector<float> & data, uint8_t joint)
{
  for (uint8_t a = 0; a < indexes.size(); a++) {
    if (indexes.at(a) == joint) {
      return data.at(a);
    }
  }
  return NAN;
}

const KeyFrame & NaosoccerPosActionServer::findPreviousKeyFrame(int time_ms)
{
  for (auto it = key_frames_.rbegin(); it != key_frames_.rend(); ++it) {
    const auto & keyFrame = *it;
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms >= keyFrameDeadline) {
      return keyFrame;
    }
  }

  return *key_frame_start_;
}

const KeyFrame & NaosoccerPosActionServer::findNextKeyFrame(int time_ms)
{
  for (const auto & keyFrame : key_frames_) {
    int keyFrameDeadline = keyFrame.t_ms;
    if (time_ms < keyFrameDeadline) {
      return keyFrame;
    }
  }

  RCLCPP_ERROR(this->get_logger(), "findKeyFrame: Should never reach here");
  return key_frames_.back();
}

bool NaosoccerPosActionServer::posFinished(int time_ms)
{
  if (key_frames_.size() == 0) {
    return true;
  }

  const auto lastKeyFrame = key_frames_.back();
  int lastKeyFrameTime = lastKeyFrame.t_ms;
  if (time_ms >= lastKeyFrameTime) {
    return true;
  }

  return false;
}

void NaosoccerPosActionServer::calculateEffectorJoints(
  nao_lola_sensor_msgs::msg::JointPositions & sensor_joints)
{
  int time_ms = (rclcpp::Node::now() - initial_time_).nanoseconds() / 1e6;

  if (posFinished(time_ms)) {
    // We've finished the motion, set to DONE
    handlePosFinished();
    return;
  }

  if (firstTickSinceActionStarted_) {
    nao_lola_command_msgs::msg::JointPositions command;
    command.indexes = indexes::indexes;
    command.positions = std::vector<float>(
      sensor_joints.positions.begin(), sensor_joints.positions.end());
    key_frame_start_ =
      std::make_unique<KeyFrame>(0, command, nao_lola_command_msgs::msg::JointStiffnesses{});
    firstTickSinceActionStarted_ = false;
  }

  RCLCPP_DEBUG(this->get_logger(), ("time_ms is: " + std::to_string(time_ms)).c_str());

  const auto & previousKeyFrame = findPreviousKeyFrame(time_ms);
  const auto & nextKeyFrame = findNextKeyFrame(time_ms);

  float timeFromPreviousKeyFrame = time_ms - previousKeyFrame.t_ms;
  float timeToNextKeyFrame = nextKeyFrame.t_ms - time_ms;
  float duration = timeFromPreviousKeyFrame + timeToNextKeyFrame;

  RCLCPP_DEBUG(
    this->get_logger(), ("timeFromPreviousKeyFrame, timeFromPreviousKeyFrame, duration: " +
    std::to_string(timeFromPreviousKeyFrame) + ", " + std::to_string(timeToNextKeyFrame) + ", " +
    std::to_string(duration)).c_str());

  float alpha = timeToNextKeyFrame / duration;
  float beta = timeFromPreviousKeyFrame / duration;

  RCLCPP_DEBUG(
    this->get_logger(), ("alpha, beta: " + std::to_string(alpha) + ", " + std::to_string(
      beta)).c_str());

  nao_lola_command_msgs::msg::JointPositions effector_joints;
  effector_joints.indexes = indexes::indexes;

  for (unsigned int i = 0; i < nao_lola_command_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    float previous = previousKeyFrame.positions.positions.at(i);
    float next = nextKeyFrame.positions.positions.at(i);
    effector_joints.positions.push_back(previous * alpha + next * beta);

    RCLCPP_DEBUG(
      this->get_logger(), ("previous, next, result: " + std::to_string(
        previous) + ", " + std::to_string(next) + ", " +
      std::to_string(effector_joints.positions.at(i))).c_str());
  }

  pub_joint_positions_->publish(effector_joints);
  pub_joint_stiffnesses_->publish(nextKeyFrame.stiffnesses);
}

void NaosoccerPosActionServer::handlePosFinished()
{
  pos_in_action_ = false;
  RCLCPP_DEBUG(this->get_logger(), "Pos finished");

  std::function<void()> callback;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback = std::move(pos_finished_callback_);
    unsetPosFinishedCallback();
  }

  if (callback) {
    callback();
  }
}

rclcpp_action::GoalResponse NaosoccerPosActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PosAction::Goal> goal)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_INFO(get_logger(), ("Received goal request for:  " + goal->action_name).c_str());
  (void)uuid;
  (void)goal;

  if (!pos_in_action_) {
    std::string filename = goal->action_name + ".pos";
    std::string path = getFullFilePath(filename);
    readPosFile(path);
    RCLCPP_INFO(get_logger(), ("found pos file:  " + path).c_str());
    if (file_successfully_read_) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse NaosoccerPosActionServer::handle_cancel(
  const std::shared_ptr<ServerGoalHandlePosAction>
  goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  pos_in_action_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NaosoccerPosActionServer::handle_accepted(
  const std::shared_ptr<ServerGoalHandlePosAction> goal_handle)
{
  std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
}

void NaosoccerPosActionServer::execute(const std::shared_ptr<ServerGoalHandlePosAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Starting Pos Action");
  initial_time_ = rclcpp::Node::now();
  pos_in_action_ = true;
  firstTickSinceActionStarted_ = true;
  rclcpp::Rate loop_rate(5);  // 5 Hz, or 200ms

  auto feedback = std::make_shared<PosAction::Feedback>();
  auto result = std::make_shared<PosAction::Result>();

  auto pos_finished_callback = [this, goal_handle, result]() {
      result->success = true;
      result->message = "Pos action completed successfully";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    };

  setPosFinishedCallback(pos_finished_callback);

  while (pos_in_action_ && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "pos action was canceled.";
      goal_handle->canceled(result);
      unsetPosFinishedCallback();
      RCLCPP_DEBUG(this->get_logger(), "pos action goal canceled");
      return;
    }

    int time_ms = (rclcpp::Node::now() - initial_time_).nanoseconds() / 1e6;
    const auto pos_time = key_frames_.back().t_ms;
    feedback->progress = time_ms / pos_time;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Feedback: progress = %d%%", feedback->progress);

    loop_rate.sleep();
  }
  if (rclcpp::ok()) {
    result->success = true;
    result->message = "Pos action completed successfully";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  } else {
    result->success = false;
    result->message = "pos action was aborted.";
    goal_handle->abort(result);
    unsetPosFinishedCallback();
    RCLCPP_ERROR(this->get_logger(), "Goal aborted");
  }
}

void NaosoccerPosActionServer::setPosFinishedCallback(std::function<void()> callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pos_finished_callback_ = std::move(callback);
}

void NaosoccerPosActionServer::unsetPosFinishedCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pos_finished_callback_ = nullptr;
}

}  // namespace naosoccer_pos_action_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(naosoccer_pos_action_server::NaosoccerPosActionServer)
