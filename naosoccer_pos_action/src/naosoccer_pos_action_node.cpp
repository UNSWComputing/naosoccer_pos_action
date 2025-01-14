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

#include <algorithm>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"

#include "indexes.hpp"
#include "parser.hpp"

namespace fs = boost::filesystem;

namespace naosoccer_pos_action
{

NaosoccerPosActionNode::NaosoccerPosActionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"NaosoccerPosActionNode", options}
{


}

NaosoccerPosActionNode::~NaosoccerPosActionNode() {}


}  // namespace naosoccer_pos_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(naosoccer_pos_action::NaosoccerPosActionNode)
