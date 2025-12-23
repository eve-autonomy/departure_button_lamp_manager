// Copyright 2024 eve autonomy inc. All Rights Reserved.
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
// limitations under the License

#include <departure_button_lamp_manager/departure_button_lamp_manager.hpp>
#include <fstream>

namespace departure_button_lamp_manager
{

DepartureButtonLampManager::DepartureButtonLampManager(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("departure_button_lamp_manager", options)

{
  // subscriber
  sub_routing_state_ = this->create_subscription<RouteState>(
    "/api/routing/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&DepartureButtonLampManager::onState, this, std::placeholders::_1));

  sub_routing_route_ = this->create_subscription<Route>(
    "/api/routing/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&DepartureButtonLampManager::onRoute, this, std::placeholders::_1));

  sub_operation_mode_state_ = this->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    std::bind(&DepartureButtonLampManager::onOperationModeState, this, std::placeholders::_1));

  // publisher
  pub_departure_button_lamp_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "button_lamp_out", rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;
}

DepartureButtonLampManager::~DepartureButtonLampManager() { publishLampState(false); }

void DepartureButtonLampManager::onState(const RouteState::ConstSharedPtr msg)
{
  state_ = msg->state;
  lampManager();
}

void DepartureButtonLampManager::onRoute(const Route::ConstSharedPtr msg)
{
  route_.data = msg->data;
  lampManager();
}

void DepartureButtonLampManager::onOperationModeState(const OperationModeState::ConstSharedPtr msg)
{
  is_autoware_control_ = msg ->is_autoware_control_enabled;
  is_in_transition_ = msg ->is_in_transition;
  mode_ = msg ->mode;
  lampManager();
}

void DepartureButtonLampManager::publishLampState(const bool value)
{
  dio_ros_driver::msg::DIOPort msg;
  msg.value = active_polarity_ ? value : !value;

  pub_departure_button_lamp_->publish(msg);
}

void DepartureButtonLampManager::lampManager()
{
  const bool is_ready =
    state_ == autoware_adapi_v1_msgs::msg::RouteState::SET &&
    !route_.data.empty() &&
    is_autoware_control_ &&
    !is_in_transition_ &&
    mode_ != OperationModeState::AUTONOMOUS;

  publishLampState(is_ready);
}
}  // namespace departure_button_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(departure_button_lamp_manager::DepartureButtonLampManager)
