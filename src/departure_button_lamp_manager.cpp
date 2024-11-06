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

#include <fstream>

#include <departure_button_lamp_manager/departure_button_lamp_manager.hpp>

namespace  departure_button_lamp_manager
{

DepartureButtonLampManager::DepartureButtonLampManager(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
): Node("departure_button_lamp_manager", options)

{
    // subscriber
    sub_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
    "autoware_state_machine/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&DepartureButtonLampManager::callbackStateMessage, this, std::placeholders::_1)
  );

    // publisher
    pub_departure_button_lamp_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
        "button_lamp_out",
        rclcpp::QoS{3}.transient_local());
    
    active_polarity_ = ACTIVE_POLARITY;
}

DepartureButtonLampManager::~DepartureButtonLampManager()
{
    publishLampState(false);
}

void DepartureButtonLampManager::callbackStateMessage(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
{

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[DepartureButtonLampManager::callbackStateMessage]"
        "service_layer_state: %u, control_layer_state: %u",
        msg->service_layer_state,
        msg->control_layer_state);
    
  lampManager(msg->service_layer_state, msg->control_layer_state);
}

void DepartureButtonLampManager::publishLampState(const bool value)
{
    dio_ros_driver::msg::DIOPort msg;
    msg.value = active_polarity_ ? value : !value;

    pub_departure_button_lamp_->publish(msg);
}

void DepartureButtonLampManager::lampManager(
    const uint16_t service_layer_state, const uint8_t control_layer_state)
{
    if (service_layer_state==autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION 
		    && control_layer_state==autoware_state_machine_msgs::msg::StateMachine::AUTO)
    {
        publishLampState(true);
    } else {
        publishLampState(false);
    }
}
}// namespace departurre_button_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(departure_button_lamp_manager::DepartureButtonLampManager)
