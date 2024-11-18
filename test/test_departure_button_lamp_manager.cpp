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

#include <gtest/gtest.h>

#include <autoware_state_machine_msgs/msg/state_machine.hpp>
#include <dio_ros_driver/msg/dio_port.hpp>
#include <rclcpp/rclcpp.hpp>

#include "departure_button_lamp_manager/departure_button_lamp_manager.hpp"

using autoware_state_machine_msgs::msg::StateMachine;
using dio_ros_driver::msg::DIOPort;

class DepartureButtonLampManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<departure_button_lamp_manager::DepartureButtonLampManager>(
      rclcpp::NodeOptions());

    auto qos = rclcpp::QoS(10).reliable().transient_local();

    pub_ = node_->create_publisher<StateMachine>("autoware_state_machine/state", qos);
    sub_ = node_->create_subscription<DIOPort>(
      "button_lamp_out", qos,
      [this](DIOPort::SharedPtr msg) { received_messages_.push_back(*msg); });
  }

  void TearDown() override { rclcpp::shutdown(); }

  void sendAndCheckMessage(uint16_t service_state, uint8_t control_state, bool expected_value)
  {
    received_messages_.clear();

    StateMachine msg;
    msg.service_layer_state = service_state;
    msg.control_layer_state = control_state;
    pub_->publish(msg);

    auto start_time = std::chrono::steady_clock::now();
    while (received_messages_.empty() &&
           std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
      rclcpp::spin_some(node_);
    }

    ASSERT_FALSE(received_messages_.empty())
      << "Message not received for service_state=" << service_state
      << ", control_state=" << control_state;

    bool actual_value = received_messages_.front().value;
    EXPECT_EQ(actual_value, expected_value)
      << "service_state=" << service_state << ", control_state=" << control_state
      << ", expected=" << expected_value << ", actual=" << actual_value;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<StateMachine>::SharedPtr pub_;
  rclcpp::Subscription<DIOPort>::SharedPtr sub_;
  std::vector<DIOPort> received_messages_;
};

TEST_F(DepartureButtonLampManagerTest, TestAllStateCombinations)
{
<<<<<<< HEAD
  {
    const std::vector<uint16_t> service_states = {
      StateMachine::STATE_UNDEFINED,
      StateMachine::STATE_DURING_WAKEUP,
      StateMachine::STATE_DURING_CLOSE,
      StateMachine::STATE_CHECK_NODE_ALIVE,
      StateMachine::STATE_DURING_RECEIVE_ROUTE,
      StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION,
      StateMachine::STATE_WAITING_CALL_PERMISSION,
      StateMachine::STATE_RUNNING,
      StateMachine::STATE_INFORM_ENGAGE,
      StateMachine::STATE_RUNNING_TOWARD_STOP_LINE,
      StateMachine::STATE_RUNNING_TOWARD_OBSTACLE,
      StateMachine::STATE_INSTRUCT_ENGAGE,
      StateMachine::STATE_TURNING_LEFT,
      StateMachine::STATE_TURNING_RIGHT,
      StateMachine::STATE_DURING_OBSTACLE_AVOIDANCE,
      StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION,
      StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE,
      StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY,
      StateMachine::STATE_INFORM_RESTART,
      StateMachine::STATE_ARRIVED_GOAL,
      StateMachine::STATE_EMERGENCY_STOP
    };

    const std::vector<uint8_t> control_states = {
      StateMachine::MANUAL, 
      StateMachine::AUTO
    };

    for (auto service_state : service_states) {
      for (auto control_state : control_states) {
        bool expected_value =
          !(service_state == STATE_WAITING_ENGAGE_INSTRUCTION && control_state == AUTO);
        sendAndCheckMessage(
          static_cast<uint16_t>(service_state), static_cast<uint8_t>(control_state),
          expected_value);
      }
    }
  }
=======
  const std::vector<ServiceState> service_states = {
    STATE_UNDEFINED,
    STATE_DURING_WAKEUP,
    STATE_DURING_CLOSE,
    STATE_CHECK_NODE_ALIVE,
    STATE_DURING_RECEIVE_ROUTE,
    STATE_WAITING_ENGAGE_INSTRUCTION,
    STATE_WAITING_CALL_PERMISSION,
    STATE_RUNNING,
    STATE_INFORM_ENGAGE,
    STATE_RUNNING_TOWARD_STOP_LINE,
    STATE_RUNNING_TOWARD_OBSTACLE,
    STATE_INSTRUCT_ENGAGE,
    STATE_TURNING_LEFT,
    STATE_TURNING_RIGHT,
    STATE_DURING_OBSTACLE_AVOIDANCE,
    STATE_STOP_DUETO_TRAFFIC_CONDITION,
    STATE_STOP_DUETO_APPROACHING_OBSTACLE,
    STATE_STOP_DUETO_SURROUNDING_PROXIMITY,
    STATE_INFORM_RESTART,
    STATE_ARRIVED_GOAL,
    STATE_EMERGENCY_STOP};

  const std::vector<ControlState> control_states = {MANUAL, AUTO};

  for (auto service_state : service_states) {
    for (auto control_state : control_states) {
      bool expected_value =
        !(service_state == STATE_WAITING_ENGAGE_INSTRUCTION && control_state == AUTO);
      sendAndCheckMessage(
        static_cast<uint16_t>(service_state), static_cast<uint8_t>(control_state), expected_value);
    }
  }
}
>>>>>>> 53b52be883725c0f328ca7b276bb0547a72e5781
