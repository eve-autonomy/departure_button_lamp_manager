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

#include "departure_button_lamp_manager/departure_button_lamp_manager.hpp"

#include <dio_ros_driver/msg/dio_port.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using Route = autoware_adapi_v1_msgs::msg::Route;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using DIOPort = dio_ros_driver::msg::DIOPort;

class DepartureButtonLampManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<departure_button_lamp_manager::DepartureButtonLampManager>(
      rclcpp::NodeOptions());

    auto qos = rclcpp::QoS(1).reliable().transient_local();

    pub_route_state_ = node_->create_publisher<RouteState>("/api/routing/state", qos);
    pub_route_ = node_->create_publisher<Route>("/api/routing/route", qos);
    pub_operation_mode_ =
      node_->create_publisher<OperationModeState>("/api/operation_mode/state", qos);

    sub_ = node_->create_subscription<DIOPort>(
      "button_lamp_out", qos,
      [this](DIOPort::SharedPtr msg) { received_messages_.push_back(*msg); });
  }

  void TearDown() override { rclcpp::shutdown(); }

  void spinUntilMessage()
  {
    auto start_time = std::chrono::steady_clock::now();
    while (received_messages_.empty() &&
           std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
      rclcpp::spin_some(node_);
    }
  }

  void publishState(uint16_t state)
  {
    RouteState msg;
    msg.state = state;
    pub_route_state_->publish(msg);
  }

  void publishRoute(bool has_data)
  {
    Route msg;
    if (has_data) {
      autoware_adapi_v1_msgs::msg::RouteData route_data;
      msg.data.push_back(route_data);
    }
    pub_route_->publish(msg);
  }

  void publishOperationMode(bool is_autoware_control, bool is_in_transition, uint8_t mode)
  {
    OperationModeState msg;
    msg.is_autoware_control_enabled = is_autoware_control;
    msg.is_in_transition = is_in_transition;
    msg.mode = mode;
    pub_operation_mode_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<RouteState>::SharedPtr pub_route_state_;
  rclcpp::Publisher<Route>::SharedPtr pub_route_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Subscription<DIOPort>::SharedPtr sub_;
  std::vector<DIOPort> received_messages_;
};

// ランプが点灯する条件：
// - state_ == RouteState::SET
// - !route_.data.empty()
// - is_autoware_control_
// - !is_in_transition_
// - mode_ != OperationModeState::AUTONOMOUS
// ACTIVE_POLARITY = false なので、value は反転される

TEST_F(DepartureButtonLampManagerTest, LampOnWhenAllConditionsMet)
{
  received_messages_.clear();

  publishRoute(true);
  publishOperationMode(true, false, OperationModeState::STOP);
  publishState(RouteState::SET);

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
  // ACTIVE_POLARITY = false なので、is_ready=true のとき value=false
  EXPECT_FALSE(received_messages_.back().value);
}

TEST_F(DepartureButtonLampManagerTest, LampOffWhenRouteNotSet)
{
  received_messages_.clear();

  publishRoute(true);
  publishOperationMode(true, false, OperationModeState::STOP);
  publishState(RouteState::UNSET);

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
  // ACTIVE_POLARITY = false なので、is_ready=false のとき value=true
  EXPECT_TRUE(received_messages_.back().value);
}

TEST_F(DepartureButtonLampManagerTest, LampOffWhenRouteEmpty)
{
  received_messages_.clear();

  publishRoute(false);
  publishOperationMode(true, false, OperationModeState::STOP);
  publishState(RouteState::SET);

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
  EXPECT_TRUE(received_messages_.back().value);
}

TEST_F(DepartureButtonLampManagerTest, LampOffWhenNotAutowareControl)
{
  received_messages_.clear();

  publishRoute(true);
  publishOperationMode(false, false, OperationModeState::STOP);
  publishState(RouteState::SET);

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
  EXPECT_TRUE(received_messages_.back().value);
}

TEST_F(DepartureButtonLampManagerTest, LampOffWhenInTransition)
{
  received_messages_.clear();

  publishRoute(true);
  publishOperationMode(true, true, OperationModeState::STOP);
  publishState(RouteState::SET);

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
  EXPECT_TRUE(received_messages_.back().value);
}

TEST_F(DepartureButtonLampManagerTest, LampOffWhenAutonomousMode)
{
  received_messages_.clear();

  publishRoute(true);
  publishOperationMode(true, false, OperationModeState::AUTONOMOUS);
  publishState(RouteState::SET);

  spinUntilMessage();

  ASSERT_FALSE(received_messages_.empty());
  EXPECT_TRUE(received_messages_.back().value);
}
