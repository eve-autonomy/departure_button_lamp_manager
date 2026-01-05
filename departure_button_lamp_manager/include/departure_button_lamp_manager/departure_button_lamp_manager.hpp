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

#ifndef DEPARTURE_BUTTON_LAMP_MANAGER__DEPARTURE_BUTTON_LAMP_MANAGER_HPP_
#define DEPARTURE_BUTTON_LAMP_MANAGER__DEPARTURE_BUTTON_LAMP_MANAGER_HPP_

#include "dio_ros_driver/msg/dio_port.hpp"
#include "rclcpp/rclcpp.hpp"

// sub input
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"

namespace departure_button_lamp_manager
{
using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using Route = autoware_adapi_v1_msgs::msg::Route;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;

class DepartureButtonLampManager : public rclcpp::Node
{
public:
  explicit DepartureButtonLampManager(const rclcpp::NodeOptions & options);
  ~DepartureButtonLampManager();

private:
#define ACTIVE_POLARITY (false)

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_departure_button_lamp_;

  // Subscriber
  rclcpp::Subscription<RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<Route>::SharedPtr sub_routing_route_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;

  // Callback
  void onState(const RouteState::ConstSharedPtr msg);
  void onRoute(const Route::ConstSharedPtr msg);
  void onOperationModeState(const OperationModeState::ConstSharedPtr msg);

  bool active_polarity_;

  void publishLampState(const bool value);
  void lampManager();

  // member variables
  uint16_t state_;
  Route route_;
  uint8_t mode_;
  bool is_autoware_control_;
  bool is_in_transition_;
};

}  // namespace departure_button_lamp_manager
#endif  // DEPARTURE_BUTTON_LAMP_MANAGER__DEPARTURE_BUTTON_LAMP_MANAGER_HPP_
