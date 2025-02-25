# Copyright 2024 eve autonomy inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(departure_button_lamp_manager)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(departure_button_lamp_manager SHARED
  src/departure_button_lamp_manager.cpp
)

rclcpp_components_register_node(departure_button_lamp_manager
  PLUGIN "departure_button_lamp_manager::DepartureButtonLampManager"
  EXECUTABLE departure_button_lamp_manager_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(autoware_state_machine_msgs REQUIRED)
  find_package(dio_ros_driver REQUIRED)

  ament_add_gtest(test_departure_button_lamp_manager
    test/test_departure_button_lamp_manager.cpp)
  target_link_libraries(test_departure_button_lamp_manager
    departure_button_lamp_manager rclcpp::rclcpp)
  ament_target_dependencies(test_departure_button_lamp_manager
    autoware_state_machine_msgs dio_ros_driver)
endif()

ament_export_dependencies(
  autoware_state_machine_msgs dio_ros_driver
  )


ament_auto_package(INSTALL_TO_SHARE
  launch
)

