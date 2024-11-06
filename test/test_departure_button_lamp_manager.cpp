#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <autoware_state_machine_msgs/msg/state_machine.hpp>
#include <dio_ros_driver/msg/dio_port.hpp>
#include "departure_button_lamp_manager/departure_button_lamp_manager.hpp"

using autoware_state_machine_msgs::msg::StateMachine;
using dio_ros_driver::msg::DIOPort;

class DepartureButtonLampManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<departure_button_lamp_manager::DepartureButtonLampManager>(rclcpp::NodeOptions());
        
        auto qos = rclcpp::QoS(10).reliable().transient_local();
        
        pub_ = node_->create_publisher<StateMachine>("autoware_state_machine/state", qos);
        sub_ = node_->create_subscription<DIOPort>(
            "button_lamp_out", qos, 
            [this](DIOPort::SharedPtr msg) { received_messages_.push_back(*msg); });
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    void sendAndCheckMessage(uint16_t service_state, uint8_t control_state, bool expected_value)
    {
        received_messages_.clear();
        
        StateMachine msg;
        msg.service_layer_state = service_state;
        msg.control_layer_state = control_state;
        pub_->publish(msg);

        auto start_time = std::chrono::steady_clock::now();
        while (received_messages_.empty() &&
               std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
        {
            rclcpp::spin_some(node_);
        }

        ASSERT_FALSE(received_messages_.empty()) << "Message not received for service_state=" 
                                                 << service_state << ", control_state=" << control_state;

        bool actual_value = received_messages_.front().value;
        EXPECT_EQ(actual_value, expected_value) << "service_state=" << service_state 
                                                << ", control_state=" << control_state 
                                                << ", expected=" << expected_value 
                                                << ", actual=" << actual_value;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<StateMachine>::SharedPtr pub_;
    rclcpp::Subscription<DIOPort>::SharedPtr sub_;
    std::vector<DIOPort> received_messages_;
};

TEST_F(DepartureButtonLampManagerTest, TestAllStateCombinations)
{
    const std::vector<uint16_t> service_states = {
        0, 100, 101, 102, 200, 201, 250, 300, 301, 302, 303, 304, 305, 
        306, 307, 402, 403, 404, 450, 500, 600
    };
    const std::vector<uint8_t> control_states = {0, 1};

    for (auto service_state : service_states) {
        for (auto control_state : control_states) {
            bool expected_value = !(service_state == 201 && control_state == 1);
            sendAndCheckMessage(service_state, control_state, expected_value);
        }
    }
}

