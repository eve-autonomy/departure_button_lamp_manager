# departure_button_lamp_manager

## Overview
This node controls departure LED lamp for logistics workers turough `/dio_ros_driver` to inform if the vehicle is able to departure.

# Input and Output

- Input
    - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine_msgs/tree/main)
    - `/autoware_state_machine/state` \[[autoware_state_machine_msgs/msg/StateMachine](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateMachine.msg)\]:<br>State of the system.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver/)
    - `/dio/dout4`  \[[dio_ros_driver/msg/DIOPort](https://github.com/tier4/dio_ros_driver/blob/develop/ros2/msg/DIOPort.msg)\]:<br>GPIO output topic for departure lamp. (this topic is remapping from button_lamp_out)
## Node Graph


