#ifndef OFFBOARD_NODE_HPP_
#define OFFBOARD_NODE_HPP_

#include <chrono>
#include <iostream>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std;
using namespace chrono;
using namespace rclcpp;
using namespace px4_msgs::msg;
using namespace chrono_literals;

class Offboard : public Node
{
public:
    Offboard();
    void arm();
    void disarm();

private:
    // Timer
    TimerBase::SharedPtr timer_;

    // Publishers
    Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif  // OFFBOARD_NODE_HPP_
