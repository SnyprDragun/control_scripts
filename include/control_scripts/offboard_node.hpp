/**
 * @brief Offboard control example
 * @file offboard_node.hpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#ifndef OFFBOARD_NODE_HPP_
#define OFFBOARD_NODE_HPP_

#include <mutex>
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

    /**
     * @brief Streams setpoints at current position, then switches to offboard mode.
     *        Must be called before any go_to() calls. Blocks for ~1 second while
     *        priming the setpoint stream (PX4 requires setpoints before mode switch).
     */
    void change_mode_offboard();

    /**
     * @brief Update the target position. The keepalive timer will continuously
     *        re-publish this target to maintain offboard mode.
     * @param x     Target x position (NED, meters)
     * @param y     Target y position (NED, meters)
     * @param z     Target z position (NED, meters — negative = up)
     * @param yaw   Target yaw angle in radians [-PI:PI], default 180 degrees
     */
    void go_to(float x, float y, float z, float yaw = -M_PI);

    void publish_offboard_control_mode();
    void setpoint(float x = 0.0f, float y = 0.0f, float z = -5.0f, float yaw = -M_PI);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

private:
    TimerBase::SharedPtr keepalive_timer_;

    Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    atomic<uint64_t> timestamp_;

    // Current target setpoint — updated by go_to(), streamed by keepalive_timer_
    float target_x_{0.0f};
    float target_y_{0.0f};
    float target_z_{-5.0f};
    float target_yaw_{-M_PI};
    bool offboard_active_{false};

    // Protects target_* writes (main thread) vs. reads (timer callback thread)
    mutex target_mutex_;
};

#endif