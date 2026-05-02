/**
 * @brief Takeoff node
 * @file takeoff_node.hpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#ifndef TAKEOFF_NODE_HPP_
#define TAKEOFF_NODE_HPP_

#include <chrono>
#include <iostream>
#include <atomic>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std;
using namespace chrono;
using namespace rclcpp;
using namespace px4_msgs::msg;
using namespace chrono_literals;

class Takeoff : public Node
{
public:
    Takeoff(int uav_id);

    /**
     * @brief Send a command to Arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

    /**
     * @brief Arm the vehicle, switch to AUTO.TAKEOFF mode, then block until the
     *        vehicle reaches the target altitude (within tolerance), as confirmed
     *        by vehicle_local_position subscription. The correct PX4 sequence is:
     *        (1) arm, (2) switch mode to AUTO.TAKEOFF, (3) monitor altitude.
     *        Sending VEHICLE_CMD_NAV_TAKEOFF alone is a mission command and will
     *        not arm or trigger flight without the mode switch.
     * @param altitude  Target takeoff altitude in meters (positive, NED converted internally)
     * @param tolerance Altitude tolerance in meters — how close counts as "reached" (default 0.3 m)
     * @param timeout_s Maximum seconds to wait before giving up and returning (default 30 s)
     */
    void takeoff(float altitude, float tolerance = 0.3f, float timeout_s = 30.0f);

    /**
     * @brief Returns true if the vehicle has reached the target altitude set by takeoff()
     */
    bool takeoff_complete() const;

    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     * @param param3    Command parameter 3
     * @param param4    Command parameter 4
     * @param param5    Command parameter 5
     * @param param6    Command parameter 6
     * @param param7    Command parameter 7
     */
    void publish_vehicle_command(uint16_t command,
        float param1 = 0.0f, 
        float param2 = 0.0f,
        float param3 = 0.0f, 
        float param4 = 0.0f,
        float param5 = 0.0f, 
        float param6 = 0.0f,
        float param7 = 0.0f
    );

private:
    int id;

    Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    // Subscriber uses Best Effort QoS to match PX4's publisher — using the default
    // Reliable QoS causes a silent mismatch and no messages are ever received.
    Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;

    // Latest altitude reading from vehicle_local_position (NED, so negative = up)
    atomic<float> current_z_{0.0f};
    atomic<bool> position_received_{false};
    atomic<bool> takeoff_complete_{false};

    /**
     * @brief Callback for vehicle_local_position subscription.
     *        Updates current_z_ with the latest NED z value.
     */
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
};

#endif
