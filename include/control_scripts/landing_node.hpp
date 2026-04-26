/**
 * @brief Landing node
 * @file landing_node.hpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#ifndef LANDING_NODE_HPP_
#define LANDING_NODE_HPP_

#include <chrono>
#include <iostream>
#include <atomic>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std;
using namespace chrono;
using namespace rclcpp;
using namespace px4_msgs::msg;
using namespace chrono_literals;

class Land : public Node
{
public:
    Land();

    /**
     * @brief Switch to AUTO.LAND mode, block until vehicle_status reports
     *        arming_state == ARMING_STATE_STANDBY (i.e. auto-disarmed after touchdown),
     *        then send an explicit disarm command for safety.
     *        The correct PX4 sequence is:
     *        (1) switch mode to AUTO.LAND, (2) monitor arming_state until disarmed.
     * @param timeout_s Maximum seconds to wait before giving up and returning (default 60 s)
     */
    void land(float timeout_s = 60.0f);

    /**
     * @brief Returns true if the vehicle has landed and disarmed successfully
     */
    bool land_complete() const;

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

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
        float param1 = 0.0f, float param2 = 0.0f,
        float param3 = 0.0f, float param4 = 0.0f,
        float param5 = 0.0f, float param6 = 0.0f,
        float param7 = 0.0f);

private:
    Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Subscriber uses Best Effort QoS to match PX4's publisher — using the default
    // Reliable QoS causes a silent mismatch and no messages are ever received.
    Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_;

    /** arming_state values from VehicleStatus:
     *   1 = ARMING_STATE_STANDBY (disarmed, ready)
     *   2 = ARMING_STATE_ARMED
     */
    atomic<uint8_t> arming_state_{0};
    atomic<bool> status_received_{false};
    atomic<bool> land_complete_{false};

    /**
     * @brief Callback for vehicle_status subscription.
     *        Updates arming_state_ with the latest value.
     */
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
};

#endif