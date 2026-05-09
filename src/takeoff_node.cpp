/**
 * @brief Takeoff node
 * @file takeoff_node.cpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#include "control_scripts/takeoff_node.hpp"

Takeoff::Takeoff(int uav_id) : Node("takeoff_control_" + to_string(uav_id))
{
    string id = to_string(uav_id);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("px4_" + id+ "/fmu/in/vehicle_command", 10);

    /** Subscribe to vehicle_local_position to monitor altitude during takeoff.
     * PX4 publishes with Best Effort reliability — must match or the QoS handshake
     * fails silently and no messages are ever received (causes infinite freeze in takeoff()).
     */
    QoS qos_best_effort(10);
    qos_best_effort.best_effort();

    vehicle_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("px4_" + id+ "/fmu/out/vehicle_local_position_v1",
        qos_best_effort,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            vehicle_local_position_callback(msg);
        });
}

/**
 * @brief Send a command to Arm the vehicle
 */
void Takeoff::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Takeoff::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Issue VEHICLE_CMD_NAV_TAKEOFF and block until the vehicle reaches the target altitude (within tolerance), as confirmed by vehicle_local_position subscription.
 *        Waits up to timeout_s seconds before returning regardless of altitude.
 * @param altitude  Target takeoff altitude in meters (positive, NED converted internally)
 * @param tolerance Altitude tolerance in meters — how close counts as "reached" (default 0.3 m)
 * @param timeout_s Maximum seconds to wait before giving up and returning (default 30 s)
 */
void Takeoff::takeoff(float altitude, float tolerance, float timeout_s)
{
    takeoff_complete_ = false;

    // Wait until we have at least one position reading before issuing the command
    RCLCPP_INFO(this->get_logger(), "Waiting for position fix...");
    Rate wait_rate(10);
    while (!position_received_) {
        wait_rate.sleep();
    }

    // Step 1: Arm the vehicle
    RCLCPP_INFO(this->get_logger(), "Arming...");
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);

    // Give the arming command time to be processed before switching mode
    Rate arm_wait(10);
    for (int i = 0; i < 10; i++) { arm_wait.sleep(); }

    // Step 2: Switch to AUTO.TAKEOFF mode with retries — PX4 may reject the first
    // attempt if pre-arm checks are still settling after arm.
    // param1=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), param2=4 (AUTO), param3=2 (TAKEOFF)
    RCLCPP_INFO(this->get_logger(), "Switching to AUTO.TAKEOFF mode, targeting %.2f m", altitude);
    Rate retry_rate(2);
    for (int attempt = 0; attempt < 5; attempt++) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,   // param1: base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            4.0f,   // param2: PX4 custom main mode — AUTO
            2.0f    // param3: PX4 custom sub mode — TAKEOFF
        );
        RCLCPP_INFO(this->get_logger(), "Mode switch attempt %d/5", attempt + 1);
        retry_rate.sleep();
    }

    // Step 3: Monitor altitude — block until target is reached or timeout
    // In NED, z is negative upward, so reaching altitude means z <= -(altitude - tolerance)
    const float target_z_ned = -(altitude - tolerance);

    RCLCPP_INFO(this->get_logger(), "Waiting to reach %.2f m (NED z threshold: %.2f)...", altitude, target_z_ned);

    const auto start_time = this->get_clock()->now();
    Rate poll_rate(10);

    while (ok()) {
        const float z = current_z_.load();

        if (z <= target_z_ned) {
            takeoff_complete_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Takeoff complete! Current z: %.2f m (NED), altitude: ~%.2f m",
                z, -z);
            return;
        }

        const double elapsed = (this->get_clock()->now() - start_time).seconds();
        if (elapsed >= timeout_s) {
            RCLCPP_WARN(this->get_logger(),
                "Takeoff timed out after %.1f s. Current z: %.2f m (NED), altitude: ~%.2f m",
                timeout_s, z, -z);
            return;
        }

        poll_rate.sleep();
    }
}

/**
 * @brief Returns true if the vehicle has reached the target altitude set by takeoff()
 */
bool Takeoff::takeoff_complete() const
{
    return takeoff_complete_.load();
}

/**
 * @brief Callback for vehicle_local_position subscription.
 *        Updates current_z_ with the latest NED z value.
 */
void Takeoff::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    current_z_.store(msg->z);
    position_received_ = true;
}

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
void Takeoff::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = this->id;
    msg.target_component = this->id;
    msg.source_system = this->id;
    msg.source_component = this->id;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}
