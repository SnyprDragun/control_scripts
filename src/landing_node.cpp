/**
 * @brief Landing node
 * @file landing_node.cpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#include "control_scripts/landing_node.hpp"

Land::Land() : Node("landing_control")
{
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    // Subscribe to vehicle_status to monitor arming_state during landing.
    // PX4 publishes with Best Effort reliability — must match or the QoS handshake
    // fails silently and no messages are ever received (causes infinite freeze in land()).
    rclcpp::QoS qos_best_effort(10);
    qos_best_effort.best_effort();

    vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        qos_best_effort,
        [this](const VehicleStatus::SharedPtr msg) {
            vehicle_status_callback(msg);
        });
}

/**
 * @brief Switch to AUTO.LAND mode, block until vehicle_status reports
 *        arming_state == ARMING_STATE_STANDBY (i.e. auto-disarmed after touchdown),
 *        then send an explicit disarm command for safety.
 * @param timeout_s Maximum seconds to wait before giving up and returning (default 60 s)
 */
void Land::land(float timeout_s)
{
    land_complete_ = false;

    // Wait until we have at least one vehicle_status reading before issuing the command
    RCLCPP_INFO(this->get_logger(), "Waiting for vehicle status...");
    rclcpp::Rate wait_rate(10);
    while (!status_received_) {
        wait_rate.sleep();
    }

    // Switch to AUTO.LAND mode with retries — mirrors the takeoff pattern.
    // param1=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), param2=4 (AUTO), param3=6 (LAND sub-mode)
    RCLCPP_INFO(this->get_logger(), "Switching to AUTO.LAND mode...");
    rclcpp::Rate retry_rate(2);
    for (int attempt = 0; attempt < 5; attempt++) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,   // param1: base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            4.0f,   // param2: PX4 custom main mode — AUTO
            6.0f    // param3: PX4 custom sub mode — LAND
        );
        RCLCPP_INFO(this->get_logger(), "Mode switch attempt %d/5", attempt + 1);
        retry_rate.sleep();
    }

    // Block until PX4 auto-disarms after touchdown (arming_state == ARMING_STATE_STANDBY = 1)
    // or until timeout_s is exceeded.
    RCLCPP_INFO(this->get_logger(), "Waiting for touchdown and auto-disarm...");
    const auto start_time = this->get_clock()->now();
    rclcpp::Rate poll_rate(10);

    while (rclcpp::ok()) {
        // arming_state 1 = ARMING_STATE_STANDBY — PX4 auto-disarms on landing
        if (arming_state_.load() == 1) {
            RCLCPP_INFO(this->get_logger(), "Vehicle auto-disarmed — touchdown confirmed.");

            // Explicit disarm for safety in case auto-disarm didn't fully complete
            disarm();
            land_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "Landing complete!");
            return;
        }

        const double elapsed = (this->get_clock()->now() - start_time).seconds();
        if (elapsed >= timeout_s) {
            RCLCPP_WARN(this->get_logger(),
                "Landing timed out after %.1f s. arming_state: %d — forcing disarm.",
                timeout_s, arming_state_.load());
            disarm();
            return;
        }

        poll_rate.sleep();
    }
}

/**
 * @brief Returns true if the vehicle has landed and disarmed successfully
 */
bool Land::land_complete() const
{
    return land_complete_.load();
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Land::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Callback for vehicle_status subscription.
 *        Updates arming_state_ with the latest value.
 */
void Land::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    arming_state_.store(msg->arming_state);
    status_received_ = true;
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
void Land::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
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
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}
