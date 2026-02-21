/**
 * @brief Offboard control example
 * @file offboard_node.cpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#include "control_scripts/offboard_node.hpp"

Offboard::Offboard() : Node("offboard_control")
{
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    /** Keepalive timer — streams the current target at 10 Hz to maintain offboard mode.
     * PX4 will fall back out of offboard mode if setpoints stop for ~0.5 seconds,
     * so this timer continuously re-publishes whatever go_to() last set as the target.
     */
	keepalive_timer_ = this->create_wall_timer(
        100ms,
        [this]() -> void {
            publish_offboard_control_mode();
            if (offboard_active_) {
                lock_guard<mutex> lock(target_mutex_);
                setpoint(target_x_, target_y_, target_z_, target_yaw_);
            }
        });
}

/**
 * @brief Send a command to Arm the vehicle
 */
void Offboard::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Offboard::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Prime the setpoint stream, switch to offboard mode, and arm the vehicle.
 *        PX4 requires setpoints to already be streaming before it will accept the
 *        mode switch command, so this function publishes ~10 setpoints at the
 *        current target position before sending the mode change and arm commands.
 *        Blocks for approximately 1 second.
 */
void Offboard::change_mode_offboard()
{
    RCLCPP_INFO(this->get_logger(), "Priming setpoint stream before offboard switch...");
    Rate rate(10);
    for (int i = 0; i < 10; i++) {
        publish_offboard_control_mode();
        {
            lock_guard<mutex> lock(target_mutex_);
            setpoint(target_x_, target_y_, target_z_, target_yaw_);
        }
        rate.sleep();
    }
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    // arm();
    offboard_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Offboard mode activated");
}

/**
 * @brief Update the target position that the keepalive timer streams to the FMU.
 *        Thread-safe — can be called from the main thread while the timer callback
 *        runs on the executor thread.
 * @param x     Target x position (NED, meters)
 * @param y     Target y position (NED, meters)
 * @param z     Target z position (NED, meters — negative = up)
 * @param yaw   Target yaw angle in radians [-PI:PI]
 */
void Offboard::go_to(float x, float y, float z, float yaw)
{
    lock_guard<mutex> lock(target_mutex_);
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
    target_yaw_ = yaw;
    RCLCPP_INFO(this->get_logger(), "go_to: [%.2f, %.2f, %.2f] yaw=%.2f", x, y, z, yaw);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void Offboard::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void Offboard::setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Offboard::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}
