/**
 * @brief main script
 * @file root_controller.cpp
 * @addtogroup examples
 * @author Subhodeep Choudhury <subho02.dc@gmail.com>
 */

#include <thread>
#include "control_scripts/takeoff_node.hpp"
#include "control_scripts/offboard_node.hpp"
#include "control_scripts/landing_node.hpp"

int main(int argc, char *argv[])
{
	cout << "Starting root controller node..." << endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	init(argc, argv);

    auto takeoff_node = make_shared<Takeoff>();
    auto offboard_node = make_shared<Offboard>();
    auto landing_node = make_shared<Land>();

    executors::MultiThreadedExecutor executor;
    executor.add_node(takeoff_node);
    executor.add_node(offboard_node);
    executor.add_node(landing_node);

    // Spin both nodes in a background thread so sequential logic runs freely in main
    thread spin_thread([&]() {
        executor.spin();
    });

    // Arm and take off to 5 metres. Blocks until altitude is reached (or timeout).
    // takeoff_node->arm();
    takeoff_node->takeoff(2.5f);

    if (!takeoff_node->takeoff_complete()) {
        RCLCPP_ERROR(takeoff_node->get_logger(), "Takeoff failed or timed out — aborting mission.");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // offboard_node->change_mode_offboard();

    // for (int i = 0; i < 20; i++) {
    //     offboard_node->go_to(i * 0.5f, 0.0f, -5.0f);
    //     this_thread::sleep_for(milliseconds(500));
    // }

    // offboard_node->go_to(10.0f, 0.0f, -5.0f);
    // this_thread::sleep_for(seconds(5));

    landing_node->land();

    if (!landing_node->land_complete()) {
        RCLCPP_WARN(landing_node->get_logger(), "Landing timed out — check vehicle state.");
    }

    shutdown();
    spin_thread.join();
    return 0;
}
