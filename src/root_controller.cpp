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

    auto offboard_node = make_shared<Offboard>();

    thread spin_thread([&]() {
        spin(offboard_node);
    });

    offboard_node->change_mode_offboard();

    for (int i = 0; i < 20; i++) {
        offboard_node->go_to(i * 0.5f, 0.0f, -5.0f);
        this_thread::sleep_for(milliseconds(500));
    }

    offboard_node->go_to(10.0f, 0.0f, -5.0f);
    this_thread::sleep_for(seconds(5));

    shutdown();
    spin_thread.join();
    return 0;
}
