/**
 * @brief Multi-UAV root controller
 */

#include <thread>
#include <vector>

#include "control_scripts/takeoff_node.hpp"
#include "control_scripts/offboard_node.hpp"
#include "control_scripts/landing_node.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    cout << "Starting root controller node..." << endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    init(argc, argv);

    constexpr int NUM_UAVS = 3;

    vector<shared_ptr<Takeoff>> takeoff_nodes;
    vector<shared_ptr<Offboard>> offboard_nodes;
    vector<shared_ptr<Land>> landing_nodes;

    executors::MultiThreadedExecutor executor;

    // ------------------------------------------------------------
    // Create nodes for all UAVs
    // ------------------------------------------------------------
    for (int uav_id = 1; uav_id <= NUM_UAVS; ++uav_id)
    {
        auto takeoff_node = make_shared<Takeoff>(uav_id);
        auto offboard_node = make_shared<Offboard>(uav_id);
        auto landing_node = make_shared<Land>(uav_id);

        takeoff_nodes.push_back(takeoff_node);
        offboard_nodes.push_back(offboard_node);
        landing_nodes.push_back(landing_node);

        executor.add_node(takeoff_node);
        executor.add_node(offboard_node);
        executor.add_node(landing_node);

        cout << "Initialized UAV " << uav_id << endl;
    }

    // ------------------------------------------------------------
    // Start executor thread
    // ------------------------------------------------------------
    thread spin_thread([&]() {
        executor.spin();
    });

    // ------------------------------------------------------------
    // TAKEOFF ALL UAVS
    // ------------------------------------------------------------
    cout << "Starting takeoff..." << endl;

    for (auto &node : takeoff_nodes)
    {
        node->takeoff(2.5f);
    }

    // Wait for all UAVs to complete takeoff
    bool all_takeoff_complete = false;

    while (!all_takeoff_complete)
    {
        all_takeoff_complete = true;

        for (auto &node : takeoff_nodes)
        {
            if (!node->takeoff_complete())
            {
                all_takeoff_complete = false;
                break;
            }
        }

        this_thread::sleep_for(milliseconds(200));
    }

    cout << "All UAVs airborne." << endl;

    // ------------------------------------------------------------
    // OFFBOARD MODE FOR ALL
    // ------------------------------------------------------------
    for (auto &node : offboard_nodes)
    {
        node->change_mode_offboard();
    }

    // ------------------------------------------------------------
    // Example trajectory
    // ------------------------------------------------------------
    for (int i = 0; i < 20; ++i)
    {
        float x = i * 0.5f;

        for (int u = 0; u < NUM_UAVS; ++u)
        {
            // Small Y offset so drones don't collide
            float y = static_cast<float>(u) * 2.0f;

            offboard_nodes[u]->go_to(x, y, -5.0f);
        }

        this_thread::sleep_for(milliseconds(500));
    }

    // ------------------------------------------------------------
    // LAND ALL UAVS
    // ------------------------------------------------------------
    cout << "Landing all UAVs..." << endl;

    for (auto &node : landing_nodes)
    {
        node->land();
    }

    // Wait for all landings
    bool all_landed = false;

    while (!all_landed)
    {
        all_landed = true;

        for (auto &node : landing_nodes)
        {
            if (!node->land_complete())
            {
                all_landed = false;
                break;
            }
        }

        this_thread::sleep_for(milliseconds(200));
    }

    cout << "All UAVs landed." << endl;

    shutdown();
    spin_thread.join();

    return 0;
}
