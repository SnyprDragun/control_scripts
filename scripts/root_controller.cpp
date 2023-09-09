#include "control_scripts/takeoff_node.hpp"
#include "control_scripts/offboard_node.hpp"
#include "control_scripts/landing_node.hpp"

mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;

int main(int argc, char **argv){
    init(argc, argv, "root_controller", init_options::AnonymousName);
    
    ROS_INFO("Initializing Root Controller...");

    int uav_id = 0;
    float altitude = 5.0;

    Takeoff takeoff(uav_id);
    takeoff.init_connection();
    takeoff.arm();
    takeoff.takeoff(altitude);

    Time last_request = Time::now();

    bool flag = true;
    while (flag){
        if (current_state.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
            ROS_INFO("Waiting to complete takeoff...");
        }
        else{
            ROS_INFO("Takeoff Completed Successfully!");
            Duration(10).sleep();

            Offboard offboard(uav_id);
            offboard.init_connection();
            offboard.offboard(3,3,3);
            offboard.offboard(-3,-3,3);
            offboard.offboard(1,1,1);

            // Duration(10).sleep();
            // Land land(uav_id);
            // land.init_connection();
            // land.land();

            flag = false;
        }
    }
    Duration(10).sleep();
    Land land(uav_id);
    land.init_connection();
    land.land();

    spin();
    return 0;
}
