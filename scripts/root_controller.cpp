#include "control_scripts/offboard_node.hpp"
#include "control_scripts/position_control.hpp"
#include "control_scripts/takeoff_node.hpp"

int main(int argc, char **argv){
    init(argc, argv, "root_controller", init_options::AnonymousName);
    
    int uav_id = 0;
    float altitude = 1.0;

    Takeoff takeoff(uav_id);
    takeoff.init_connection();
    takeoff.arm();
    takeoff.takeoff(altitude);

    spin();
    return 0;
}