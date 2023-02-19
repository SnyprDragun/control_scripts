#include "control_scripts/offboard_node.hpp"
#include "control_scripts/position_control.hpp"

int main(int argc, char **argv){
    init(argc, argv, "root_controller", ros::init_options::AnonymousName);
    
    int uav_id = 0;

    Offboard offboard(uav_id);
    offboard.init_connection();
    offboard.init_offboard_arm();

    Position position(uav_id);
    position.velocity_cmd();

    spin();
    return 0;
}