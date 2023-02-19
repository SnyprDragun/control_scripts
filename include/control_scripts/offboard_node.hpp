#ifndef OFFBOARD_NODE_HPP
#define OFFBOARD_NODE_HPP

#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

using namespace std;
using namespace ros;

class Offboard{
    private:
        NodeHandle nh;
        Subscriber state_sub;
        Publisher local_pos_pub;
        ServiceClient arming_client;
        ServiceClient landing_client;
        ServiceClient set_mode_client;

    public:
        Offboard(int);
        void state_cb(const mavros_msgs::State&);
        void init_connection();
        void init_offboard_arm();
};

#endif