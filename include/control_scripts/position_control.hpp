#ifndef POSITION_CONTROL_HPP
#define POSITION_CONTROL_HPP

#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandTOL.h>

using namespace std;
using namespace ros;

class Position{
    private:
        NodeHandle nh;
        Subscriber gps_points_sub;
        Publisher local_pos_pub;
        Subscriber end_mission_sub;
        ServiceClient landing_client;
    
    public:
        Position(int);
        void gps_points_cb(const geometry_msgs::PoseStamped&);
        void end_mission_cb(const std_msgs::Bool&);
        void velocity_cmd();
};

#endif