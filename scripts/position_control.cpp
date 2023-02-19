#include "control_scripts/position_control.hpp"

geometry_msgs::PoseStamped gps_points_msg;
std_msgs::Bool end_mission_state;

Position::Position(int id){
    string uav = "uav";
    string uav_id = to_string(id);

    string gps_points_sub_topic = uav + uav_id + "/gps_points";
    this->gps_points_sub = this->nh.subscribe(gps_points_sub_topic, 10, &Position::gps_points_cb, this);

    string local_pos_pub_topic = uav + uav_id + "/mavros/setpoint_position/local";
    this->local_pos_pub = this->nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_topic, 10);

    string end_mission_sub_topic = "/end_mission";
    this->end_mission_sub = this->nh.subscribe(end_mission_sub_topic,10, &Position::end_mission_cb, this);

    string landing_client_topic = uav + uav_id + "/mavros/cmd/land";
    this->landing_client = this->nh.serviceClient<mavros_msgs::CommandTOL>(landing_client_topic);
}

void Position::gps_points_cb(const geometry_msgs::PoseStamped& msg){
    gps_points_msg = msg;
    ROS_INFO("------GOING TO------");
    ROS_INFO("x = %f", gps_points_msg.pose.position.x);
    ROS_INFO("y = %f", gps_points_msg.pose.position.y);
    ROS_INFO("z = %f", gps_points_msg.pose.position.z);
}


void Position::end_mission_cb(const std_msgs::Bool& msg){
    end_mission_state = msg;
}

void Position::velocity_cmd(){
    Rate rate(20);
    gps_points_msg.pose.position.x = 0;
    gps_points_msg.pose.position.y = 0;
    gps_points_msg.pose.position.z = 2;

    while(!end_mission_state.data){
        local_pos_pub.publish(gps_points_msg);
        spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = nan("1");
    land_cmd.request.longitude = nan("1");
    land_cmd.request.altitude = 0;

    if(end_mission_state.data){
        ROS_INFO("--------------------");
        ROS_INFO("Ending mission...");
        while(!(landing_client.call(land_cmd) && land_cmd.response.success)){
            ROS_INFO("Attempting to land...");
            spinOnce();
            rate.sleep();
        }
        ROS_INFO("Landed!");

        rate.sleep();

        exit(0);
    }
}