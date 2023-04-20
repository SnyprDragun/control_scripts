#include "control_scripts/offboard_node.hpp"

#define HOVER_ALTITUDE 1.0f

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;

Offboard::Offboard(int id){
    string uav = "uav";
    string uav_id = to_string(id);

    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Offboard::state_cb, this);

    string local_pos_pub_topic = "/mavros/setpoint_position/local";
    this->local_pos_pub = this->nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_topic, 10);

    string arming_client_topic = "/mavros/cmd/arming";
    this->arming_client = this->nh.serviceClient<mavros_msgs::CommandBool>(arming_client_topic);

    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);
}

void Offboard::state_cb(const mavros_msgs::State& msg){
    current_state = msg;
}

void Offboard::init_connection(){
    Rate rate(20);
    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state.connected){
        ROS_INFO("Connecting to FCT...");
        spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = HOVER_ALTITUDE;

    for(int i = 100; ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        }
}

void Offboard::init_offboard_arm(){
    Rate rate(20);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    Time last_request = Time::now();

    bool flag = true;

    while(flag){
        if(current_state.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = Time::now();
        } 

        else {
            if(!current_state.armed && (Time::now() - last_request > Duration(5.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    flag = false;
                }
                last_request = Time::now();
            }
        }
        local_pos_pub.publish(pose);
        spinOnce();
        rate.sleep();
    }
}
