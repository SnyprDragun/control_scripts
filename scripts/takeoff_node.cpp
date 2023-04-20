#include "control_scripts/takeoff_node.hpp"

mavros_msgs::State current_state_takeoff;
geometry_msgs::PoseStamped pose_takeoff;

Takeoff::Takeoff(int id){
    string uav = "uav";
    string uav_id = to_string(id);

    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Takeoff::state_cb, this);

    string local_pos_pub_topic = "/mavros/setpoint_position/local";
    this->local_pos_pub = this->nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_topic, 10);

    string takeoff_client_topic = "/mavros/cmd/takeoff";
    this->takeoff_client = this->nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_client_topic);

    string arming_client_topic = "/mavros/cmd/arming";
    this->arming_client = this->nh.serviceClient<mavros_msgs::CommandBool>(arming_client_topic);

    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);
}

void Takeoff::state_cb(const mavros_msgs::State& msg){
    current_state_takeoff = msg;
}

void Takeoff::init_connection(){
    Rate rate(20);
    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state_takeoff.connected){
        ROS_INFO("Connecting to FCT...");
        spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    pose_takeoff.pose.position.x = 0;
    pose_takeoff.pose.position.y = 0;
    pose_takeoff.pose.position.z = 0;

    for(int i = 100; ok() && i > 0; --i){
        local_pos_pub.publish(pose_takeoff);
        spinOnce();
        rate.sleep();
        }
}

void Takeoff::arm(){
    Rate rate(20);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    Time last_request = Time::now();

    bool flag = true;
    while(flag){
        if(!current_state_takeoff.armed && (Time::now() - last_request > Duration(5.0))){
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
                flag = false;
            }
            last_request = Time::now();
        }
        spinOnce();
        rate.sleep();
    }
}

void Takeoff::takeoff(float altitude){
    Rate rate(20);
    
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.latitude = nan("1");
    takeoff_cmd.request.longitude = nan("1");
    takeoff_cmd.request.altitude = altitude;

    Time last_request = Time::now();

    bool flag = true;

    while(flag){
        if(current_state_takeoff.mode != "AUTO.TAKEOFF" && (Time::now() - last_request > Duration(5.0))){
            if(takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
                ROS_INFO("Taking Off");
                flag = false;
            }
            last_request = Time::now();
        } 
        spinOnce();
        rate.sleep();
    }
}