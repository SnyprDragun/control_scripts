#include "control_scripts/offboard_node.hpp"

mavros_msgs::State current_state_offboard;

Offboard::Offboard(int id){
    string uav = "uav";
    string uav_id = to_string(id);

    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Offboard::state_cb, this);
    
    string position_pub_topic = "/mavros/setpoint_position/local";
    this->position_pub = this->nh.advertise<geometry_msgs::PoseStamped>(position_pub_topic, 10);
    
    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);
}

void Offboard::state_cb(const mavros_msgs::State& msg){
    current_state_offboard = msg;
}

void Offboard::init_connection(){
    Rate rate(20);
    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state_offboard.connected){
        ROS_INFO("Initializing offboard_node...");
        spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");
}

void Offboard::offboard(float latitude, float longitude, float altitude){
    Rate rate(20.0);

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = latitude;
    pose.pose.position.y = longitude;
    pose.pose.position.z = altitude;

    for(int i = 100; ok() && i > 0; --i){
        position_pub.publish(pose);
        spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    Time last_request = Time::now();

    while(ok()){
        if( current_state_offboard.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Preparing to move...");
            }
            last_request = Time::now();
        }

        position_pub.publish(pose);
        if(pose.pose.position.x = latitude && (Time::now() - last_request > Duration(5.0))){
            ROS_INFO("Setpoint Reached!");
            offb_set_mode.request.custom_mode = "AUTO.LOITER";
            while(ok()){
                if( current_state_offboard.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Standby");
                        break;
                    }
                    last_request = Time::now();
                }
            }
            break;
        }
    
        spinOnce();
        rate.sleep();
    }
}
