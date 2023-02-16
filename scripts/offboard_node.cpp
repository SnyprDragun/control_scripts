#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#define HOVER_ALTITUDE 2.0f

using namespace std;
using namespace ros;

mavros_msgs::State current_state;
std_msgs::Bool end_mission_state;
geometry_msgs::PoseStamped pose;

class Offboard{
    private:
        NodeHandle nh;
        Subscriber state_sub;
        Publisher local_pos_pub;
        Subscriber end_mission_sub;
        ServiceClient arming_client;
        ServiceClient landing_client;
        ServiceClient set_mode_client;
    
    public:
        Offboard(int id){
            string uav = "uav";
            string uav_id = to_string(id);

            string state_sub_topic = uav + uav_id + "/mavros/state";
            this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Offboard::state_cb, this);

            string local_pos_pub_topic = uav + uav_id + "/mavros/setpoint_position/local";
            this->local_pos_pub = this->nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_topic, 10);

            string end_mission_topic = "/end_mission";
            this->end_mission_sub = this->nh.subscribe(end_mission_topic,10, &Offboard::end_mission_cb, this);

            string arming_client_topic = uav + uav_id + "/mavros/cmd/arming";
            this->arming_client = this->nh.serviceClient<mavros_msgs::CommandBool>(arming_client_topic);

            string landing_client_topic = uav + uav_id + "/mavros/cmd/land";
            this->landing_client = this->nh.serviceClient<mavros_msgs::CommandTOL>(landing_client_topic);

            string set_mode_client_topic = uav + uav_id + "/mavros/set_mode";
            this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);

            Rate rate(20);
        }

        void state_cb(const mavros_msgs::State& msg){
            current_state = msg;
        }

        void end_mission_cb(const std_msgs::Bool& msg){
            end_mission_state = msg;
        }

        void init_connection(){
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

            //send a few setpoints before starting
            for(int i = 100; ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            }
        }

        void init_offboard_arm(){
            Rate rate(20);

            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";

            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

            mavros_msgs::CommandTOL land_cmd;
            land_cmd.request.yaw = 0;
            land_cmd.request.latitude = nan("1");
            land_cmd.request.longitude = nan("1");
            land_cmd.request.altitude = 0;

            Time last_request = Time::now();

            while(ok()){
                if(!end_mission_state.data){
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
                            }
                            last_request = Time::now();
                        }
                    }
                    local_pos_pub.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                }
                
                else{
                    ROS_INFO("Ending mission...");
                    while(!(landing_client.call(land_cmd) && land_cmd.response.success)){
                        ROS_INFO("Attempting to land...");
                        ros::spinOnce();
                        rate.sleep();
                    }
                    ROS_INFO("Landed!");

                    rate.sleep();

                    exit(0);
                }
            }
        }
};

int main(int argc, char **argv){
    init(argc, argv, "offboard_node", ros::init_options::AnonymousName);
    
    int uav_id = 0;
    Offboard offboard = Offboard(uav_id);
    offboard.init_connection();
    offboard.init_offboard_arm();

    spin();
    return 0;
}