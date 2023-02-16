#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

current_state = State()
land_state = Bool()

class Offboard():
    def __init__(self, uav_no):
        state_pub_topic = "/uav" + str(uav_no) + "/mavros/state"
        self.state_sub = rospy.Subscriber(state_pub_topic, State, callback = self.state_cb)
        local_pos_pub_topic = "/uav" + str(uav_no) + "mavros/setpoint_position/local"
        self.local_pos_pub = rospy.Publisher(local_pos_pub_topic, PoseStamped, queue_size=10)
        land_topic = "/land_cmd"
        self.land_sub = rospy.Subscriber(land_topic, Bool, callback = self.land_cb)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/cmd/land")
        self.landing_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)     
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.rate = rospy.Rate(20)

    def state_cb(self, msg):
        global current_state
        current_state = msg

    def land_cb(self, msg):
        global land_state
        land_state = msg
    
    def init_connection(self):
        while(not rospy.is_shutdown() and not current_state.connected):
            self.rate.sleep()

        self.pose = PoseStamped()

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

    def init_offboard_arm(self):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):
            global land_state
            if not land_state.data:
                if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                        rospy.loginfo("OFFBOARD enabled")
                    
                    last_req = rospy.Time.now()
                else:
                    if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                        if(self.arming_client.call(arm_cmd).success == True):
                            rospy.loginfo("Vehicle armed")
                    
                        last_req = rospy.Time.now()

                self.local_pos_pub.publish(self.pose)
                self.rate.sleep()
                
            else:
                land_cmd = CommandTOLRequest()
                land_cmd.yaw = 0
                land_cmd.altitude = 0
                land_cmd.latitude = math.nan
                land_cmd.longitude = math.nan

                self.landing_client.call(land_cmd)

                rospy.loginfo("Landed!")

                self.rate.sleep()
                quit()

if __name__ == "__main__":
    rospy.init_node('offboard_node',anonymous=True)

    offboard = Offboard(0)
    offboard.init_connection()
    offboard.init_offboard_arm()