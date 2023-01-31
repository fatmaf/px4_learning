#! /usr/bin/env python

# a mix of the mavros tutorials and the mavros integration test files

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import String

import math


class MavrosOffboardPositions(object):

    def __init__(self):
        self.point_reached_time = None
        self.current_state = State()
        self.local_position = PoseStamped()
        self.pos = PoseStamped()
        rospy.init_node("offb_node_py")
        self.state_sub = rospy.Subscriber("mavros/state",State,callback=self.state_cb)

        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped,callback=self.local_pos_cb)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",PoseStamped,queue_size=10)

        self.take_off_pub = rospy.Publisher("uav/takeoff",String,queue_size=10)
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming",CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode",SetMode)

        self.rate = rospy.Rate(20)
        self.last_req = None
        self.offboard = False
        self.armed = False
        self.radius = 1
        self.waypoints =[]
        self.current_wp = None
        self.last_print = None
        self.takeoff = False



    def create_wp_list(self):
        self.waypoints= [self.create_pose(0.2),self.create_pose(0.5),self.create_pose(0.7),self.create_pose(1.0),self.create_pose(1.0,1.0,1.0)]


    def wait_for_connection(self):
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

    def create_pose(self,z_val,x_val=0, y_val= 0):
        pose = PoseStamped()

        pose.pose.position.x = x_val
        pose.pose.position.y = y_val
        pose.pose.position.z = z_val

        return pose

    def stream_some_setpoints(self):



        for i in range(100):
            if (rospy.is_shutdown()):
                break

            self.pub_dummy_setpoint()
            self.rate.sleep()

    def pub_dummy_setpoint(self):
        pose = self.create_pose(0.1)
        self.local_pos_pub.publish(pose)

    def get_offboard_msg_req(self):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode= "OFFBOARD"
        return offb_set_mode

    def set_last_req_time(self):
        self.last_req = rospy.Time.now()

    def service_call_spaced(self):
        return (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)

    def switch_to_offboard(self):
        offb_set_mode = self.get_offboard_msg_req()
        if(self.current_state.mode != "OFFBOARD" and self.service_call_spaced()):
            if (self.set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
                self.offboard = True
                return True
            self.set_last_req_time()

        return False

    def arm_to_fly(self):
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        # self.set_last_req_time()

        if(not self.current_state.armed):
            if (self.service_call_spaced()):
                if (self.arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Ready to fly")
                    self.armed = True
                    return True
                self.set_last_req_time()
            else:
                rospy.loginfo("not enough time passed")
        else:
            rospy.loginfo("Already armed ")

        return False


    # def go_to_locations(self):

    def print_pose(self,p1):
        self.rospyloginfo(
                "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
                format(p1.position.x, p1.position.y, p1.position.z, self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       self.local_position.pose.position.z))

    def rospyloginfo(self,tolog):
        if(rospy.Time.now() - self.last_print) > rospy.Duration(10.0):
            rospy.loginfo(tolog)
            self.last_print = rospy.Time.now()

    def do_distance(self,p1,p2):
        dist = (p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2 + (p1.position.z-p2.position.z)**2;
        dist = math.sqrt(dist)
        return dist

    def stay_in_position(self,mode):
        if mode == "start":
            self.point_reached_time = rospy.Time.now()
            return True
        elif mode == "check":
            if (rospy.Time.now() - self.point_reached_time) > rospy.Duration(10):
                return False
            else:
                return True

    def reached_position(self):
        if(self.current_wp is None):
            self.current_wp = self.waypoints.pop(0)
            self.stay_in_position("start")

        pose = self.current_wp
        if(abs(self.do_distance(pose.pose,self.local_position.pose)) <= 0.1):
            self.rospyloginfo("Reached position")
            if(not self.stay_in_position("check")):
                if(len(self.waypoints) > 0):
                    self.stay_in_position("start")
                    self.current_wp = self.waypoints.pop(0)
                    pose = self.current_wp
                    rospy.loginfo("Moving to new position - wp remaining {0}".format(len(self.waypoints)))
                    # rospy.loginfo(pose)
                else:
                    self.rospyloginfo("All done staying here")
            else:
                self.rospyloginfo("Staying in place")

        self.local_pos_pub.publish(pose)
        if self.last_print is None:
            self.last_print = rospy.Time.now()
        self.print_pose(pose.pose)


    def go_to_places(self):
        while (not rospy.is_shutdown()):
            if not self.offboard:
                # rospy.loginfo("Attempting OFFBOARD mode")
                self.switch_to_offboard()
                self.pub_dummy_setpoint()
            elif not self.armed:
                rospy.loginfo("Attempting to fly")
                self.arm_to_fly()
                self.pub_dummy_setpoint()
            else:
                if not self.takeoff:
                    self.takeoff = True
                    self.take_off_pub.publish("True")
                else:
                    self.take_off_pub.publish("False")

                self.reached_position()



            self.rate.sleep()

    def run(self):
        self.take_off_pub.publish("False")
        self.wait_for_connection()
        self.stream_some_setpoints()
        self.set_last_req_time()
        self.create_wp_list()
        self.go_to_places()



    def state_cb(self,msg):
        self.current_state = msg

    def local_pos_cb(self,msg):
        self.local_position = msg




if __name__=="__main__":
    #do this
    test = MavrosOffboardPositions()
    rospy.loginfo("Running good locations")
    input("Press any key to start")
    test.run()
