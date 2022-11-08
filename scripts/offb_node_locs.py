#! /usr/bin/env python

# a mix of the mavros tutorials and the mavros integration test files

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
import math
from tf.transformations import quaternion_from_euler
from six.moves import xrange

class MavrosOffboardPositions(object):

    def __init__(self):
        self.current_state = State()
        self.local_position = PoseStamped()
        self.pos = PoseStamped()
        rospy.init_node("offb_node_py")
        self.state_sub = rospy.Subscriber("mavros/state",State,callback=self.state_cb)

        self.local_pos_pub = rospy.Subscriber("mavros/local_position/pose", PoseStamped,)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",PoseStamped,queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming",CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode",SetMode)

        self.rate = rospy.Rate(20)
        self.last_req = None
        self.offboard = False
        self.armed = False


    def wait_for_connection(self):
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

    def create_pose(self,z_val):
        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
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

        self.set_last_req_time()

        if(not self.current_state.armed and (self.service_call_spaced())):
            if (self.arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Ready to fly")
                self.armed = False
                return True
            self.set_last_req_time()

        return False


    def go_to_places(self):
        while (not rospy.is_shutdown()):
            if(self.switch_to_offboard()):
                if(self.arm_to_fly()):
                    rospy.loginfo("Attempting to reach position 2")
                    self.reach_position(0,0,2.0,30)


            self.pub_dummy_setpoint()
            self.rate.sleep()

    def run(self):
        self.wait_for_connection()
        self.stream_some_setpoints()
        self.go_to_places()



    def state_cb(self,msg):
        self.current_state = msg

    def local_pos_cb(self,msg):
        self.local_position = msg


    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

if __name__=="__main__":
    #do this