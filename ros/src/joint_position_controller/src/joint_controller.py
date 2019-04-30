#!/usr/bin/env python

import math
import rospy
import tf

from simple_pid import PID

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class Node:


    def __init__(self):
        rospy.init_node("joint_controller_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Setting up joint_controller_node")

        self.pitch_pub = rospy.Publisher("roboclaw/pitch_vel",Twist)
        self.height_pub = rospy.Publisher("roboclaw/height_vel",Twist)
        self.joint_pub = rospy.Publisher('arm_joint_state',JointState,queue_size=10)
        self.pitch_sub = rospy.Subscriber('roboclaw/pitch_state',JointState,self.pitch_pos_callback)
        self.height_sub = rospy.Subscriber('roboclaw/height_state',JointState,self.height_pos_callback)

        self.rate = rospy.Rate(30)

        self.height_min_length = 15.71
        self.height_max_length = 25.55

        self.pitch_min_length = 11.69
        self.pitch_max_length = 17.6

        self.h1_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)
        self.h2_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)
        self.p1_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)
        self.p2_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)

        self.h_params = {"m1":(78,2047),"m2":(78,2047)}
        self.p_params = {"m1":(78,2047),"m2":(78,2047)}

        self.height_pos = {"m1":0,"m2":0}
        self.pitch_pos = {"m1":0,"m2":0}

        
        
        


    def run(self):
        rospy.loginfo("waiting for data")

        while self.height_pos['m1'] == 0 or self.pitch_pos['m1'] == 0:
            self.rate.sleep()

        rospy.loginfo("got data")

        self.h1_pid.set_auto_mode(True,last_output=0.0)
        #self.h2_pid.set_auto_mode(True,last_output=0.0)
        self.p1_pid.set_auto_mode(True,last_output=0.0)
        #self.p2_pid.set_auto_mode(True,last_output=0.0)

        self.h1_pid.setpoint = 100
        self.p1_pid.setpoint = 100

        while not rospy.is_shutdown():

            h_output = self.h1_pid(self.height_pos['m1'])
            p_output = self.p1_pid(self.pitch_pos['m1'])

            self.publish_pitch_vel(p_output,p_output)
            self.publish_height_vel(h_output,h_output)

            this.publish_arm_state()
            rate.sleep()


    def shutdown(self):
        rospy.loginfo("Shutting down joint_controller_node")

    def pitch_pos_callback(self, msg):
        self.pitch_pos['m1'] = msg.position[0]
        self.pitch_pos['m2'] = msg.position[1]
        rospy.logdebug("pitch m1 %d", self.pitch_pos['m1'])

    def height_pos_callback(self, msg):
        self.height_pos['m1'] = msg.position[0]
        self.height_pos['m2'] = msg.position[1]
        rospy.logdebug("height m1 %d", self.pitch_pos['m1'])

    def publish_arm_state(self):
            arm_msg = JointState()
            arm_msg.header = Header()
            arm_msg.header.stamp = rospy.Time.now()
            arm_msg.name = ['height', 'pitch']
            height_angle = this.height_dist_to_angle(this.height_pos)
            pitch_angle = this.pitch_dist_to_angle(this.pitch_pos)
            arm_msg.position = [height_angle, pitch_angle]
            arm_msg.velocity = []
            arm_msg.effort = []
            self.joint_pub(arm_msg)

    def publish_pitch_vel(self,m1,m2):
        vel = Twist()
        vel.header = Header()
        vel.header.stamp = rospy.Time.now()
        vel.linear.x = m1
        vel.linear.y = m2
        self.pitch_pub(vel)

    def publish_height_vel(self,m1,m2):
        vel = Twist()
        vel.header = Header()
        vel.header.stamp = rospy.Time.now()
        vel.linear.x = m1
        vel.linear.y = m2
        self.height_pub(vel)

    def height_angle_to_dist(self, theta):
        return math.sqrt(793.5625 - 708*math.cos(math.radians(90)-theta))

    def height_dist_to_angle(self, pos): #pos in inches
        return math.radians(90 - math.degrees(math.acos((793.5625-(pos*pos))/708)))

    def pitch_angle_to_dist(self, theta):
        return math.sqrt(((30-11*math.cos(theta))-14.875)**2 + ((11*math.sin(theta)-3)-6.75)**2)

    def pitch_dist_to_angle(self, pos):
        return math.asin(0.05557*pos*math.sin(math.acos((pos*pos - 202.828125)/(22*pos*pos))))+math.radians(32.807)




if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
