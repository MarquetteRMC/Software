#!/usr/bin/env python

import math
import rospy
import tf

from simple_pid import PID

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class Node:


    def __init__(self):
        rospy.init_node("joint_controller_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Setting up joint_controller_node")

        self.pitch_pub = rospy.Publisher("roboclaw/pitch_vel",Twist,queue_size=10)
        self.height_pub = rospy.Publisher("roboclaw/height_vel",Twist,queue_size=10)
        self.joint_pub = rospy.Publisher('arm_joint_state',JointState,queue_size=10)

        self.pitch_sub = rospy.Subscriber('roboclaw/pitch_state',JointState,self.pitch_pos_callback)
        self.height_sub = rospy.Subscriber('roboclaw/height_state',JointState,self.height_pos_callback)
        self.joint_status_pub = rospy.Suscriber('joint_states',JointState,self.joint_states_callback)

        self.rate = rospy.Rate(5)

        self.height_min_length = 15.71
        self.height_max_length = 25.55

        self.pitch_min_length = 11.69
        self.pitch_max_length = 17.6

        self.h1_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)
        self.h2_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)
        self.p1_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)
        self.p2_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/30)

        self.h_params = {"m1":78,"m2":78}
        self.p_params = {"m1":78,"m2":78}

        self.height_pos = {"m1":0,"m2":0}
        self.pitch_pos = {"m1":0,"m2":0}

    def run(self):
        rospy.loginfo("waiting for data")

        while self.height_pos['m1'] == 0 and self.pitch_pos['m1'] == 0:
            self.rate.sleep()

        rospy.loginfo("got data")

        self.h1_pid.set_auto_mode(True,last_output=0.0)
        #self.h2_pid.set_auto_mode(True,last_output=0.0)
        self.p1_pid.set_auto_mode(True,last_output=0.0)
        #self.p2_pid.set_auto_mode(True,last_output=0.0)

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
        rospy.loginfo("pitch m1 %d", self.pitch_pos['m1'])

    def height_pos_callback(self, msg):
        self.height_pos['m1'] = msg.position[0]
        self.height_pos['m2'] = msg.position[1]
        rospy.loginfo("height m1 %d", self.pitch_pos['m1'])
    
    def joint_states_callback(self,msg):
        print(msg)
        height_angle_target = msg.position[2]
        height_inch_target = self.height_angle_to_dist(height_angle_target)
        height_enc_target = self.height_inch_to_enc(height_inch_target,self.h_params['m1'])
        self.h1_pid.setpoint = height_enc_target
        # height_enc_target = self.height_inch_to_enc(height_inch_target,self.h_params['m2'])
        # self.h2_pid.setpoint = height_enc_target

        pitch_angle_target = msg.position[3]
        pitch_inch_target = self.pitch_angle_to_dist(pitch_angle_target)
        pitch_enc_target = self.pitch_inch_to_enc(pitch_inch_target,self.p_params['m1'])
        self.p1_pid.setpoint = pitch_enc_target
        # pitch_enc_target = self.pitch_inch_to_enc(pitch_inch_target,self.p_params['m2'])
        # self.p2_pid.setpoint = pitch_enc_target
        


    def publish_arm_state(self):
            arm_msg = JointState()
            arm_msg.header = Header()
            arm_msg.header.stamp = rospy.Time.now()
            arm_msg.name = ['height', 'pitch']
            height_pos_inch = self.height_enc_to_inch(self.height_pos)
            pitch_pos_inch = self.pitch_enc_to_inch(self.pitch_pos)
            height_angle = this.height_dist_to_angle(height_pos_inch)
            pitch_angle = this.pitch_dist_to_angle(pitch_pos_inch)
            arm_msg.position = [height_angle, pitch_angle]
            self.joint_pub(arm_msg)

    def publish_pitch_vel(self,m1,m2):
        vel = Twist()
        vel.linear.x = m1
        vel.linear.y = m2
        self.pitch_pub(vel)

    def publish_height_vel(self,m1,m2):
        vel = Twist()
        vel.linear.x = m1
        vel.linear.y = m2
        self.height_pub(vel)

    def height_enc_to_inch(self,enc,start_val):
        total_length = self.height_max_length - self.height_min_length
        percent = (enc-start_val)/(2047-start_val)
        return self.height_min_length + total_length * percent

    def height_inch_to_enc(self,inch,start_val)
        total_length = self.height_max_length - self.height_min_length
        percent = (inch-self.height_min_length)/(total_length)
        return start_val + 2047 * percent

    def pitch_enc_to_inch(self,enc,start_val):
        total_length = self.pitch_max_length - self.pitch_min_length
        percent = (enc-start_val)/(2047-start_val)
        return self.pitch_min_length + total_length * percent

    def pitch_inch_to_enc(self,inch,start_val)
        total_length = self.pitch_max_length - self.pitch_min_length
        percent = (inch-self.pitch_min_length)/(total_length)
        return start_val + 2047 * percent

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
