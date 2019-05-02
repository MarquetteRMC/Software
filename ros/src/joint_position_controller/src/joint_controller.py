#!/usr/bin/env python

import math
import rospy
import tf

from simple_pid import PID
from scipy import signal

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

        self.pitch_sub = rospy.Subscriber('roboclaw/pitch',JointState,self.pitch_pos_callback)
        self.height_sub = rospy.Subscriber('roboclaw/height',JointState,self.height_pos_callback)
        self.joint_status_pub = rospy.Subscriber('joint_states',JointState,self.joint_states_callback)

        self.rate = rospy.Rate(5)

        self.height_min_length = 15.71
        self.height_max_length = 25.55
        self.height_stroke_length = self.height_max_length - self.height_min_length

        self.pitch_min_length = 11.69
        self.pitch_max_length = 17.6
        self.pitch_stroke_length = self.pitch_max_length - self.pitch_min_length

        self.start = False

        self.h1_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/5)
        self.h2_pid = PID(1,1,1,output_limits=(-127,127),auto_mode=False,sample_time=1/5)
        self.p1_pid = PID(5,0,0,output_limits=(-127,127),auto_mode=False,sample_time=1/5)
        self.p2_pid = PID(5,0,0,output_limits=(-127,127),auto_mode=False,sample_time=1/5)

        self.MAX_ENC = 2047
        self.pe_div = 12
        self.he_div = 12

        self.deadband = 10

        self.h_params = {"m1":50/self.he_div,"m2":50/self.he_div}
        self.p_params = {"m1":30/self.pe_div,"m2":30/self.pe_div}

        self.p1_vals = [0,0,0,0,0]
        self.p2_vals = [0,0,0,0,0]
        self.p1_vals = [0,0,0,0,0]
        self.p2_vals = [0,0,0,0,0]

        self.height_pos = {"m1":0,"m2":0}
        self.pitch_pos = {"m1":0,"m2":0}

    def run(self):
        rospy.loginfo("waiting for data")

        while self.height_pos['m1'] == 0 and self.pitch_pos['m1'] == 0:
            self.rate.sleep()

        rospy.sleep(1)
        rospy.loginfo("got data")
        
        self.h1_pid.set_auto_mode(True,last_output=0.0)
        self.h2_pid.set_auto_mode(True,last_output=0.0)
        self.p1_pid.set_auto_mode(True,last_output=0.0)
        self.p2_pid.set_auto_mode(True,last_output=0.0)

        while not rospy.is_shutdown():
            
            h_output1 = self.h1_pid(self.height_pos['m1'])
            h_output2 = self.h2_pid(self.height_pos['m2'])
            if abs(h_output1) < self.deadband: h_output1 = 0
            if abs(h_output2) < self.deadband: h_output2 = 0

            p_output1 = self.p1_pid(self.pitch_pos['m1'])
            p_output2 = self.p2_pid(self.pitch_pos['m2'])
            if abs(p_output1) < self.deadband: p_output1 = 0
            if abs(p_output2) < self.deadband: p_output2 = 0
            rospy.loginfo("pid output p1 %d, p2 %d", p_output1,p_output2)

            self.publish_pitch_vel(-p_output1,-p_output2)
            self.publish_height_vel(h_output1,h_output2)

            self.publish_arm_state()
            self.rate.sleep()


    def shutdown(self):
        rospy.loginfo("Shutting down joint_controller_node")

    def get_avg(self,lst):
        return sum(lst) / len(lst)

    def pitch_add_meas(self, p1, p2):
        self.p1_vals.insert(0,int(p1/self.pe_div))
        self.p1_vals.pop()
        self.p2_vals.insert(0,int(p2/self.pe_div))
        self.p2_vals.pop()

    def pitch_pos_callback(self, msg):
        if msg.position[0] < 0.1 or msg.position[0] < 0.1:
            return
        self.pitch_add_meas(msg.position[0],msg.position[1])
        self.pitch_pos['m1'] = self.get_avg(self.p1_vals)
        self.pitch_pos['m2'] = self.get_avg(self.p2_vals)
        #rospy.loginfo("pitch m1 %d", self.pitch_pos['m2'])

    def height_pos_callback(self, msg):
        if msg.position[0] < 0.1 or msg.position[0] < 0.1:
            return
        self.height_pos['m1'] = int(msg.position[0]/self.he_div)
        self.height_pos['m2'] = int(msg.position[1]/self.he_div)
        #rospy.loginfo("height m1 %d", self.height_pos['m1'])
    
    def joint_states_callback(self,msg):
        height_angle_target = msg.position[2]
        height_inch_target = self.height_angle_to_dist(height_angle_target)
        height_enc_target = self.height_inch_to_enc(height_inch_target,self.h_params['m1'])
        self.h1_pid.setpoint = int(height_enc_target/self.he_div)
        height_enc_target = self.height_inch_to_enc(height_inch_target,self.h_params['m2'])
        self.h2_pid.setpoint = int(height_enc_target/self.he_div)
        # self.h2_pid.setpoint = height_enc_target

        pitch_angle_target = msg.position[3]
        pitch_inch_target = self.pitch_angle_to_dist(pitch_angle_target)
        pitch_enc_target = self.pitch_inch_to_enc(pitch_inch_target,self.p_params['m1'])
        self.p1_pid.setpoint = int(pitch_enc_target/self.pe_div)
        pitch_enc_target = self.pitch_inch_to_enc(pitch_inch_target,self.p_params['m2'])
        self.p2_pid.setpoint = int(pitch_enc_target/self.pe_div)
        #rospy.loginfo("setpoint h %d, p %d", height_enc_target , pitch_enc_target)

    def publish_arm_state(self):
            arm_msg = JointState()
            arm_msg.header = Header()
            arm_msg.header.stamp = rospy.Time.now()
            arm_msg.name = ['height', 'pitch']
            height_pos_inch = self.height_enc_to_inch(self.height_pos['m1']*self.he_div,self.h_params['m1']*self.he_div)
            pitch_pos_inch = self.pitch_enc_to_inch(self.pitch_pos['m1']*self.pe_div,self.p_params['m1']*self.pe_div)
            height_angle = self.height_dist_to_angle(height_pos_inch)
            pitch_angle = self.pitch_dist_to_angle(pitch_pos_inch)
            arm_msg.position = [height_angle, pitch_angle]
            self.joint_pub.publish(arm_msg)

    def publish_pitch_vel(self,m1,m2):
        vel = Twist()
        vel.linear.x = m1
        vel.linear.y = m2
        self.pitch_pub.publish(vel)

    def publish_height_vel(self,m1,m2):
        vel = Twist()
        vel.linear.x = m1
        vel.linear.y = m2
        self.height_pub.publish(vel)

    def height_enc_to_inch(self,enc,enc_min):
        percent = (enc-enc_min)/(self.MAX_ENC-enc_min)
        return self.height_min_length + self.height_stroke_length * percent

    def height_inch_to_enc(self,inch,enc_min):
        percent = (inch-self.height_min_length)/(self.height_stroke_length)
        return enc_min + (self.MAX_ENC-enc_min) * percent

    def pitch_enc_to_inch(self,enc,enc_min):
        percent = (enc-enc_min)/(self.MAX_ENC-enc_min)
        return self.pitch_min_length + self.pitch_stroke_length * percent

    def pitch_inch_to_enc(self,inch,enc_min):
        percent = (inch-self.pitch_min_length)/(self.pitch_stroke_length)
        return enc_min + (self.MAX_ENC-enc_min) * percent

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
