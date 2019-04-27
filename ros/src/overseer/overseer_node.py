#!/usr/bin/env python

import rospy
from std_msgs.msg import String

global curr_state

def command_listener_callback(data):
    global curr_state 
    curr_state = data


def initialize():
    global curr_state
    curr_state = "inactive"
    state_pub = rospy.Publisher('execution_state', String, queue_size=10)
    rospy.init_node('execute_state_publisher')
    rospy.Subscriber("state_commands", String, command_listener_callback)
    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        state_pub.publish(curr_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass





