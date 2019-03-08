#!/usr/bin/env python

import struct
import math
import io
import rospy
from geometry_msgs.msg import Twist


class xboxControllerTeleop():
    def __init__(self):

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        self._hz = rospy.get_param('~hz', 30)

        self._forward_rate = rospy.get_param('~forward_rate', 1.0)
        self._backward_rate = rospy.get_param('~backward_rate', 1.0)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self._speed_number = 325
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0

        #could be js0-3 look at the light on the xbox controller and subtract 1 from where the light is
        self._infile_path = "/dev/input/js0"

        # The joysticks range from -32768 to 32767
        self._left_joy_LR_thresh = 21000
        self._left_joy_UD_thresh = 21000
        self._right_joy_LR_thresh = 21000
        self._right_joy_UD_thresh = 21000

        #current joystick positions
        self._left_joy_vertical = 0
        self._left_joy_horizontal = 0
        self._right_joy_vertical = 0
        self._right_joy_horizontal = 0

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True


        EVENT_SIZE = struct.calcsize("IhBB")
        input_stream = io.open(self._infile_path,"rb")
        event = input_stream.read(EVENT_SIZE)

        while event or self._running:

            (tv_msec,  value, presstype, number) = struct.unpack("IhBB", event)
            if presstype==1:
                self._button_press_handler(value, number)
            if presstype==2:
                self._joystick_handler(value, number)

            self._set_velocity()
            self._publish()
            
            event = input_stream.read(EVENT_SIZE)
            

    def _button_press_handler(self, value, number):
        if number == 0:
            print("you pressed A")
        if number == 1:
            print("you pressed B")
        if number == 2:
            print("you pressed X")
        if number == 3:
            print("you pressed Y")
        if number == 4:
            print("you pressed 4")
        if number == 5:
            print("you pressed 5")
        if number == 6:
            print("you pressed 6")
        if number == 7:
            print("you pressed 7")

    def _joystick_handler(self, value, number):
        if number == 0: #left joystick right left
            if value > self._left_joy_LR_thresh:
                print("left joy right")
            elif value < -self._left_joy_LR_thresh:
                print("left joy left")
            self._left_joy_horizontal = value

        elif number == 1: #left joystick up down
            if value > self._left_joy_UD_thresh:
                print("left joy down")
            elif value < -self._left_joy_UD_thresh:
                print("left joy up")
            self._left_joy_vertical = value

        elif number == 2: #right joystick left right
            if value > self._right_joy_LR_thresh:
                print("right joy right")
            elif value < -self._right_joy_LR_thresh:
                print("right joy left")
            self._right_joy_horizontal = value

        elif number == 3: #right joystick up down
            if value > self._right_joy_UD_thresh:
                print("right joy down")
            elif value < -self._right_joy_UD_thresh:
                print("right joy up")
            self._right_joy_vertical = value

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):

        if self._left_joy_vertical < -self._left_joy_UD_thresh: #forward
            linear = self._speed_number * self._forward_rate
        elif self._left_joy_vertical > self._left_joy_UD_thresh: #backwards
            linear = -self._speed_number * self._forward_rate
        else:
            linear = 0
            
        if self._left_joy_horizontal < -self._left_joy_UD_thresh: #left
            angular = -self._speed_number * self._rotation_rate
        elif self._left_joy_horizontal > self._left_joy_UD_thresh: #right
            angular = self._speed_number * self._rotation_rate
        else:
            angular = 0
        
        self._angular = angular
        self._linear = linear

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)
        self.vel_pub.publish(twist)
    

def main():
    rospy.init_node('xbox_teleop')
    app = xboxControllerTeleop()
    app.run()

if __name__ == '__main__':
    main()
    
