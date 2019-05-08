#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import tf.transformations
import tf_conversions
import tf2_ros

from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import std_srvs.srv

import time
import math

from odrive_interface import ODriveInterfaceSerial, ODriveInterfaceAPI

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
    
    #use_index = False (bool)
    # offset_float = 0.590887010098 (float)
    # calib_range = 0.019999999553 (float)
    # mode = 0 (int)
    # offset = 1809 (int)
    # cpr = 4096 (int)
    # idx_search_speed = 10.0 (float)
    # pre_calibrated = False (bool)

    #m_s_to_rpm = 60.0/tyre_circumference
    #m_s_to_erpm = 10 * m_s_to_rpm 

# 4096 counts / rev, so 4096 == 1 rev/s


# 1 m/s = 3.6 km/hr

class ODriveNode(object):

    last_speed = 0.0
    driver = None
    last_cmd_vel_time = None
    
    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = .6477
    tyre_circumference = .5282438
    encoder_counts_per_rev = 24
    m_s_to_value = 45.433 #change this back to 0. Changed for testing purposes  
    axis_for_right = 0

    # Startup parameters
    connect_on_startup = True
    calibrate_on_startup = True
    engage_on_startup = True
    
    def __init__(self): 

        self.axis_for_right = float(rospy.get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        self.wheel_track = float(rospy.get_param('~wheel_track', 0.6477)) # m, distance between wheel centres
        self.tyre_circumference = float(rospy.get_param('~tyre_circumference', 0.5282438)) # used to translate velocity commands in m/s into motor rpm
        
        self.connect_on_startup   = rospy.get_param('~connect_on_startup', True)
        self.calibrate_on_startup = rospy.get_param('~calibrate_on_startup', True)
        self.engage_on_startup    = rospy.get_param('~engage_on_startup', True)
        self.has_preroll     = rospy.get_param('~use_preroll', False)
        
        self.max_speed   = rospy.get_param('~max_speed', 176)
        self.max_angular = rospy.get_param('~max_angular', 270)
        
        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
    
        rospy.Service('calibrate_motors',  std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',     std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',    std_srvs.srv.Trigger, self.release_motor)

        self.vel_subscribe = rospy.Subscriber("/digging/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_check) # stop motors if no cmd_vel received > 1second

        if not self.connect_on_startup:
            rospy.loginfo("ODrive node started, but not connected.")
            return
        
        if not self.connect_driver(None)[0]:
            return # Failed to connect
        
        if not self.calibrate_on_startup:
            rospy.loginfo("ODrive node started and connected. Not calibrated.")
            return
        
        if not self.calibrate_motor(None)[0]:
            return
            
        if not self.engage_on_startup:
            rospy.loginfo("ODrive connected and configured. Engage to drive.")
            return
        
        if not self.engage_motor(None)[0]:
            return
        
        rospy.loginfo("ODrive connected and configured. Ready to drive.")
        
    def terminate(self):
        if self.driver:
            self.driver.release()
        self.timer.shutdown()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            rospy.logerr("Already connected.")
            return (False, "Already connected.")
        
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right):
            self.driver = False
            rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        rospy.loginfo("ODrive connected.")
        
        self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference
        
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.disconnect():
            return (False, "Failed disconnection, but try reconnecting.")
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            if not self.driver.preroll():
                return (False, "Failed preroll.")        
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
                    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
    
    def reset_odometry(self, request):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        return(True, "Odometry reset.")
    
    # Helpers and callbacks
    
    def constrain(self, val):
        return min(self.max_speed, max(-self.max_speed, val))
    
    def cmd_vel_callback(self, twist):
        
        digging = self.constrain(twist.linear.x)
        dumping = self.constrain(twist.linear.y)
            
        dig_linear_val  = int(digging * self.m_s_to_value)
        dump_linear_val = int(dumping * self.m_s_to_value)

        if self.last_speed < 0.001 and abs(dig_linear_val) < 0.001 and abs(dump_linear_val) < 0.001:
            return
        
        self.driver.drive(dig_linear_val, dump_linear_val)

        self.last_speed = max(abs(dig_linear_val), abs(dump_linear_val))
        self.last_cmd_vel_time = rospy.Time.now()
                
    def timer_current(self, event):
        if not self.driver or not hasattr(self.driver, 'driver') or not hasattr(self.driver.driver, 'axis0'):
            return
        
        self.left_current_accumulator += self.driver.left_axis.motor.current_control.Ibus
        self.right_current_accumulator += self.driver.right_axis.motor.current_control.Ibus
        
        self.current_loop_count += 1
        if self.current_loop_count >= 9:
            self.current_publisher_left.publish(self.left_current_accumulator)
            self.current_publisher_right.publish(self.right_current_accumulator)
            
            self.current_loop_count = 0
            self.left_current_accumulator = 0.0
            self.right_current_accumulator = 0.0

    def timer_check(self, event):
        """Check for cmd_vel 1 sec timeout. """
        if not self.driver:
            return
        
        if self.last_cmd_vel_time is None:
            return
        
        if (event.current_real-self.last_cmd_vel_time).to_sec() > 1.0 and (self.last_speed > 0):
            self.driver.drive(0,0)
            self.last_speed = 0
            self.last_cmd_vel_time = event.current_real
            

def start_odrive():
    rospy.init_node('odrive_digging_dumping')
    odrive_node = ODriveNode()
    rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass
