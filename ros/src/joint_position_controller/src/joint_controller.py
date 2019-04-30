import math
import rospy
import tf

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class Node:

    self.height_min_length = 15.71
    self.height_max_length = 25.55

    self.pitch_min_length = 11.69
    self.pitch_max_length = 17.6

    def __init__(self):
        rospy.init_node("joint_controller_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Setting up joint_controller_node")

    def run(self):
        rospy.loginfo("running")

    def shutdown(self):
        rospy.loginfo("Shutting down joint_controller_node")

    def height_angle_to_dist(self, theta):
        return math.sqrt(793.5625 - 708*math.cos(math.radians(90)-theta))

    def height_dist_to_angle(self, pos): #pos in inches
        return math.radians(90 - math.degrees(math.acos((793.5625-(pos*pos))/708)))

    def pitch_angle_to_dist(self, theta):
        return 



    def pitch_dist_to_angle(self, pos):
        pos
        return math.asin(0.05557*pos*math.sin(math.acos((pos*pos - 202.828125)/(22*pos*pos))))+math.radians(32.807)




if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
