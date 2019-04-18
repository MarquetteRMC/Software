
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
import csv

f = open('imu_data.csv', mode='w')
writer = csv.writer(f,delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
writer.writerow(['date','x','y','z','w'])

lastsec = 0

def callback(d):
    global writer
    global lastsec
    if lastsec < d.header.stamp.secs:
        thisquat = [d.header.stamp.secs, d.orientation.x, d.orientation.y, d.orientation.z, d.orientation.w]
        writer.writerow(thisquat)
        print(thisquat)
        lastsec = d.header.stamp.secs

def listener():
    rospy.init_node('imutest', anonymous=True)
    rospy.Subscriber("imu/data", sensor_msgs.msg.Imu, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()






