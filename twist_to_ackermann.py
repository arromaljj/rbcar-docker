#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def twist_callback(twist_msg):
    ackermann_msg = AckermannDriveStamped()
    ackermann_msg.header.stamp = rospy.Time.now()
    ackermann_msg.drive.speed = twist_msg.linear.x
    ackermann_msg.drive.steering_angle = twist_msg.angular.z
    ackermann_pub.publish(ackermann_msg)

if __name__ == '__main__':
    rospy.init_node('twist_to_ackermann')
    twist_sub = rospy.Subscriber('/robot/cmd_vel', Twist, twist_callback)
    ackermann_pub = rospy.Publisher('/robot/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    rospy.spin()