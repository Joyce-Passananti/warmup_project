#!/usr/bin/env python3

# TOPICS
#   cmd_vel: publish to, used for setting robot velocity.
#   clock: Type: rosgraph_msgs/Clock (time clock)
#   

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class DriveInSquare(object):

    def __init__(self):
        rospy.init_node("drive_in_square")

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        lin = Vector3(x=0.1,y=0,z=0)
        ang = Vector3(x=0,y=0,z=0)
        self.twist = Twist(linear=lin,angular=ang)

    def run(self):
        r = rospy.Rate(1)
        timer = 0
        while not rospy.is_shutdown():
            timer = timer + 1
            if timer%10 == 0:
                self.twist.angular.z=1.571
            else:
                self.twist.angular.z=0
            self.twist_pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = DriveInSquare()
    node.run()
