#!/usr/bin/env python3

import rospy, math
 
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to the wall
distance = 0.5

# This node follows a wall
class WallFollower(object):

    def __init__(self):
        # Start rospy node
        rospy.init_node("follow_the_wall")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist()

    def process_scan(self, data):
        # Determine closeness to wall by looking at scan data from in front of
        #   the robot, set linear/angular velocity based on that information,
        #   publish to cmd_vel.

        # Find the shortest distance from each "side" (front, front left, front right) of the robot to the wall        
        front = min(min(data.ranges[0:20] + data.ranges[340:359]),3.5)
        frontleft = min(min(data.ranges[31:60]),3.5)
        frontright = min(min(data.ranges[240:315]),3.5)

        # Checks to see if there are walls to follow --> if no walls to the right and nothing in front of the robot
        #   set the angular and linear velocity so it wanders to a wall. (also applicable in the case where there
        #   are walls on both sides and nothing in front of the robot)
        if front > distance and frontleft > distance and frontright > distance:
            self.twist.linear.x = .4
            self.twist.angular.z = -0.1
        elif front > distance and frontleft < distance and frontright > distance:
            self.twist.linear.x = .4
            self.twist.angular.z = -0.1
        elif front > distance and frontleft < distance and frontright < distance:
            self.twist.linear.x = .4
            self.twist.angular.z = -0.1

        # Checks if there are walls to follow --> if there are walls in front of the robot, make it turn left
        elif front < distance*2 and frontleft > distance and frontright > distance:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2
        elif front < distance*2 and frontleft > distance and frontright < distance:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2
        elif front < distance*2 and frontleft < distance and frontright < distance:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2

        # If there's a wall to the right of the robot, "follow it" by going forward while facing 90 degrees from it
        elif front > distance and frontleft > distance and frontright < distance:
            self.twist.linear.x = 0.4
            self.twist.angular.z = 0

        # Publish msg to cmd_vel
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # Declare the node and run it
    node = WallFollower()
    node.run()

