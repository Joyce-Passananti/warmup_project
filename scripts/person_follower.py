#!/usr/bin/env python3

import rospy
 
# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# This node follows the nearest person or object
class PersonFollower(object):

    def __init__(self):
        # Start rospy node
        rospy.init_node("follow_the_Person")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)

    def process_scan(self, data):
        # Determine direction of object by looking at scan data from in front of
        #   the robot, set linear/angular velocity based on that information, and
        #   publish to cmd_vel.

        # Find the shortest distance from the robot to anything
        #   Find the index of that distance in the ranges array
        #   Use that index to set the angular velocity of the robot

        # Iterate through ranges to find shortest distance/index
        shortest = data.ranges[0]
        shortest_index = 0
        i = 0
        for x in data.ranges:
            if x < shortest:
                shortest = x
                shortest_index = i
            i = i + 1

        # Check to see if the angle is approximately correct, go forward if right direction
        if shortest_index < 8 or shortest_index > 352:
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0
        # Otherwise rotate the robot in the fastest direction
        else:
            if shortest_index < 180 :
                self.twist.angular.z = .6
            else: 
                self.twist.angular.z = -.6

        # When close to the person/object, stop moving forward
        if shortest < 0.75:
            self.twist.linear.x = 0

        # Publish msg to cmd_vel
        self.twist_pub.publish(self.twist)


    def run(self):
        # Keep the program alive
        rospy.spin()

if __name__ == '__main__':
    # Declare the node and run it
    node = PersonFollower()
    node.run()

