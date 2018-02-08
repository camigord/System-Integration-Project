#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',  Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # self.base_waypoints = None
        self.final_waypoints = None
        # self.current_pose = None

        self.decimator_i = 0
        self.decimator_n = 10

        rospy.spin()

        # Comment the above and uncomment this to use the timer
        """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.action()
            rate.sleep()
        """

    """
    Main action
    """
    def action(self):
        # Proceed only if the first messages have been received
        if hasattr(self, 'current_pose') and hasattr(self, 'base_waypoints'):

            # Get the closest waypoint id
            closest_waypoint_id = self.get_closest_waypoint()
            rospy.logwarn('Closest waypoint id: {}'.format(closest_waypoint_id))

            # Calculate next waypoints
            self.final_waypoints = self.calculate_final_waypoints(closest_waypoint_id, LOOKAHEAD_WPS)

            # Publish final waypoints
            self.publish_waypoints()

    def publish_waypoints(self):
        msg = Lane()
        # msg.header = self.base_waypoints.header
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = '/world'
        msg.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(msg)


    """
    Callbacks
    """
    def pose_cb(self, msg):
        # First thing first, get the current pose
        self.current_pose = msg
        rospy.logwarn('{} New pose received'.format(rospy.Time().now()))
        
        # Trigger action
        if self.decimator_i == self.decimator_n:
            self.decimator_i = 0
            self.action()
        else:
            self.decimator_i = self.decimator_i + 1

    def waypoints_cb(self, waypoints):
        # Storing waypoints given that they are published only once
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # Note that wp1 and wp2 are indexes
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    """
    Return the id of the waypoint closest to the pose
    """
    def get_closest_waypoint(self):
        dist = float('inf')
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp = 0
        for i in range(len(self.base_waypoints.waypoints)):
            new_dist = dl(self.current_pose.pose.position, self.base_waypoints.waypoints[i].pose.pose.position)
            if new_dist < dist:
                dist = new_dist
                wp = i
        return wp

    """
    Calculate the fnal waypoints to follow. For the moment this is just the list of the next base_waypoints.
    """
    def calculate_final_waypoints(self, closest_waypoint_id, n):
        next_waypoints = []
        for i in range(closest_waypoint_id, (closest_waypoint_id + n)):
            # Make the index modulo lenght of base_waypoints
            j = i % len(self.base_waypoints.waypoints)
            next_waypoints.append(self.base_waypoints.waypoints[j])
        return next_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
