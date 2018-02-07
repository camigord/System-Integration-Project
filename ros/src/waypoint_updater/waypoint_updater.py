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

        self.waypoints = None
        self.final_waypoints = None

        rospy.spin()

    def publish_waypoints(self):
        msg = Lane()
        msg.header = self.base_waypoints.header
        msg.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(msg)

    """
    Callbacks
    """
    def pose_cb(self, msg):
        #
        dist_f = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
        distances = []
        if self.base_waypoints:      # If waypoints have already been received
            # Find the closest waypoint to current pos
            for waypoint in self.base_waypoints.waypoints:
                distances.append(dist_f(waypoint.pose.pose.position, msg.pose.position))

            # Get index of closest waypoint
            self.closest_index = np.argmin(distances)

            # We need to add the waypoints in front of us, but we need to consider this is a circular track
            if self.closest_index + LOOKAHEAD_WPS + 1 > np.shape(self.base_waypoints.waypoints)[0]:
                # We are close to the end of the track, we need to take points from the beginning
                end_index = self.closest_index + LOOKAHEAD_WPS + 1 - np.shape(self.waypoints.waypoints)[0]
                self.final_waypoints = self.base_waypoints.waypoints[self.closest_index:] + self.base_waypoints.waypoints[:end_index]
            else:
                self.final_waypoints = self.base_waypoints.waypoints[self.closest_index:self.closest_index+LOOKAHEAD_WPS+1]

            # Publish final waypoints
            self.publish_waypoints()

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
