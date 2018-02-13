#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
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
BRAKE_BUFFER_WP = 1  # How far from the stop line we would like the car to stop, to have some margin
BRAKE_BUFFER_M = 5
USE_TIMER_TRIGGERED = 0  # Defines whether the action is triggered by a timer or by an incoming pose 


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # self.base_waypoints = None
        self.final_waypoints = None
        # self.current_pose = None
        self.traffic_waypoint = -1

        # FSM state
        # 0 = drive normally
        # 1 = brake
        self.state = 0
        self.breaking_acceleration = 1
        self.breaking_acceleration_limit = rospy.get_param('~decel_limit', -5)

        if USE_TIMER_TRIGGERED:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.action()
                rate.sleep()
        else:
            self.decimator_i = 0
            self.decimator_n = 10
            rospy.spin()

    """
    Main action
    """
    def action(self):
        # Proceed only if the first messages have been received
        if hasattr(self, 'current_pose') and hasattr(self, 'base_waypoints'):

            # Start from the next (see later whether to make it more sophisticated)
            next_wp = self.get_next_waypoint()
            # rospy.logwarn('Next waypoint id: {}'.format(next_wp))

            # State transition
            if self.state == 0:
                # Check if there's a traffic light in sight
                if self.traffic_waypoint != -1:
                    # Get distance from light and minimum stopping distance
                    traffic_light_distance = self.distance_wp(self.base_waypoints.waypoints, next_wp, self.traffic_waypoint)
                    min_distance = self.current_velocity**2 / (2*self.breaking_acceleration_limit)
                    rospy.logwarn("Traffic light distance: {}".format(traffic_light_distance))
                    # Decide what to do if there's not enough room to brake
                    if traffic_light_distance > min_distance:
                        self.state = 1
                    else:
                        self.state = 1
                else:
                    self.state = 0

            elif self.state == 1:
                # Check if the red traffic light is still there
                if self.traffic_waypoint != -1:
                    self.state = 1
                # If not, get back to speed
                else:
                    self.state = 0

            rospy.logwarn('State: {}'.format(self.state))

            # State action, calculate next waypoints
            if self.state == 0:
                self.final_waypoints = self.calculate_final_waypoints(self.state, next_wp, next_wp+LOOKAHEAD_WPS)

            elif self.state == 1:
                self.final_waypoints = self.calculate_final_waypoints(self.state, next_wp, self.traffic_waypoint)

            # Publish final waypoints
            self.publish_waypoints()

    """
    Calculate the final waypoints to follow
    """
    def calculate_final_waypoints(self, state, start_wp, end_wp):
        # Empty output list
        final_waypoints = []

        # Just make sure end_wp is bigger than start_wp, if not (because of the circular path), add the length of the path. This isn't a problem because the actual index, j, is then taken as modulo path length
        if end_wp < start_wp:
            end_wp = end_wp + len(self.base_waypoints.waypoints)

        # Keep the same speed
        if state == 0:

            # Just copy the waypoints ahead            
            for i in range(start_wp, start_wp + LOOKAHEAD_WPS):
                j = i % len(self.base_waypoints.waypoints)
                tmp = self.base_waypoints.waypoints[j]
                final_waypoints.append(tmp)


        # Brake in order to stop in front of the traffic light
        elif state == 1:

            # For the waypoints up to the traffic light, decrease speed linearly         
            for i in range(start_wp, end_wp):
                j = i % len(self.base_waypoints.waypoints)
                tmp = self.base_waypoints.waypoints[j]

                # Decrease velocity linearly with some buffer
                traffic_light_distance_in_wp = end_wp - BRAKE_BUFFER_WP - start_wp

                if traffic_light_distance_in_wp > 0:
                    v = self.current_velocity - self.current_velocity/traffic_light_distance_in_wp*(j-start_wp)

                else:
                    v = 0.0

                # Clamp velocity
                if v < 1.0:
                    v = 0.0
                tmp.twist.twist.linear.x = min(v, tmp.twist.twist.linear.x)

                final_waypoints.append(tmp)

            # For the waypoints after the traffic light, set their speed to 0
            for i in range(end_wp+1, start_wp + LOOKAHEAD_WPS):
                j = i % len(self.base_waypoints.waypoints)

                tmp = self.base_waypoints.waypoints[j]
                tmp.twist.twist.linear.x = 0.0
                final_waypoints.append(tmp)

        return final_waypoints

    """
    Calculate the final waypoints to follow, another version
    """
    def calculate_final_waypoints2(self, state, start_wp, end_wp):
        # Empty output list
        final_waypoints = []
        
        # To be done in either state
        for i in range(start_wp, end_wp):
            j = i % len(self.base_waypoints.waypoints)
            tmp = self.base_waypoints.waypoints[j]

            # Reach stop line if we stopped too early
            if state == 1:
                dist = self.distance_poses(tmp.pose.pose.position, self.base_waypoints.waypoints[end_wp].pose.pose.position)
                if dist > BRAKE_BUFFER_M and self.current_velocity < 1.0:
                    tmp.twist.twist.linear.x = 2.0
                elif dist < BRAKE_BUFFER_M and self.current_velocity < 1.0:
                    tmp.twist.twist.linear.x = 0.0
                else:
                    tmp.twist.twist.linear.x = min(self.current_velocity, self.base_waypoints.waypoints[j].twist.twist.linear.x)
            else:
                tmp.twist.twist.linear.x = self.base_waypoints.waypoints[j].twist.twist.linear.x
            final_waypoints.append(tmp)

        if state == 1:
            # Brake to target
            target_wp = len(final_waypoints)

            # For the waypoints after the traffic light, set their speed to 0
            for i in range(end_wp, start_wp + LOOKAHEAD_WPS):
                j = i % len(self.base_waypoints.waypoints)
                tmp = self.base_waypoints.waypoints[j]
                tmp.twist.twist.linear.x  = 0.0
                final_waypoints.append(tmp)

            last = final_waypoints[target_wp]
            last.twist.twist.linear.x = 0.0
            for wp in final_waypoints[:target_wp][::-1]:
                dist = self.distance_poses(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0.0, dist-BRAKE_BUFFER_M)
                vel  = math.sqrt(2*self.breaking_acceleration*dist)
                if vel < 1.0:
                    vel = 0.0
                wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return final_waypoints

    """
    Return the id (wp) of the waypoint closest to the pose
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
    Returns the id (wp) of the waypoint immediately ahead of the current pose
    """
    def get_next_waypoint(self):
        # Get the closest waypoint id
        closest_wp = self.get_closest_waypoint()
        next_wp = closest_wp + 1
        return next_wp

    """
    Return distance between two waypoints
    """
    def distance_wp(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    """
    Return distance between to poses
    """
    def distance_poses(self, p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    """
    Publish waypoints on the approrpiate topic
    """
    def publish_waypoints(self):
        msg = Lane()
        # msg.header = self.base_waypoints.header
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = 'world'
        msg.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(msg)

    """
    Get and set linear velocity for a single waypoint id (wp) in a list of waypoints
    Unused
    """
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    """
    Callbacks
    """
    def pose_cb(self, msg):
        # First thing first, get the current pose
        self.current_pose = msg
        # rospy.logwarn('{} New pose received'.format(rospy.Time().now()))

        # Trigger action
        if not USE_TIMER_TRIGGERED:
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
        # rospy.logwarn('Traffic msg received: {}'.format(self.traffic_waypoint))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_waypoint = msg.data
        # rospy.logwarn('Obstacle msg received: {}'.format(self.obstacle_waypoint))

    def current_velocity_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.current_velocity = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
