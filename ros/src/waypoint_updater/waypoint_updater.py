#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32

import math
import numpy as np
import tf

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
BRAKE_BUFFER_M = 5 # How far from the stop line we would like the car to stop, to have some margin
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

        self.final_waypoints = None
        self.traffic_waypoint = -1

        # FSM state
        # 0 = drive normally
        # 1 = brake
        self.state = 0
        self.breaking_acceleration = None
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
            
            # Useful variables to take decisions
            traffic_light_distance = self.distance_wp(self.base_waypoints.waypoints, next_wp, self.traffic_waypoint)

            # State transition
            if self.state == 0:
                # Check if there's a traffic light in sight
                if self.traffic_waypoint != -1:
                    # Get minimum stopping distance
                    min_distance = abs(self.current_velocity**2 / (2*self.breaking_acceleration_limit))
                    rospy.logwarn("Traffic light distance / min stop distance: {} / {}".format(traffic_light_distance, min_distance))
                    # Decide what to do if there's not enough room to brake
                    if traffic_light_distance > min_distance:
                        self.state = 1
                        # compute braking deceleration
                        self.breaking_acceleration = abs(self.current_velocity**2 / (2*traffic_light_distance))
                        rospy.logwarn("Braking deceleration: {}".format(self.breaking_acceleration))
                    else:
                        rospy.logwarn("Too late to break !!")
                        self.state = 0 
                else:
                    self.state = 0

            elif self.state == 1:
                if self.traffic_waypoint == -1:
                    # There's no red anymore, get back to speed
                    self.state = 0

            #rospy.logwarn('Next wp: {}, Traffic wp: {}, State: {}, Vel: {}'.format(next_wp, self.traffic_waypoint, self.state, self.current_velocity))

            # State action, calculate next waypoints
            self.calculate_final_waypoints(next_wp)
            #self.print_final_waypoints(10)
            # Publish final waypoints
            self.publish_waypoints()

    """
    Calculate the final waypoints to follow
    """
    # TODO
    # There a few bugs in this function I can't find. I'm reasonably sure all the rest is fine and they're concentrated here.
    # When starting, it takes some time to switch from state = 0 to 1. This is not a bug (or at least not a bug here), the reason is that the traffic_waypoint message takes some time to arrive.
    #
    # BUGS:
    # As the traffic lights are all red now as default, the only way to trigger a restart is to drive manually a little beyond the stop line and then switch again back to automatic. For some reason, the self.base_waypoints are not all 11.11 but some are zeros and it takes some time to get rid of these 0s. No fuckin idea why. 
    #
    # THING THAT WORK:
    # The car stops at the traffic light (most of the times, when it doesn't I guess it's a PID problem as the waypoint's velocities are 0)
    def calculate_final_waypoints(self, start_wp):           

        # Empty output list
        self.final_waypoints = []

        if self.state == 0:
            for i in range(start_wp, start_wp + LOOKAHEAD_WPS):
                j = i % len(self.base_waypoints.waypoints)
                tmp = Waypoint()
                tmp.pose.pose = self.base_waypoints.waypoints[j].pose.pose
                tmp.twist.twist.linear.x = self.base_waypoints.waypoints[j].twist.twist.linear.x
                self.final_waypoints.append(tmp)

        elif self.state == 1:
            stop_bw = self.traffic_waypoint
            
            # Waypoints before the traffic light -> set pose/speed to base_waypoint's values
            for i in range(start_wp, stop_bw):
                j = i % len(self.base_waypoints.waypoints)
                tmp = Waypoint()
                tmp.pose.pose = self.base_waypoints.waypoints[j].pose.pose
                tmp.twist.twist.linear.x = self.base_waypoints.waypoints[j].twist.twist.linear.x
                self.final_waypoints.append(tmp)
    
            # Brake to target
            target_wp = len(self.final_waypoints)
            
            # Waypoints after the traffic light -> set pose to base_waypoint's pose and set speed to 0
            i_max = max(start_wp + LOOKAHEAD_WPS, stop_bw+1)
            for i in range(stop_bw, i_max):
                j = i % len(self.base_waypoints.waypoints)
                tmp = Waypoint()
                tmp.pose.pose = self.base_waypoints.waypoints[j].pose.pose
                tmp.twist.twist.linear.x  = 0.0
                self.final_waypoints.append(tmp)
    
            # Waypoints before the traffic light -> set their speed considering a specific braking acceleration
            last = self.final_waypoints[target_wp]
            last.twist.twist.linear.x = 0.0
            
            for wp in self.final_waypoints[:target_wp][::-1]:
                dist = self.distance_poses(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0.0, dist-BRAKE_BUFFER_M)
                vel  = math.sqrt(2*self.breaking_acceleration*dist)  # use maximum braking acceleration
                if vel < 1.0:
                    vel = 0.0
                wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

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
        next_wp = self.get_closest_waypoint()
        heading = math.atan2((self.base_waypoints.waypoints[next_wp].pose.pose.position.y - self.current_pose.pose.position.y), (self.base_waypoints.waypoints[next_wp].pose.pose.position.x - self.current_pose.pose.position.x))
        theta = tf.transformations.euler_from_quaternion([self.current_pose.pose.orientation.x,
                                                          self.current_pose.pose.orientation.y,
                                                          self.current_pose.pose.orientation.z,
                                                          self.current_pose.pose.orientation.w])[-1]
        angle = math.fabs(theta-heading)
        if angle > math.pi / 4.0:
            next_wp += 1
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
        msg.waypoints = self.final_waypoints[:LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(msg)

    """
    Print the next N final waypoints
    """
    def print_final_waypoints(self, n):
        s = "final wp: "
        for i in range(0,n):
            s = s + "  {}".format(self.final_waypoints[i].twist.twist.linear.x)
        rospy.logwarn(s)

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
        #rospy.logwarn('{} New pose received'.format(rospy.Time().now()))

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
	#rospy.logwarn('Waypoint msg received: {}'.format(self.base_waypoints))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data
        #rospy.logwarn('Traffic msg received: {}'.format(self.traffic_waypoint))

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
