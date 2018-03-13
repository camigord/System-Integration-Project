#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import os

STATE_COUNT_THRESHOLD = 2

VISIBLE_DISTANCE = 100
# Use true states of traffic lights, provided by simulator
DEBUG_GROUND_TRUTH = False
DEBUG_SAVE_IMAGES = False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        model_filename = rospy.get_param("/traffic_light_model", "frozen_inference_graph_simulation_ssd")
        self.config = yaml.load(config_string)
        self.light_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.camera_image = None
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.state_count = 0
        self.last_wp = -1

        if DEBUG_SAVE_IMAGES:
            self.image_counter = 0
            self.save_dir = '/images/'

        if not DEBUG_GROUND_TRUTH:
            # TLClassifier now takes the "model_filename", read from parameter "/traffic_light_model", as model
            self.light_classifier = TLClassifier(model_filename)
            rospy.spin()
        else:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.process_traffic_lights()
                rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """
        Callback function for camera images
        """
        self.camera_image = msg
        if DEBUG_SAVE_IMAGES:
            self.img_save(msg)
        self.process_traffic_lights()

    def img_save(self, img):
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        img.encoding = "rgb8"
        cv_image = CvBridge().imgmsg_to_cv2(img, "bgr8")
        file_name = curr_dir + self.save_dir+ '/img_'+'%06d'% self.image_counter +'.png'
        rospy.logwarn("file_name = %s", file_name)
        cv2.imwrite(file_name, cv_image)
        self.image_counter += 1
        rospy.logwarn("[TD] Image saved!")

    """
    Creates a pose for the traffic light in the format required by get_closest_waypoint function instead of creating a new one
    """
    def create_light_site(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = 'world'

        # Create a Pose object to place inside the TrafficLight object
        light.pose = PoseStamped()

        light.pose.header = Header()
        light.pose.header.stamp = rospy.Time.now()
        light.pose.header.frame_id = 'world'

        light.pose.pose.position.x = x
        light.pose.pose.position.y = y
        light.pose.pose.position.z = z

        # For reference: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi * yaw / 180.0)
        light.pose.pose.orientation = Quaternion(*q)

        light.state = state

        return light


    """
    2D Euclidean distance
    """
    def distance2d(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """

        dist = float('inf')
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp = 0
        for i in range(len(self.waypoints.waypoints)):
            new_dist = dl(pose.position, self.waypoints.waypoints[i].pose.pose.position)
            if new_dist < dist:
                dist = new_dist
                wp = i
        return wp

    def get_light_state(self):
        """Determines the current color of the traffic light
        """
        if self.camera_image is None:
            rospy.logwarn('[Detector] No image')
            return False
        else:
            self.camera_image.encoding = "rgb8"
            #return TrafficLight.UNKNOWN
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
            #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            #Get classification
            # tic = rospy.get_time()
            state = self.light_classifier.get_classification(cv_image)
            # tac = rospy.get_time()
            # rospy.logwarn("Detection time: {:1.6f}s".format(tac-tic))

            # If classification result is unknown, return last state
            if state == TrafficLight.UNKNOWN and self.last_state:
                state = self.last_state
            return state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color. It then publishes
        """

        light = None
        # Proceed only if the first messages have been received
        if hasattr(self, 'pose') and hasattr(self, 'waypoints'):
            # Get position of car
            car_wp = self.get_closest_waypoint(self.pose.pose)

            # If using GT, make sure the data have been received
            if DEBUG_GROUND_TRUTH:
                if hasattr(self, 'lights'):
                    # Real state of traffic lights
                    light_positions_state = self.lights
                else:
                    # If message has not been received yet
                    rospy.logwarn('Traffic_lights not received yet')
                    return

            # Stop poritions for each traffic light
            light_positions = self.light_positions

            min_distance = float('inf')
            for i, light_position in enumerate(light_positions):
                if DEBUG_GROUND_TRUTH:
                    true_state = light_positions_state[i].state
                    light_candidate = self.create_light_site(light_position[0], light_position[1], 0.0, 0.0, true_state)
                else:
                    light_candidate = self.create_light_site(light_position[0], light_position[1], 0.0, 0.0, TrafficLight.UNKNOWN)

                light_wp = self.get_closest_waypoint(light_candidate.pose.pose)

                # Check that's ahead of us and not behind and is in sight
                light_distance = self.distance2d(self.waypoints.waypoints[car_wp].pose.pose.position.x,
                                                 self.waypoints.waypoints[car_wp].pose.pose.position.y,
                                                 self.waypoints.waypoints[light_wp].pose.pose.position.x,
                                                 self.waypoints.waypoints[light_wp].pose.pose.position.y)

                # Getting the closest traffic light which is ahead of us and within visible distance
                if (light_wp % len(self.waypoints.waypoints)) > (car_wp % len(self.waypoints.waypoints)) and (light_distance < VISIBLE_DISTANCE) and (light_distance < min_distance):
                    light = light_candidate
                    closest_light_wp = light_wp
                    min_distance = light_distance

            if light:
                if DEBUG_GROUND_TRUTH:
                    # Use ground truth provided by simulator
                    state = light.state
                else:
                    # Nominal mode: light state comes from classifier
                    state = self.get_light_state()
                rospy.logwarn('[TD] Traffic light id {} in sight, color state: {}'.format(closest_light_wp, state))
                light_wp = closest_light_wp
            else:
                light_wp = -1
                state = TrafficLight.UNKNOWN

        else: # if messages are not received yet (only for beginning of simulation), return position of first traffic light with RED status
            # light_wp = 292 will raise a problem in second test
            light_wp = -1
            state = TrafficLight.RED

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state in [TrafficLight.RED, TrafficLight.YELLOW] else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

        '''
        if state == TrafficLight.RED:
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(-1))
        '''

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
