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

STATE_COUNT_THRESHOLD = 3
VISIBLE_DISTANCE = 50

# TODO It calls self.image_cb at regular intervals even if the camera is switched off in the simulator. Deactivate this for complete testing and final release!
DEBUG_CAMERA_ALWAYS_ON = True
DEBUG_CAMERA_ALWAYS_ON_RATE = 50
# Use true states of traffic lights, provided by simulator
DEBUG_GROUND_TRUTH =  True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
	'''
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	'''
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
        self.config = yaml.load(config_string)
        self.light_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
	if not DEBUG_GROUND_TRUTH:
	    self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        # initial TL status is forced to RED so as to avoid car to throttle while we are waiting for the first TL state
	self.state = TrafficLight.RED    
        self.last_state = TrafficLight.RED
        self.last_wp = -1
        self.state_count = 0

        if DEBUG_CAMERA_ALWAYS_ON:
            rate = rospy.Rate(DEBUG_CAMERA_ALWAYS_ON_RATE)
            while not rospy.is_shutdown():
                self.image_cb(0)
                rate.sleep()
        else:
            rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
	rospy.logwarn('tl_detector: waypoint msg received')
        
    def traffic_cb(self, msg):
        self.lights = msg.lights
	#rospy.logwarn('tl_detector: traffic light msg received')

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
	#rospy.logwarn('state_count: {}, self.state: {}, state: {}'.format(self.state_count,self.state, state))
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    
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
        #TODO implement
        dist = float('inf')
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp = 0
        for i in range(len(self.waypoints.waypoints)):
            new_dist = dl(pose.position, self.waypoints.waypoints[i].pose.pose.position)
            if new_dist < dist:
                dist = new_dist
                wp = i
        return wp

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
	# Proceed only if the first messages have been received
        if hasattr(self, 'pose') and hasattr(self, 'waypoints'):
            # List of positions that correspond to the line to stop in front of for a given intersection
            car_wp = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

            # Search if there's a traffic light ahead of us
            closest_light_distance = float('inf')
	    if DEBUG_GROUND_TRUTH:
		if hasattr(self, 'lights'):
		    # use ground truth data (positions+state)
		    light_positions_state = self.lights	
		else:
		    # msg /vehicle/traffic_lights not received yet
		    rospy.logwarn('Traffic_lights not received yet, force to RED')
		    return 292, TrafficLight.RED
	    
	    light_positions = self.light_positions
	    #rospy.logwarn('Stop line positions: {}'.format(light_positions))
	    
	    i = 0
	    for light_position in light_positions:
		if DEBUG_GROUND_TRUTH:
		    true_state = light_positions_state[i].state
	            light_candidate = self.create_light_site(light_position[0], light_position[1], 0.0, 0.0, true_state)
		    i+=1
		else:
		    light_candidate = self.create_light_site(light_position[0], light_position[1], 0.0, 0.0, TrafficLight.UNKNOWN)
                light_wp = self.get_closest_waypoint(light_candidate.pose.pose)
		
                # Check that's ahead of us and not behind and is in sight
                light_distance = self.distance2d(self.waypoints.waypoints[car_wp].pose.pose.position.x,
                                                 self.waypoints.waypoints[car_wp].pose.pose.position.y,
                                                 self.waypoints.waypoints[light_wp].pose.pose.position.x,
                                                 self.waypoints.waypoints[light_wp].pose.pose.position.y)
		#rospy.logwarn('tl_det/ TL id: {}, wp: {}, dist: {}, state: {}'.format(i, light_wp, light_distance, true_state))
		if (light_wp % len(self.waypoints.waypoints)) > (car_wp % len(self.waypoints.waypoints)) and (light_distance < VISIBLE_DISTANCE):
                    closest_light_distance = light_distance
                    light = light_candidate
                    closest_light_wp = light_wp

            if light:
	    	if DEBUG_GROUND_TRUTH:
		    # Use ground truth provided by simulator
            	    state = light.state
	        else: 
                    # Nominal mode: light state comes from classifier
                    state = self.get_light_state(light)
	        rospy.logwarn('Next traffic light: {} , color state: {}'.format(closest_light_wp, state))
	        return closest_light_wp, state
	    else:
	        return -1, TrafficLight.UNKNOWN

	# if messages are not received yet (only for beginning of simulation), return position of first traffic light with RED status
	return 292, TrafficLight.RED
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
