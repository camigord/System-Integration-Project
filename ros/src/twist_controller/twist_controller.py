import rospy
import math
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from std_msgs.msg import Float32

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self):
        # TODO: Implement
        pass

    def control(self, twist_cmd, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 0.5, 0., 0.
