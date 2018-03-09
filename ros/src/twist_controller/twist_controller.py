import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

'''
# PID parameters
VEL_PID_P = 0.8
VEL_PID_I = 0.0005
VEL_PID_D = 0.01
'''

VEL_PID_P = 5
VEL_PID_I = 0.5
VEL_PID_D = 0.5

DEBUG_ENABLE_LPF = True

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.sampling_rate = kwargs["sampling_rate"]
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.wheel_radius = kwargs["wheel_radius"]
        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]

        #self.delta_t = 1.0 / self.sampling_rate
        self.last_timestamp = None

        # How to calculate brake torque: https://sciencing.com/calculate-brake-torque-6076252.html
        self.torque_constant = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY) * self.wheel_radius

        # PID for controlling linear velocity
        self.pid_velocity = PID(VEL_PID_P, VEL_PID_I, VEL_PID_D, self.decel_limit, self.accel_limit)

        # Yaw controller
        self.yaw_controller = YawController(wheel_base = self.wheel_base,
                                            steer_ratio = self.steer_ratio,
                                            min_speed = 0.5,                    # Minimum speed before trying to steer
                                            max_lat_accel = self.max_lat_accel,
                                            max_steer_angle = self.max_steer_angle)

        self.s_lpf = LowPassFilter(tau = 3, ts = 1)
        self.t_lpf = LowPassFilter(tau = 3, ts = 1)


    def control(self, current_linear_velocity, required_linear_velocity, required_angular_velocity):
        throttle, brake, steering = 0.0, 0.0, 0.0

        # Timing
        if self.last_timestamp is None:
            self.last_timestamp = rospy.get_time()
            return throttle, brake, steering
        timestamp = rospy.get_time()
        delta_t = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        # To help keeping the car still at traffic lights
        if abs(required_linear_velocity) < 0.5:
            self.pid_velocity.reset()
            # Keep the brake pedal down when at the traffic light to prevent "wheezing"
            throttle = 0
            brake = 100
            steering = 0
            return throttle, brake, steering

        # Get difference between target and current velocities
        vel_error = required_linear_velocity - current_linear_velocity

        # Use PID controller to compute desired acceleration
        desired_accel = self.pid_velocity.step(vel_error, delta_t)
        steering = self.yaw_controller.get_steering(required_linear_velocity, required_angular_velocity, current_linear_velocity)

        # Low pass filtering commands
        if DEBUG_ENABLE_LPF:
            steering = self.s_lpf.filt(steering)
            throttle = self.t_lpf.filt(throttle)

        if abs(required_linear_velocity) < 0.5:
            self.pid_velocity.reset()

        if desired_accel > 0.0:
            # If we want to speed up
            brake = 0.0
            throttle = desired_accel
        else:
            # If we want to slow down
            throttle = 0.0
            if abs(desired_accel) > self.brake_deadband:
                # Brake only if necessary, otherwise just let the car stop by itself
                brake = abs(desired_accel) * self.torque_constant

        # rospy.logwarn('[CTRL] dT={:0.3f}, out={:0.3f}'.format(delta_t, desired_accel))

        # Return throttle, brake, steer
        return throttle, brake, steering

    def reset(self):
        self.pid_velocity.reset()
