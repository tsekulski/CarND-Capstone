from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy
import random

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DEBUGGING = True

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
       	self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]

        # Define minimum speed for the YawController - my understanding is that this is simply to ensure that the car is not steering when it's not moving
        self.min_speed = 0. # m/s


        # Createa a YawController object - to be used to generate steering values
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

        # Create a LowPassFilter object - to smoothen steering angles
        # self.lowpass = LowPassFilter(0.96, 1)

        # Create a PID controller for throttle
        self.pid_filter = PID(kp=2.6, ki=0.0, kd=1.3, mn=self.decel_limit, mx=self.accel_limit)


        if DEBUGGING:
            rospy.logwarn("Controller object initialized")

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        target_linear_velocity = args[0]
        target_angular_velocity = args[1]
        current_linear_velocity = args[2]
        current_angular_velocity = args[3]
        dbw_enabled = args[4]
        time_diff = args[5]

        # Get steering angle
        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        # Smoothen steering angle
        # steer = self.lowpass.filt(steer)

        # Note to self: implement also controllers for throttle and brake
        #steer = -5.0
        #throttle = 0.07 + random.randint(1,5) / 100.0
        error = target_linear_velocity - current_linear_velocity
        throttle = self.pid_filter.step(error, time_diff)

        brake = 0.

        if DEBUGGING:
            #rospy.logwarn("Control values returned")
            rospy.logwarn("steer %s", steer)
            rospy.logwarn("throttle %s", throttle)
            #rospy.logwarn("brake %s", brake)

        return throttle, brake, steer
