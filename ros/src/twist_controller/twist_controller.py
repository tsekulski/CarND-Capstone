from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

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
        self.min_speed = 0.1 # m/s


        # Createa a YawController object - to be used to generate steering values
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

        # Create a LowPassFilter object - to smoothen steering angles
        self.lowpass = LowPassFilter(3, 1)

        if DEBUGGING:
            rospy.logwarn("Controller object initialized")

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        self.target_linear_velocity = args[0]
        self.target_angular_velocity = args[1]
        self.current_linear_velocity = args[2]
        self.current_angular_velocity = args[3]
        self.dbw_enabled = args[4]

        # Get steering angle
        steer = self.yaw_controller.get_steering(self.target_linear_velocity, self.target_angular_velocity, self.current_linear_velocity)

        # Smoothen steering angle
        steer = self.lowpass.filt(steer)

        # Note to self: implement also controllers for throttle and brake
        throttle = 0.15
        brake = 0.

        if DEBUGGING:
            rospy.logwarn("Control values returned")
            rospy.logwarn("steer %s", steer)
            print "steer =", steer
            #ospy.loginfo('throttle =', throttle)
            #rospy.loginfo('brake =', brake)

        return throttle, brake, steer
