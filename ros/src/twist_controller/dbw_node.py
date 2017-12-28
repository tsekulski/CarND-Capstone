#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

'''
My (potential) approach:
- INPUT: /twist_cmd (target linear and angular velocities), /current_velocity (current linear and angular velocity?), /vehicle/dbw_enabled
- OUPUT: steering angle, throttle, brake
- Genral flow:
1a. Create & initialize controller object
1b. [Loop] Grab target and current velocities (linear and angular) from the relevant topic subscriptions
2. [Loop] Call the controller, with target and current velocities as input; controller should return steering, throttle, brake
3. Publish steer, throttle, brake to relevant topics

Steering angle controller:
- Udacity's suggestion: use yaw_controller.py, a controller that can be used to convert target linear and angular velocity to steering commands.
- TBD: use of lowpass filter?
- TBD: adjustment of any parameters, like e.g. steer ratio (this apparently has been fixed in the latest code release)

Throttle and brake controller:
* Note that throttle values passed to publish should be in the range 0 to 1, although a throttle of 1 means the vehicle throttle will be fully engaged. 
* Brake values passed to publish should be in units of torque (N*m). 
* The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.
- Use PID controller
- TBD: Kp, Kd, Ki values
- TBD: User of low-pass filter
- Calculating throttle value between 0 and 1 seems straightforward
- TBD: How to compute brake values?

First partial step: Implement steering control, for throttle and brake just publish hardcoded values to test whether everything works.
'''

DEBUGGING = True

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(
            vehicle_mass,
            fuel_capacity,
            brake_deadband,
            decel_limit,
            accel_limit,
            wheel_radius,
            wheel_base,
            steer_ratio,
            max_lat_accel,
            max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        self.sub_twist_cmd = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        self.sub_current_velocity = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        self.sub_dbw_enabled = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # Create variables to store the latest state coming from the subscriptions
        self.target_linear_velocity = 0.
        self.target_angular_velocity = 0.
        self.current_linear_velocity = 0.
        self.current_angular_velocity = 0.
        self.dbw_enabled = False
        # self.reset = False # Reset the "autonomous" state in case a human takes over

        self.previous_time = rospy.get_time()

        if DEBUGGING:
            rospy.logwarn("DBW Node initialized")

        self.loop()

    # Define callback functions
    def twist_cmd_cb(self, twist_cmd):
        self.target_linear_velocity = twist_cmd.twist.linear.x
        self.target_angular_velocity = twist_cmd.twist.angular.z
        if DEBUGGING:
            rospy.logwarn("twist_cmd_cb called")
            rospy.logwarn("target_linear_velocity = %s", self.target_linear_velocity)
            rospy.logwarn("target_angular_velocity = %s", self.target_angular_velocity)

    def current_velocity_cb(self, current_velocity):
        self.current_linear_velocity = current_velocity.twist.linear.x
        self.current_angular_velocity = current_velocity.twist.angular.z
        if DEBUGGING:
            rospy.logwarn("current_velocity_cb called")
            rospy.logwarn("current_linear_velocity = %s", self.current_linear_velocity)
            rospy.logwarn("current_angular_velocity = %s", self.current_angular_velocity)

    def dbw_enabled_cb(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled
        if DEBUGGING:
            rospy.logwarn("dbw_enabled_cb called, dbw_enabled = %s", dbw_enabled)

    def loop(self):
        rate = rospy.Rate(10) # Changed to 10 Hz from 50Hz to avoid simulator latency issues
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            #############
            current_time = rospy.get_time()
            time_diff = current_time - self.previous_time
            self.previous_time = current_time
            #############

            throttle, brake, steer = self.controller.control(
                self.target_linear_velocity,
                self.target_angular_velocity,
                self.current_linear_velocity,
                self.current_angular_velocity,
                self.dbw_enabled,
                time_diff)

            if self.dbw_enabled:
                self.publish(throttle, brake, steer)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
