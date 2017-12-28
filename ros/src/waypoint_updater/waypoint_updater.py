#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
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

'''
My potential call flow:
0. Extract and store base_waypoints (as they are published only once)
1. Determine the closest waypoint ahead of the car 
- by comparing current_pose with base_waypoints
- extract the sequence number of this closest waypoint
- tbd: convert from vehicle to global coordinates?
2. Generate x = LOOKAHEAD_WPS waypoint objects ahead of the vehicle:
- in the first step w/o any target velocity (or with constant target velocity)
- in the second step with target velocities depeding on detected traffic lights and/or obstacles
3. Publish the waypoint objects to the final_waypoints topic

Helpful resource for transforming quaternions to Euler values:
https://answers.ros.org/question/69754/quaternion-transformations-in-python/

#type(pose) = geometry_msgs.msg.Pose
quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DEBUGGING = False
TARGET_SPEED_MPH = 10
TARGET_SPEED_MPS = TARGET_SPEED_MPH * 0.44704 # Convert from mph to m/s

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.curr_pose = None
        self.all_waypoints = None

        if DEBUGGING:
            rospy.logwarn("Waypoint Updater Node initialized")

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # Grab current position from the current_pose message
        self.curr_pose = msg.pose
        ### Find the nearest waypoint ahead
        # First, find the closest waypoint
        min_dist = 100000

        closest_index = 0
        for i, waypoint in enumerate(self.all_waypoints):
        	dist = math.sqrt(((self.curr_pose.position.x - waypoint.pose.pose.position.x) ** 2) + ((self.curr_pose.position.y - waypoint.pose.pose.position.y) ** 2))
        	if (dist < min_dist):
        		min_dist = dist
        		closest_index = i

        # Then, check whether the waypoint is behind, if it is, increment the index to make sure it's ahead
   		heading = math.atan2(self.all_waypoints[closest_index].pose.pose.position.y - self.curr_pose.position.y,
   			self.all_waypoints[closest_index].pose.pose.position.x - self.curr_pose.position.x)
        	
        quaternion = (self.curr_pose.orientation.x,
        	self.curr_pose.orientation.y,
        	self.curr_pose.orientation.z,
        	self.curr_pose.orientation.w)

        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        angle = abs(yaw - heading)
        
        if (angle > (math.pi / 4)):
        	closest_index += 1

        # If closest_index is higher than no. of items in base_waypoints --> reset closest_index to 0
        if (closest_index >= len(self.all_waypoints)):
        	closest_index = 0

        #rospy.logwarn("closest waypoint index %s", closest_index)
        
        # Generate the list of waypoints to be followed
        if (closest_index + LOOKAHEAD_WPS < len(self.all_waypoints)):
        	lookahead_waypoints = self.all_waypoints[closest_index:closest_index+LOOKAHEAD_WPS]
		# if the slice is out of bounds:
        else:
        	lookahead_waypoints = self.all_waypoints[closest_index:] + self.all_waypoints[:(closest_index+LOOKAHEAD_WPS - len(self.all_waypoints))] 
        
        # Set target speed per waypoint
        for waypoint in lookahead_waypoints:
        	waypoint.twist.twist.linear.x = TARGET_SPEED_MPS
        	# This will need to be expanded once traffic lights are recognized

        # Generate the message with the list of waypoints to be followed
        lookahead_waypoints_msg = Lane()
        lookahead_waypoints_msg.header.frame_id = '/world'
        lookahead_waypoints_msg.header.stamp = rospy.Time(0)
        lookahead_waypoints_msg.waypoints = lookahead_waypoints

        # Publish the list of waypoints to be followed
        self.final_waypoints_pub.publish(lookahead_waypoints_msg)

        if DEBUGGING:
        	rospy.logwarn("lookahead_waypoints_msg published")
        
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Grab the list of all waypoints from the base_waypoints message
        self.all_waypoints = waypoints.waypoints

        # Unsubscribe since this list is published only once
        self.sub_base_waypoints.unregister()

        if DEBUGGING:
        	rospy.logwarn("base_waypoints loaded and unsubscribed")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
