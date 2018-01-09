#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
TARGET_SPEED = 4 # Meters per second

'''
01/09/2018 - A simple first implementation of the waypoint_updater
	1. The car locates the nearest waypoint to itself
		- If this is the first attempt, it searches all waypoints around the track
		- If a previous closest waypoint is known, the search starts from there down the road 100 points
	2. The car picks out the next 200 (or LOOKAHEAD_WPS) waypoints to follow
	3. These next waypoints are published
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
       

	# Publishing next waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_position = None
	self.current_position_idx = None
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        # Grab current position from the current_pose message
        self.current_position = msg.pose

        ### Finding the nearest waypoint ahead
	# If the previous nearest point is unknown, start the search from the beginning
	if (self.current_position_idx == None):
		min_dist = 1e6
		min_idx = 0
		for i, waypoint in enumerate(self.base_waypoints):
			dist = self.straight_distance(self.current_position.position.x, self.current_position.position.y, waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)
			if (dist < min_dist):
				min_dist = dist
				min_idx = i
	# If the previous nearest point is known, start the search from that point going forward	
	else:
		min_idx = self.current_position_idx		
		min_dist = self.straight_distance(self.current_position.position.x, self.current_position.position.y, self.base_waypoints[min_idx].pose.pose.position.x, self.base_waypoints[min_idx].pose.pose.position.y)
		
		## Looking ahead to the next 100 points to find the minimum
		# This is likely not so rubust!
		search_window = 100;
		if (min_idx + search_window <= len(self.base_waypoints)):
			window = range(min_idx, min_idx + search_window)
		else:
			window = range(min_idx, len(self.base_waypoints)) + range(0, (min_idx + search_window - len(self.base_waypoints)))
		# Looking in the "window" to find the nearest next point
		for i in window:
			waypoint = self.base_waypoints[i]
			dist = self.straight_distance(self.current_position.position.x, self.current_position.position.y, waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)
			if (dist < min_dist):
				min_dist = dist
				min_idx = i		

	# Saving this position so we can start searching from this point for the next minimum distance
	self.current_position_idx = min_idx

        # Chopping out the next LOOKAHEAD_WPS number of waypoints, accounting for wraparound
        if (min_idx + LOOKAHEAD_WPS < len(self.base_waypoints)):
        	next_points = self.base_waypoints[min_idx:min_idx+LOOKAHEAD_WPS]
        else:
        	next_points = self.base_waypoints[min_idx:] + self.base_waypoints[:(min_idx+LOOKAHEAD_WPS - len(self.base_waypoints))] 
        
        # Set target speed per waypoint
	for waypoint in next_points:
		waypoint.twist.twist.linear.x = TARGET_SPEED
        
        # Generate the message with the list of waypoints to be followed
        next_points_msg = Lane()
        next_points_msg.header.frame_id = '/world'
        next_points_msg.header.stamp = rospy.Time(0)
        next_points_msg.waypoints = next_points

        # Publish the list of waypoints to be followed
        self.final_waypoints_pub.publish(next_points_msg)
   
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Grab the list of all waypoints from the base_waypoints message
        self.base_waypoints = waypoints.waypoints
        self.sub_base_waypoints.unregister()

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

    def straight_distance(self,x1,y1,x2,y2):
	return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
