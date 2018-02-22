#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

DISTANCE_DECELERATION = 20 # Distance to start deceleration in m
DISTANCE_STOP = 1 # Distance to stop before red traffic light
search_window = 10

'''
01/09/2018 - A simple first implementation of the waypoint_updater
	1. The car locates the nearest waypoint to itself
		- If this is the first attempt, it searches all waypoints around the track
		- If a previous closest waypoint is known, the search starts from there up and down the road n number of points
	2. The car picks out the next 200 (or LOOKAHEAD_WPS) waypoints to follow
	3. These next waypoints are published

01/20/2018 - 2nd implementation of waypoint_updater
    1. Subscribe traffic_waypoint from the tl_detector
    2. Subscribe current_velocity from the simulator
    3. Velocity update based on the input light state and current velocity
    4. Deceleration then red traffic light detected
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb, queue_size=1)
       
	    # Publishing final waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Get parameter
        self.cruise_speed = self.kpm_to_mps(rospy.get_param('~/waypoint_loader/velocity', 40.0))
        self.decel_limit = abs(rospy.get_param('~/twist_controller/decel_limit', -5))
        self.accel_limit = rospy.get_param('~/twist_controller/accel_limit', 1)

        # Add other member variables you need below
        self.current_position = None
        self.current_position_idx = None
        self.current_velocity = 0.1
        self.base_waypoints = None # Read from base waypoints
        self.base_waypoints_size = 0
        self.final_waypoints = None # Send for final waypoints

        self.traffic_point_idx = None # Traffic light waypoint index recieved from traffic waypoint
        self.red_light_detected = False # If red state of traffic light detected

        rospy.spin()


    def pose_cb(self, msg):
        # Grab current position from the current_pose message
        self.current_position = msg.pose

        ## Finding the nearest waypoint ahead
	    # If the previous nearest point is unknown, start the search from the beginning
        if (self.current_position_idx == None):
            min_dist = 1e6
            min_idx = 0
            for i, waypoint in enumerate(self.base_waypoints):
                dist = self.straight_distance(self.current_position.position.x, self.current_position.position.y, waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)
                if (dist < min_dist):
                    min_dist = dist
                    min_idx = i
        # If the previous nearest point is known, start the search from that point going up and down the road	
        else:
            min_idx = self.current_position_idx		
            min_dist = self.straight_distance(self.current_position.position.x, self.current_position.position.y, self.base_waypoints[min_idx].pose.pose.position.x, self.base_waypoints[min_idx].pose.pose.position.y)
            
            ## Creating the window around the last known closest point to find the minimum
            window = range(min_idx - search_window, min_idx + search_window) # The window of where to look		

            # Looking in the "window" to find the nearest next point
            for i in window:
                this_idx = i%(self.base_waypoints_size - 1)			
                waypoint = self.base_waypoints[this_idx]

                # Distance to the car's location
                dist = self.straight_distance(self.current_position.position.x, self.current_position.position.y, waypoint.pose.pose.position.x, waypoint.pose.pose.position.y)		
            
                if (dist < min_dist):
                    min_dist = dist
                    min_idx = this_idx

        ## Checking if the closest waypoint is farther down the road than the car
        # Vector from the closest waypoint to the next waypoint along the road (a vector of the road direction)
        road_dir = [self.base_waypoints[(min_idx+1)%(self.base_waypoints_size - 1)].pose.pose.position.x - self.base_waypoints[min_idx].pose.pose.position.x, self.base_waypoints[(min_idx+1)%(self.base_waypoints_size - 1)].pose.pose.position.y - self.base_waypoints[min_idx].pose.pose.position.y]
        # Vector from the closest waypoint to the car's location
        wp_to_car = [self.current_position.position.x - self.base_waypoints[min_idx].pose.pose.position.x, self.current_position.position.y - self.base_waypoints[min_idx].pose.pose.position.y]	
        
        # If the dot product is less than 0, then the closest waypoint is ahead of the car
        check_dir = np.dot(road_dir, wp_to_car)	
        if (check_dir >= 0):
            min_idx = (min_idx + 1)%(self.base_waypoints_size - 1)


        ## Saving this position so we can start searching from this point for the next minimum distance
        self.current_position_idx = min_idx

        ## Chopping out the next LOOKAHEAD_WPS number of waypoints, accounting for wraparound
        if (min_idx + LOOKAHEAD_WPS < len(self.base_waypoints)):
        	self.final_waypoints = self.base_waypoints[min_idx:(min_idx + LOOKAHEAD_WPS + 1)]
        else:
        	self.final_waypoints = self.base_waypoints[min_idx:] + self.base_waypoints[:(min_idx+LOOKAHEAD_WPS - len(self.base_waypoints))] 
        
        ## Update velocity per waypoint
        self.velocity_update(self.final_waypoints)

        ## Generate the message with the list of waypoints to be followed
        next_points_msg = Lane()
        next_points_msg.header.frame_id = '/world'
        next_points_msg.header.stamp = rospy.Time(0)
        next_points_msg.waypoints = self.final_waypoints

        # Publish the list of waypoints to be followed
        self.final_waypoints_pub.publish(next_points_msg)
   
    def velocity_cb(self, msg):
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        self.current_velocity = math.sqrt(x*x + y*y)
        #rospy.logwarn("Vecolity is {}".format(self.current_velocity))
    
    def waypoints_cb(self, waypoints):
        # Grab the list of all waypoints from the base_waypoints message
        self.base_waypoints = waypoints.waypoints
        self.base_waypoints_size = len(self.base_waypoints)
        self.sub_base_waypoints.unregister()
        

    def traffic_cb(self, msg):
        # Read the red light detection and traffic light point
        if(msg.data >=0):
            self.traffic_point_idx = msg.data
            self.red_light_detected = True
        else:
            self.traffic_point_idx = None
            self.red_light_detected = False
            

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_update(self, next_points):
        # Check traffic light
        if self.red_light_detected:
            # RED light
            distance_idx = self.traffic_point_idx - self.current_position_idx
            if distance_idx > 1 & distance_idx < LOOKAHEAD_WPS:
                # Distance from current position to traffic light position
                distance_to_tl = self.distance(next_points, 0, distance_idx)
                # Begin to deceleration
                if distance_to_tl < DISTANCE_DECELERATION:
                    # Deceleration limited with max deceleration
                    decel = min(self.current_velocity**2 / (2 * distance_to_tl), -self.decel_limit)
                    #rospy.logwarn("Deceleration... {}".format(decel))
                    # Velocity for next points
                    for idx in range(len(next_points)):
                        dist = self.distance(next_points, 0, idx)
                        velocity2 = self.current_velocity**2 - 2 * decel * dist
                        velocity = math.sqrt(max(velocity2, 0.0))
                        self.set_waypoint_velocity(next_points, idx, velocity)
                # STOP before red light
                elif distance_to_tl < DISTANCE_STOP:
                    #rospy.logwarn("STOP....")
                    velocity = 0.0
                    for idx in range(len(next_points)):
                        self.set_waypoint_velocity(next_points, idx, velocity)
                # Keep current velocity
                else:
                    #rospy.logwarn("Keep Speed...")
                    velocity = self.current_velocity
                    for idx in range(len(next_points)):
                        self.set_waypoint_velocity(next_points, idx, velocity)
        else:
            #rospy.logwarn("Accerlation...")
            velocity = self.cruise_speed
            for idx in range(len(next_points)):
                self.set_waypoint_velocity(next_points, idx, velocity)

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
        return abs(math.sqrt((x2 - x1)**2 + (y2 - y1)**2))

    def kpm_to_mps(self, kmp):
        return 0.278 * kmp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
