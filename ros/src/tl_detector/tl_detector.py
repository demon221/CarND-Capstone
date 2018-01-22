#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
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

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

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

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement DONE
        if self.waypoints is None:
            return
        
        # Extract given position info
        given_x = pose.position.x
        given_y = pose.position.y
        
        # Initialize search
        closest_wp_dist = float('inf')
        closest_wp_idx = -1
        
        # Exhaustive search
        for idx, wp in enumerate (self.waypoints.waypoints):
            x_squared = (wp.pose.pose.position.x - given_x)**2
            y_squared = (wp.pose.pose.position.y - given_y)**2
            distance = math.sqrt(x_squared + y_squared)
            
            if (distance < closest_wp_dist):
                closest_wp_dist = distance
                closest_wp_idx = idx

        return closest_wp_idx

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
        # Hyper-Parameters
        max_visible_distance = 60   # Anything further is too pay attention to
        
        # Initialize
        waypoint_idx = None
        light_color  = TrafficLight.UNKNOWN
        
        # Exit early if no pose info
        if(self.pose):
            ego_position = self.pose.pose.position
            ego_orientation = self.pose.pose.orientation
        else:
            return waypoint_idx, light_color
        
        # Find closest traffic_light stop_line that's ahead of the ego vehicle
        stop_line_positions = self.config['stop_line_positions']
        
        closest_dist = max_visible_distance
        closest_idx = -1     # Negative value indicates no light found
        
        # Iterate through all stop line positions
        for i, line_loc in enumerate(stop_line_positions):
            # Get stop line location
            line_x = line_loc[0]
            line_y = line_loc[1]
            
            # Calculate distance to line
            x_squared = (ego_position.x - line_x)**2
            y_squared = (ego_position.y - line_y)**2
            line_distance = math.sqrt(x_squared + y_squared)
            
            # Check if current line is closer than the previous closest line
            if line_distance < closest_dist:

                # Check if line is ahead of vehicle.
                # The inner product of the car's heading (yaw) and a vector 
                # between the car and the stop line will be positive if the 
                # line is ahead of the vehicle. Yaw angle transform taken from
                # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
                quaternion = (
                    ego_orientation.x,
                    ego_orientation.y,
                    ego_orientation.z,
                    ego_orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                yaw = euler[2]
                
                yaw_vec_x = math.cos(yaw)
                yaw_vec_y = math.sin(yaw)
                line_vec_x = line_x - ego_position.x
                line_vec_y = line_y - ego_position.y
                inner_prod = yaw_vec_x*line_vec_x + yaw_vec_y*line_vec_y
                
                if inner_prod > 0:
                    # Update closest line info
                    closest_x = line_x
                    closest_y = line_y
                    closest_idx = i
                    closest_dist = line_distance
                    
        # Process light (if we found one)
        if closest_idx >= 0:
            line_pose = Pose()
            line_pose.position.x = closest_x
            line_pose.position.y = closest_y
            waypoint_idx = self.get_closest_waypoint(line_pose)
            closest_light = self.lights[closest_idx]
            # FIXME Temporary ground truth passthrough while get_classification method is in development.
            light_color = closest_light.state
            # light_color = get_light_state(closest_light)
            print ("Light Waypoint: ", waypoint_idx, " Color: ", light_color)
            
        return waypoint_idx, light_color

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
