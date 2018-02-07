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

    def get_light_state(self, light, azimuth):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For use_ground_truth, True means the light state is taken from 
        # simulator ROS messages.
        # 
        # False calls light_classifier.get_classification() with a close 
        # cropped color image of the light to be classified.
        use_ground_truth = True
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        ##### Grab a Region Of Interest in the input image that includes just 
        # the traffic light using camera intrinsics and projective geometry. 
        # These numbers were determined experimentally from observation of the 
        # simulator and will need to be changed for different environments.
        # Baseline assumption is that the simulator camera has no distortion.
        
        # 3D space object information
        car_x = self.pose.pose.position.x
        car_y = self.pose.pose.position.y
        car_z = self.pose.pose.position.z
        
        light_x = light.pose.pose.position.x
        light_y = light.pose.pose.position.y
        light_z = light.pose.pose.position.z

        light_v = 2.0
        light_h = 0.89
        
        x_squared = (car_x - light_x)**2
        y_squared = (car_y - light_y)**2
        xy_distance = math.sqrt(x_squared + y_squared)
        
        # Project vertical locations into pixel space
        img_ctr_y = 618.5
        ifov_v = .0003626794     # radians/pixel
        
        light_top = img_ctr_y - math.atan(light_z/xy_distance)/ifov_v
        light_bottom = img_ctr_y - math.atan((light_z-light_v)/xy_distance)/ifov_v
        light_height = light_bottom - light_top
        
        # Add some space above and below for region of ineterest
        # NOTE: (0,0) is the top-leftmost pixel in the image
        y_margin = light_height/4
        ROI_top = int(math.floor(light_top - y_margin))
        ROI_bottom = int(1 + math.floor(light_bottom + y_margin))
        
        
        # Project horizontal locations into pixel space
        img_ctr_x = 365.2
        ifov_h = 0.0003853372     # radians/pixel
        light_ctr_x = img_ctr_x - (azimuth/ifov_h)
        light_width = math.atan(light_h/xy_distance)/ifov_h
        x_margin = 0.35*light_width
        ROI_left = int(math.floor(light_ctr_x - 0.5*light_width - x_margin))
        ROI_right = int(1 + math.floor(light_ctr_x + 0.5*light_width + x_margin))
        
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cropped_image = cv_image[ROI_top:ROI_bottom, ROI_left:ROI_right]

        #Get classification
        light_state  = TrafficLight.UNKNOWN
        if use_ground_truth is True:
            light_state = light.state
            source_text = "ground truth message"
        else:
            light_state = self.light_classifier.get_classification(cropped_image)
            source_text = "classifier"
            
        state_text = "UNKNOWN"
        if light_state == TrafficLight.RED:
            state_text = "RED"
        if light_state == TrafficLight.GREEN:
            state_text = "GREEN"
        if light_state == TrafficLight.YELLOW:
            state_text = "YELLOW"
        # print "Light state from", source_text, "is", state_text

        return light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Anything further is too far away to consider
        max_visible_distance = 120   

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
                    light_x = self.lights[i].pose.pose.position.x
                    light_y = self.lights[i].pose.pose.position.y
                    light_vec_x = light_x - ego_position.x
                    light_vec_y = light_y - ego_position.y
                    azimuth = math.atan2(light_vec_y, light_vec_x) - math.atan2(yaw_vec_y, yaw_vec_x)
                    
        # Process light (if we found one)
        if closest_idx >= 0:
            line_pose = Pose()
            line_pose.position.x = closest_x
            line_pose.position.y = closest_y
            waypoint_idx = self.get_closest_waypoint(line_pose)
            closest_light = self.lights[closest_idx]
            light_color = self.get_light_state(closest_light, azimuth)
            
        return waypoint_idx, light_color

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
