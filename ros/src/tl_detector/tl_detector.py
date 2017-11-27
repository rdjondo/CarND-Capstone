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

STATE_COUNT_THRESHOLD = 1

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
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb,  queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb,  queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config['running_in_simulator'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
            
        #self.ros_sleep = rospy.Rate(25)

        rospy.spin()
    

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

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

	#rospy.loginfo ('TLD: process traffic lights return ({}, {})'.format (light_wp,state))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        try:
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
            #self.ros_sleep.sleep()
        except AttributeError:
            pass


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
	# resue code from src/waypoint_updater/waypoint_updater.py
	if (self.waypoints!=None):
	    id_start = 0
	    id_end = len(self.waypoints)

            # search of for closest waypoint on the map    
	    closest_distance = 1000000
	    closest_waypoint_id = 0
	    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

	    for i in range(id_start,id_end):
                distance= dl(self.waypoints[i].pose.pose.position, pose.position)
	        #rospy.loginfo('TLD GCW distance: {}'.format(distance))

	        if distance < closest_distance:
	    	    closest_distance = distance
            	    closest_waypoint_id = i
   	            #rospy.loginfo('TLD GCW update: pose:({}, {}) xi:{} dist:{}'.format(pose.position.x,pose.position.y,i,distance))

            return closest_waypoint_id
	else:
	    return -1

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        try:
            light_id = self.light_classifier.get_classification(cv_image)
        except AttributeError:
            light_id = -1 
        return light_id

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = 100000 # global way-point index of 1st TL's stopping line

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
	else:
	    return -1, TrafficLight.UNKNOWN

	# Find the waypoint index of the closest front traffic light stop line to the car.
	for stop_line_position in stop_line_positions:
            # create a Pose given the stop line position
	    stop_line_pose = Pose()
	    stop_line_pose.position.x = stop_line_position[0]
            stop_line_pose.position.y = stop_line_position[1]
            stop_line_wp = self.get_closest_waypoint(stop_line_pose)

	    # update only when front stop line is closer in comparison to the old one.
	    # keep seeing traffic light a few waypoints behind
	    SEE_BEHIND_WAYPOINTS = 2
	    if (stop_line_wp + SEE_BEHIND_WAYPOINTS >= car_position and stop_line_wp < light):
	        light = stop_line_wp


        #if (light and light - car_position >= 200): # Only check the traffic light color when the car is close enough 
        if light:
	    state = self.get_light_state(light)
	    rospy.loginfo('TLD: Car position waypoints index: {}'.format(car_position))
	    rospy.loginfo('TLD: Closest front stop line waypoints index: {}'.format(light))
	    rospy.loginfo('TLD: Detected light status: {}'.format(state))
            return light, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
