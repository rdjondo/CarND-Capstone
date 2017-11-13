#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
import tf

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.path_final_waypoints_pub = rospy.Publisher("/path_final_waypoints", Path, queue_size=10)
        self.path_map_pub = rospy.Publisher("/path_map", Path, queue_size=10)

        # TODO: Add other member variables you need below
	self.waypoints = []
        self.transform = None

	self.current_pose = PoseStamped()
	self.current_pose_id = -1
	self.final_waypoints = Lane()

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
	self.current_pose = msg

        # transformation base_footprint to world
        br = tf.TransformBroadcaster()
        br.sendTransform((self.current_pose.pose.position.x, self.current_pose.pose.position.y, 0),
                         (self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w),
                        rospy.Time.now(), "base_footprint", "world")


	# If no previous map matching --> whole map, else --> only short area backwards and forwards
        if self.current_pose_id == -1: 
	  id_start = 0
	  id_end = len(self.waypoints)
        else:
	  id_start_tmp = self.current_pose_id - 5
	  id_end_tmp = self.current_pose_id + 5
	  if ( id_start_tmp < 0 ):
	    id_start_tml = len(self.waypoints) - id_start_tmp

	  if ( id_end_tmp > len(self.waypoints) ):
	    id_end_tmp = id_end_tmp - len(self.waypoints)

	  id_start = min(id_start_tmp, id_end_tmp)
	  id_end   = max(id_start_tmp, id_end_tmp)
      
	# search of for closest waypoint on the map    
	closest_distance = 1000000
	closest_waypoint_id = 0
	valid_waypoint = False
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(id_start,id_end):
            distance= dl(self.waypoints[i].pose.pose.position, self.current_pose.pose.position)
	    if distance < closest_distance:
	      closest_distance = distance
              self.current_pose_id = i
	      valid_waypoint = True


        # if map matching is ok --> publish final waypoints adn path (RVIZ visualization)
        if valid_waypoint:
          #print "valid point"
 	  #print "pose x: " + str(self.current_pose.pose.position.x) + " y: " + str(self.current_pose.pose.position.y)
          #print "waypoint x: " + str(self.waypoints[self.current_pose_id].pose.pose.position.x) + " y: " + str(self.waypoints[self.current_pose_id].pose.pose.position.y)

          path = Path()
          path.header.frame_id = "world"
          path.header.stamp = rospy.Time()


          self.final_waypoints = Lane()
	  self.final_waypoints.header.frame_id = "world"

  	  id_waypoint = self.current_pose_id
	  for i in range(0,LOOKAHEAD_WPS):
	    id_waypoint = id_waypoint + 1;
	    if ( id_waypoint >= len(self.waypoints)):
	      id_waypoint = 1

	    waypoint = self.waypoints[id_waypoint]
	    self.final_waypoints.waypoints.append(waypoint)

            # Path
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = waypoint.pose.pose.position.x
            pose.pose.position.y = waypoint.pose.pose.position.y
            path.poses.append(pose)



	  self.final_waypoints_pub.publish(self.final_waypoints)
          self.path_final_waypoints_pub.publish(path)


        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	self.waypoints = waypoints.waypoints


	


        pass

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

