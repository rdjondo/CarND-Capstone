#!/usr/bin/env python


import sys
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from copy import deepcopy

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)


        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        rospy.Subscriber('/max_velocity', Float64, self.max_velocity_cb, queue_size=1)


        # Acceleration values
        self.a_max = rospy.get_param("~A_MAX", 4)
        self.a_min = 1.0

        self.offset_id_stop_point = rospy.get_param("~offset_id_stop_point", 2)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.path_final_waypoints_pub = rospy.Publisher("/path_final_waypoints", Path, queue_size=10)
        self.path_map_pub = rospy.Publisher("/path_map", Path, queue_size=10)

        self.viz_velocity_marker_pub = rospy.Publisher("/viz_velocity", MarkerArray, queue_size=1000)
        self.viz_traffic_light_marker_pub = rospy.Publisher("/viz_waypoint_updater", Marker, queue_size=10)

        # TODO: Add other member variables you need below
        self.waypoints = []
        self.transform = None


        self.set_speed_mps = 0.0
        rospy.loginfo("Waypoint_updater - set_speed: %f [m/s]", self.set_speed_mps)

        self.current_pose = PoseStamped()
        self.current_pose_id = -1
        self.current_velocity = TwistStamped()
        self.final_waypoints = Lane()
        self.traffic_waypoint_idx = -1


        self.END_SPEED_PLAN_OFFSET = 50 

        self.base_frame = "base_footprint"
        self.map_frame = "world"

        rospy.spin()


    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg

        # transformation base_footprint to world
        br = tf.TransformBroadcaster()
        br.sendTransform((self.current_pose.pose.position.x, self.current_pose.pose.position.y, 0),
                         (self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w),
                        rospy.Time.now(), self.base_frame, self.map_frame)



	# If no previous map matching --> whole map, else --> only short area backwards and forwards
        if self.current_pose_id == -1: 
            id_start = 0
            id_end = len(self.waypoints)
        else:
            id_start_tmp = self.current_pose_id - 5
            id_end_tmp = self.current_pose_id + 5
            if ( id_start_tmp < 0 ):
                id_start_tmp = len(self.waypoints) - id_start_tmp

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
            # Defensive test
            if i>=len(self.waypoints): return

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
          #print self.current_pose_id

          path = Path()
          path.header.frame_id = "world"
          path.header.stamp = rospy.Time()


          self.final_waypoints = Lane()
          self.final_waypoints.header.frame_id = "world"

          id_waypoint = self.current_pose_id
          for i in range(0,LOOKAHEAD_WPS):
            id_waypoint = id_waypoint + 1
            if ( id_waypoint >= len(self.waypoints)):
              id_waypoint = 1

            # modify vehicle speed if required via launch file
            self.waypoints[id_waypoint].twist.twist.linear.x = self.set_speed_mps


            #print self.waypoints[id_waypoint]
            waypoint = self.waypoints[id_waypoint]
            self.final_waypoints.waypoints.append(waypoint)

            # Path
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = waypoint.pose.pose.position.x
            pose.pose.position.y = waypoint.pose.pose.position.y
            path.poses.append(pose)


          if self.traffic_waypoint_idx > 0:
            #print "traffic light active"
            self.update_final_waypoints_tl()
          else:
            #print "no valid traffic light index"
            ter = 1

          self.visualize_velocity_vector()

          self.final_waypoints_pub.publish(self.final_waypoints)
          self.path_final_waypoints_pub.publish(path)


        pass

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint_idx = msg.data

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "traffic_light_stop_pose"
        marker.id = 0
        
        if self.traffic_waypoint_idx > 0 and self.traffic_waypoint_idx < len(self.waypoints):
          marker.type = Marker.SPHERE
          marker.action = Marker.ADD
          marker.pose = self.waypoints[self.traffic_waypoint_idx].pose.pose
          marker.scale.x = 5
          marker.scale.y = 5
          marker.scale.z = 0.2
          marker.color.a = 1.0
          marker.color.r = 1.0
          marker.color.g = 0.0
          marker.color.b = 0.0

        elif self.traffic_waypoint_idx == -1:
          marker.action = Marker.DELETE
          #rospy.loginfo ('WP_U: No red light detected. traffic_waypoint_idx : {}, len(waypoints) : {})'.format (self.traffic_waypoint_idx, len(self.waypoints)) ) 

        else:
          marker.action = Marker.DELETE
          #rospy.loginfo ('WP_U: Index invalid. traffic_waypoint_idx : {}, len(waypoints) : {})'.format (self.traffic_waypoint_idx, len(self.waypoints)) )
  
        self.viz_traffic_light_marker_pub.publish(marker)
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, msg):
        self.current_velocity = msg

    def max_velocity_cb(self, msg):
        self.set_speed_mps = float(msg.data)
        rospy.logwarn('MAX velocity is : {}'.format(self.set_speed_mps) )

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp2 = max( min(wp2, len(waypoints)-1) , 0) #Defensive indexing
        for i in range(wp1, wp2+1):
            pos1 = waypoints[wp1].pose.pose.position
            if not ( 0 < i < len(waypoints) ):
                rospy.logwarn('Waypoint Index is out of range i:{}'.format(i))
            pos2 = waypoints[i].pose.pose.position
            dist += dl(pos1, pos2)
            wp1 = i
        return dist


    # update vehicle trajectory if a traffic light is read
    def update_final_waypoints_tl(self):

      v_ego = self.current_velocity.twist.linear.x

      # calculate distance to traffic light
      distance_to_traffic_light = self.distance(self.waypoints, self.current_pose_id, self.traffic_waypoint_idx-self.offset_id_stop_point)
      points_to_stop = self.traffic_waypoint_idx - self.current_pose_id



      required_distance_to_stop = v_ego*v_ego/(2*self.a_min)

      #rospy.loginfo("distance_to_traffic_light: {}, required_distance_to_stop: {} ".format(distance_to_traffic_light, required_distance_to_stop))

      # car stops, if the stop distance is smaller than the distance to the traffic light
      if distance_to_traffic_light < required_distance_to_stop:
        v_reduction_per_m = v_ego / required_distance_to_stop if required_distance_to_stop > 1e-1 else 0.0

        v_tmp = v_ego
        summe = 0
        #print "brake"
        #for i in range(self.current_pose_id, self.traffic_waypoint_idx-self.offset_id_stop_point):
        for i in range(self.current_pose_id, self.current_pose_id + self.END_SPEED_PLAN_OFFSET):
          if i<self.traffic_waypoint_idx-self.offset_id_stop_point:
            distance_to_next_waypoint = self.distance(self.waypoints, i, i+1)   
            summe = summe +  distance_to_next_waypoint
            v_segment = v_reduction_per_m * distance_to_next_waypoint
            v_tmp = v_tmp - v_segment
          else:
            v_tmp = 0.0
          #print str(i) + ": distance: " + str(distance_to_next_waypoint) +  " v: " + str(v_tmp)  
          self.set_waypoint_velocity(self.waypoints, i, v_tmp)

        #print summe  
      return self.waypoints

    def visualize_velocity_vector(self):
      marker_array = MarkerArray()
      marker = Marker()
      marker.header.frame_id = self.map_frame
      marker.header.stamp = rospy.Time.now()
      marker.ns = "velocity_vector"
      marker.id = 0;
      marker.pose.orientation.w = 1.0

      for i in range(0, self.END_SPEED_PLAN_OFFSET, 2):
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        point = deepcopy(self.final_waypoints.waypoints[i].pose.pose.position)
        point.z = point.z + self.get_waypoint_velocity(self.final_waypoints.waypoints[i]) / 6.0
        marker.points.append(point)
        marker_array.markers.append(deepcopy(marker))

      self.viz_velocity_marker_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

