#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

GAS_DENSITY = 2.858

from twist_controller import Controller

'''
This node is responsible for controlling the throttle, braking, and steering in high-level. 

Important factors that affect the decision making are:

dbw_enabled : The status of whether the driving mode is auto or manual. Attention must be given to the fact that, when
not enabled, our PID controller can accumulate error because the car may be driven by a human instead of our controller.
Therefore an error resetting process when switching from manual to automatic was considered.

brake_deadband: the range where breaking is ignored

The total mass of the car is used in calculating the breaking torque
'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # parameters
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

        # follow Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # initial parameters of the `TwistController` object
        self.dbw_enabled = False
        self.dbw_switch_on = False
        self.current_velocity = None
        self.target_velocity = None
        self.target_angular = None
        min_speed = 0.
        total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY

        # initialization of the object
        self.controller = Controller (wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                                      decel_limit, accel_limit,wheel_radius, brake_deadband, total_mass)

        # subscribe to all the needed topics
        rospy.Subscriber('vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cur_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)

        self.loop()

    def dbw_enabled_cb(self, message):

        # extract the dbw_enabled status
        self.dbw_enabled = bool(message.data)

    def cur_velocity_cb(self, message):

        # extract the current velocity
        self.current_velocity = message.twist.linear.x

    def twist_cmd_cb(self, message):

        # extract the target linear and angular velocities
        self.target_velocity  = message.twist.linear.x
        # print("target velocity1", self.target_velocity)
        self.target_angular = message.twist.angular.z
        # print("target angular1", self.target_angular)

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            current_timestamp = rospy.get_time()

            # get predicted throttle, brake, and steering using `twist_controller`

            # waiting to get values from nodes
            if self.target_velocity is None or self.current_velocity is None or self.target_angular is None:
                continue

            # we have the needed values
            # print("target velocity2", self.target_velocity)
            # print("current velocity2", self.current_velocity)
            # print("target angular2", self.target_velocity)

            # calculate the throttle, brake and steering values
            throttle, brake, steer = self.controller.control(self.target_velocity, self.target_angular,
                                                             self.current_velocity,current_timestamp)



            # drive-by-wire is enabled, so we publish the commands
            if self.dbw_enabled is True:
                if self.dbw_switch_on == False:
                    self.dbw_switch_on = True
                    self.controller.reset()
                self.publish(throttle, brake, steer)
            
            # Sleep until next execution step
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
    try:
        DBWNode()
    except rospy.ROSInterruptException:
        pass