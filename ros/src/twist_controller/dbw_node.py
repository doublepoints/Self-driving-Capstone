#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from twist_controller import Controller

PUBLISHING_RATE = 50
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class VehicleParams(object):
    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None
        self.total_vehicle_mass = None


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        carla_params = VehicleParams()
        carla_params.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        carla_params.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        carla_params.brake_deadband = rospy.get_param('~brake_deadband', .1)
        carla_params.decel_limit = rospy.get_param('~decel_limit', -5)
        carla_params.accel_limit = rospy.get_param('~accel_limit', 1.)
        carla_params.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        carla_params.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        carla_params.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        carla_params.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        carla_params.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        carla_params.total_vehicle_mass = carla_params.vehicle_mass + carla_params.fuel_capacity * GAS_DENSITY

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.controller = Controller(vehicle_params=carla_params)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        self.current_vel = None
        self.curr_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.pose = None
        self.final_waypoints_2d = None
        self.throttle = self.steering = self.brake = 0

        self.loop()

    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)  # 50Hz
        while not rospy.is_shutdown():

            if not None in (self.current_vel, self.linear_vel, self.angular_vel):

                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                   self.dbw_enabled,
                                                                                   self.linear_vel,
                                                                                   self.angular_vel)

            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg

    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def final_waypoints_cb(self, msg):
        final_waypoints = msg.waypoints
        self.final_waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                   final_waypoints]

    def pose_cb(self, msg):
        self.pose = msg

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
