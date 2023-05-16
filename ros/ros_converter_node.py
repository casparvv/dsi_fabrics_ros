#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Pose2D, Twist
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float64MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
import std_msgs

class ActionConverterNode(object):
    def __init__(self, dt, rate_int):
        rospy.init_node("ActionConverter", anonymous=True)
        self._rate = rospy.Rate(rate_int)
        self._dt = dt

        self._n = 3
        self._nu = 2
        self._actionIndices = [0, 1]
        self._stateIndices = [0, 1, 2]
        self._qdotIndices = [3, 4]
        self._joint_state_sub = rospy.Subscriber("/joint_states_filtered", JointState, self.joint_state_cb)
        self._point_cloud_sub = rospy.Subscriber("/scan_pointcloud", PointCloud2, self.point_cloud_cb)
        #self._acc_pub = rospy.Publisher(
        #    '/joint_acc_des',
        #    Float64MultiArray, queue_size=100
        #)
        self._vel_pub = rospy.Publisher(
                '/cmd_vel',
                Twist, queue_size=10
        )
        self._stop_pub = rospy.Publisher(
            '/motion_stop_request',
            Bool, queue_size=10
        )
        self._x = np.zeros(self._n)
        self._xdot = np.zeros(self._n)
        self._obstacles = np.zeros((3, 2))
        #self._acc_msg = Float64MultiArray()
        self._vel_msg = Twist()

    def joint_state_cb(self, data):
        self._x = np.array([data.position[i] for i in self._stateIndices])
        self._xdot = np.array([data.velocity[i] for i in self._stateIndices])

    def point_cloud_cb(self, data):
        point_array = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        point_cloud_xyz = np.array(list(point_array))
        point_cloud_xyz[np.isinf(point_cloud_xyz)] = 16.0
        self._obstacles = point_cloud_xyz

    def ob(self):
        return {'x': self._x, 'xdot': self._xdot, 'obs': self._obstacles}, rospy.get_time()

    def publishAction(self, action):
        self._vel_msg = Twist()
        action = action*0.1
        self._vel_msg.linear.x = action[0]
        self._vel_msg.linear.y = action[1]
        print(action) 
        self._vel_pub.publish(self._vel_msg)
        self._rate.sleep()
        return self.ob()

    def stopMotion(self):
        rospy.loginfo("Stopping ros converter")
        stop_msg = std_msgs.msg.Bool(data=False)
        for i in range(10):
            rospy.loginfo("Stoping node")
            self._stop_pub.publish(stop_msg)
            self._rate.sleep()

