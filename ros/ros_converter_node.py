#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import std_msgs
from tf.transformations import euler_from_quaternion
import tf.transformations as tf

class ActionConverterNode(object):
    def __init__(self, dt, rate_int):
        rospy.init_node("ActionConverter", anonymous=True)
        self._rate = rospy.Rate(rate_int)
        self._dt = dt
        self._n = 2
        self._odom_sub = rospy.Subscriber("/optitrack_state_estimator/Heijn/state", Odometry, self.odometry_cb, queue_size=1)
        self._point_cloud_sub = rospy.Subscriber("/scan_pointcloud", PointCloud2, self.point_cloud_cb)
        self._vel_pub = rospy.Publisher(
                '/cmd_vel',
                Twist, queue_size=1
        )
        self._stop_pub = rospy.Publisher(
            '/motion_stop_request',
            Bool, queue_size=10
        )
        self._x = np.zeros(self._n)
        self._xdot = np.zeros(self._n)
        self._yaw = np.zeros(1)
        self._obstacles = np.zeros((3, 2))
        self._vel_msg = Twist()

    def odometry_cb(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        xdot = data.twist.twist.linear.x
        ydot = data.twist.twist.linear.y
        self._x = np.array([x, y])
        self._xdot = np.array([xdot, ydot])
        
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        self._yaw = euler_angles[2]

    def point_cloud_cb(self, data):
        translation_matrix = tf.translation_matrix([self._x[0], self._x[1], 0.0])
        rotation_matrix = tf.rotation_matrix(self._yaw, (0, 0, 1))
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)
    
        point_array = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        point_cloud_xyz = np.array(list(point_array))
        point_cloud_xyz[np.isinf(point_cloud_xyz)] = 16.0
        
        transformed_point_cloud = np.hstack((point_cloud_xyz, np.ones((point_cloud_xyz.shape[0], 1))))
        transformed_point_cloud = np.dot(transformation_matrix, transformed_point_cloud.T).T
        transformed_point_cloud = transformed_point_cloud[:, :3]

        self._obstacles = transformed_point_cloud

    def ob(self):
        return {'x': self._x, 'xdot': self._xdot, 'obs': self._obstacles}, rospy.get_time()

    def publishAction(self, action):
        """
        Currently rotating the action according to the robot's yaw.
        """
        self._vel_msg = Twist()
        
        angle_rad =  -self._yaw
        cos_theta = np.cos(angle_rad)
        sin_theta = np.sin(angle_rad)
        rotation_matrix = np.array([[cos_theta, -sin_theta],
                                    [sin_theta, cos_theta]])
        action = np.dot(rotation_matrix, action)
        action = action*0.1
        self._vel_msg.linear.x = action[0]
        self._vel_msg.linear.y = action[1]
        self._vel_pub.publish(self._vel_msg)
        self._rate.sleep()
        return self.ob()

    def stopMotion(self):
        """
        To do: test the stopMotion function.
        """
        rospy.loginfo("Stopping ros converter")
        stop_msg = std_msgs.msg.Bool(data=False)
        for i in range(10):
            rospy.loginfo("Stoping node")
            self._stop_pub.publish(stop_msg)
            self._rate.sleep()

