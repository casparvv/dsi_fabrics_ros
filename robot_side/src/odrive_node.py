#!/usr/bin/env python
from __future__ import print_function

import rospy
import numpy as np
import tf_conversions

from nav_msgs.msg import Odometry
import std_srvs.srv

class ODriveNode(object):
    """
    Only the odometry publish function for reference.
    """
    def pub_odometry(self, time_now):
        """
        Calculate odometry from wheel motions using eqs. 13.8, 13.10, 13.33, 13.35, and 13.36 from Modern Robotics [1].
            
        The encoders output the 'vel_estimate_counts' variable which is the estimate of the linear velocity of an axis, in counts/s [2].
        Using this variable, the angular change per wheel is calculated.
        The body twist is calculated using the kinematics of a four-mecanum-wheel robot and the angular changes of the wheels.
        Finally, the body twist is transformed into a fixed frame and the odometry data is published.
       
        Most variables are named to match the eqs. in [1].

            [1] - May 2017 preprint of Modern Robotics, Lynch and Park, Chapter 13, Cambridge U. Press, 2017.
            [2] - https://docs.odriverobotics.com/v/0.5.4/fibre_types/com_odriverobotics_ODrive.html#ODrive.Encoder.vel_estimate
        """
        dt = 0.1                            # Time step
        l = self.wheel_base_front_rear/2    # Length: center robot - wheel axis
        w = self.wheel_base_width/2         # Width: center robot - center wheel
        gear_ratio = 3.83                   # Calculated empirically
        encoder_count_per_rev = 4000        # Total encoder counts per revolution
        
        vel_estimate_counts = np.array([self.controllers.get('data')[0]['vel_l'],
                                        self.controllers.get('data')[0]['vel_r'],
                                        self.controllers.get('data')[1]['vel_r'],
                                        self.controllers.get('data')[1]['vel_l']])
        
        delta_theta = vel_estimate_counts*2*np.pi/(gear_ratio*encoder_count_per_rev)   # rad/s

        F = self.wheel_radius/4 * np.array([[-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)],
                                            [         1,         1,         1,          1],
                                            [        -1,         1,        -1,          1]])

        V_b = np.dot(F, delta_theta)        # Chassis body twist (13.33)
        V_b_dt = V_b*dt
        wbz = V_b_dt[0]
        vbx = V_b_dt[1]
        vby = V_b_dt[2]

        if wbz == 0.0:
            delta_q_b = np.array([0, vbx, vby])
        else:
            delta_q_b = np.array([                                          wbz,
                                  (vbx*np.sin(wbz) + vby*(np.cos(wbz) - 1))/wbz,
                                  (vby*np.sin(wbz) + vbx*(1 - np.cos(wbz)))/wbz])
       
        self.phi += wbz
        self.phi = self.phi % (2*np.pi)     # Keep the chassis angle between 0 - 2pi
        
        # Transform with rotation matrix around z-axis using the chassis angle phi
        R = np.array([[1,                0,                 0],
                      [0, np.cos(self.phi), -np.sin(self.phi)],
                      [0, np.sin(self.phi),  np.cos(self.phi)]])
        delta_q = np.dot(R, delta_q_b)
        self.q += delta_q
        self.q[0] = self.q[0] % (2*np.pi)
       
        # Fill in the odometry message and publish
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.q[1]
        self.odom_msg.pose.pose.position.y = self.q[2]
        quat = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.phi)
        self.odom_msg.pose.pose.orientation.z = quat[2]
        self.odom_msg.pose.pose.orientation.w = quat[3]
        self.odom_msg.twist.twist.linear.x = V_b[1]
        self.odom_msg.twist.twist.linear.y = V_b[2]
        self.odom_msg.twist.twist.angular.z = V_b[0]
    
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.transform.translation.x = self.q[1]
        self.tf_msg.transform.translation.y = self.q[2]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        self.odom_publisher.publish(self.odom_msg)
        self.tf_publisher.sendTransform(self.tf_msg)

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass

