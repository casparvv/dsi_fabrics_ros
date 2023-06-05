#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

def laser_cb(laserscan):
    """
    Calculates 3D points from LaserScan data. 
     Subscribes to '/scan', converts to 3D points and then to a PointCloud2.
     PointCloud2 is published to '/scan_pointcloud'.

    Use range_step to filter sensor data.
     Set to 1 to use all data points, 10 for only every 10th data point, etc.
    """
    point_dimension = 3
    range_step = 40
    num_ranges = len(laserscan.ranges)
    start_angle = laserscan.angle_min
    angle_increment = laserscan.angle_increment

    # Calculate 3D points from the laserscan ranges and angles
    points = np.zeros((round(num_ranges/range_step), point_dimension))
    for i in range(0, num_ranges, range_step):
        r = laserscan.ranges[i]
        angle = start_angle + i * angle_increment
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        z = 0.2
        points[int(i/range_step)] = x, y, z

    # Define header for pointcloud
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser'

    # Set the pointcloud elements
    pointcloud = PointCloud2()
    pointcloud.header = header
    pointcloud.height = 1
    pointcloud.width = len(points)
    pointcloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    pointcloud.is_bigendian = False
    pointcloud.point_step = 12
    pointcloud.row_step = pointcloud.point_step * pointcloud.width
    pointcloud.is_dense = True
    pointcloud.data = points.astype(np.float32).tobytes()

    pub.publish(pointcloud)
    

if __name__ == "__main__":
    # Initialize ROS node and create LaserScan subscriber and PointCloud publisher.
    rospy.init_node('laser_scan_processor')
    sub = rospy.Subscriber('/scan', LaserScan, laser_cb)
    pub = rospy.Publisher('/scan_pointcloud', PointCloud2, queue_size=10)

    rospy.spin()

