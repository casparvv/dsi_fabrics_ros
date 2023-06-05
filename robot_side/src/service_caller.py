#!/usr/bin/env python
"""
This script adjusts the permissions of the usb port for the LiDAR sensor.
Then it calls the connect to odrive driver  and calibrate odrive driver services.
Sometimes part of the launch file fails, run again to start all.
The calibrate motors service call can be commented after being calibrated,
 to avoid the noise (only required after restart).
"""

import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

def call_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        trigger_service = rospy.ServiceProxy(service_name, Trigger)
        request = TriggerRequest()
        response = trigger_service(request)
        rospy.loginfo(f"Service call to {service_name} succeeded.")
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to {service_name} failed: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('service_caller')

    # Change permissions of /dev/ttyUSB0 for the LiDAR
    subprocess.call(['sudo', 'chmod', '666', '/dev/ttyUSB0'])

    # Call /odrive/connect_driver service
    call_service('/odrive/connect_driver')

    # Call /odrive/calibrate_motors service
    call_service('/odrive/calibrate_motors')
    
