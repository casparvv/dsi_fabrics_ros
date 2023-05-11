## Connect to Mirte:
```
ssh mirte@192.168.42.1
pw
```

## Mirte side:
```
roslaunch lidar_processor mirte.launch
```
### Does all of this:
```
roslaunch odrive ...
rosservice call /odrive/connect_driver
rosservice call /odrive/calibrate_motors

sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar.launch

rosrun lidar_processor laser_scan_processor.py
```

## PC side:
```
export ROS_IP=192.168.42.95
export ROS_MASTER_URI=http://192.168.42.1:11311
```

## Mapping and localizing
### Start teleop with joystick
```
roslaunch amcl_starter teleop.launch
```

### Start gmapper to create a map
```
roslaunch amcl_starter mapping.launch
```
### Save the map in current directory:
```
rosrun map_server map_saver -f name_map
```

### Run amcl for localizing
```
roslaunch amcl_starter amcl_omni_example.launch
```

## Sync time Mirte and PC
```
sudo vim /etc/systemd/timesyncd.conf
```
#### Adjust: NTP=IP_PC
#### Make sure ntp is installed on PC, with:
```
sudo apt-get install ntp
```

## Misc
### Move files or directory from PC to Mirte:
```
scp Develop/catkin_ws/src/lidar_processor/src/lidar_processor.cpp mirte@192.168.42.1:~/catkin_lidar/src/lidar_processor/src
scp -r Desktop/navigation mirte@192.168.42.1:~/catkin_lidar/src/
```

