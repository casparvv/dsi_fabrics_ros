# DSI Fabrics ROS
Repository for real-world robot experiments using Direct Sensor Integrated (DSI) Optimization Fabrics via ROS

## Connect to robot
1. Connect to robot WiFi
2. SSH into robot
```
ssh mirte@192.168.42.1
```

## Robot side
- Start up the drivers, calibrate the motors, spin up the LiDAR sensor and processor
```
roslaunch lidar_processor robot.launch
```

<details>
<summary>Does all of this:</summary>

```
roslaunch odrive_ros odrive.launch
rosservice call /odrive/connect_driver
rosservice call /odrive/calibrate_motors

sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar.launch

rosrun lidar_processor laser_scan_processor.py
```

</details>

## PC side
```
export ROS_IP=192.168.42.95
export ROS_MASTER_URI=http://192.168.42.1:11311
```

### Install and Run DSI Optimization Fabrics on robot
#### Installation w. Poetry
```
git clone git@github.com:casparvv/dsi_fabrics_ros.git
```
```
cd dsi_fabrics_ros
```
```
poetry install
```
#### Startup from `dsi_fabrics_ros`
```
poetry shell
```
```
python runner.py
```

### Mapping and localizing
- Start teleop with joystick
```
roslaunch amcl_starter teleop.launch
```
- Start gmapper to create a map
```
roslaunch amcl_starter mapping.launch
```
- Save the map in current directory
```
rosrun map_server map_saver -f name_map
```
- Run amcl for localizing
```
roslaunch amcl_starter amcl_omni_example.launch
```

## Misc
- Sync time robot and PC, adjust `NTP=IP_PC`
```
sudo vim /etc/systemd/timesyncd.conf
```
- Make sure ntp is installed on PC, with:
```
sudo apt-get install ntp
```
- Move files or directory from PC to robot:
```
scp Develop/catkin_ws/src/lidar_processor/src/lidar_processor.cpp mirte@192.168.42.1:~/catkin_lidar/src/lidar_processor/src
scp -r Desktop/navigation mirte@192.168.42.1:~/catkin_lidar/src/
```

## To do
- [ ] Tune fabrics
- [ ] Improve localization
- [ ] Add charging info
- [ ] Add robot side code
- [ ] Add amcl_starter code
