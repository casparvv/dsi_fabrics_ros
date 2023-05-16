## Connect to robot (Mirte):
```
ssh mirte@192.168.42.1
```

## Robot side:
```
roslaunch lidar_processor mirte.launch
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

## PC side:
```
export ROS_IP=192.168.42.95
export ROS_MASTER_URI=http://192.168.42.1:11311
```

### Direct Sensor Integrated (DSI) Optimization Fabrics
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
#### Startup
```
poetry shell
```
```
python runner.py
```

### Mapping and localizing
#### Start teleop with joystick
```
roslaunch amcl_starter teleop.launch
```
#### Start gmapper to create a map
```
roslaunch amcl_starter mapping.launch
```
#### Save the map in current directory:
```
rosrun map_server map_saver -f name_map
```
#### Run amcl for localizing
```
roslaunch amcl_starter amcl_omni_example.launch
```

## Misc

### Sync time robot and PC
```
sudo vim /etc/systemd/timesyncd.conf
```
#### Adjust: NTP=IP_PC
#### Make sure ntp is installed on PC, with:
```
sudo apt-get install ntp
```

### Move files or directory from PC to Mirte:
```
scp Develop/catkin_ws/src/lidar_processor/src/lidar_processor.cpp mirte@192.168.42.1:~/catkin_lidar/src/lidar_processor/src
scp -r Desktop/navigation mirte@192.168.42.1:~/catkin_lidar/src/
```

### Charging Mirte robot
1.
2.
3.

