#
### Node and Topic
## Dependency

## Setup
```
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/lucia_vital_calibration.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```

## Usage
```
$ ros2 run lucia_vital_calibration lucia_vital_calibration
```
### Request calibration command
```
$ ros2 topic pub /reset_all_sensors std_msgs/msg/Empty "{}" --once
$ ros2 topic echo /reset_sensor_ack
```
## License
## Authors

## References
