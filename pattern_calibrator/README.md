# Pattern Calibrator

## Required Packages

* ROS2
* Eigen3
* OpenCV
* yaml-cpp
* DC1394 (Optional)

## Build
example
``` bash
cd path/to/active_marker_gui/pattern_calibrator
colcon build
```
If build is failed, you should try the command `colcon build --cmake-clean-cache --cmake-clean-first` or remove `build`,`install`,`log` directories.

## Launch
```bash
cd path/to/active_marker_gui/pattern_calibrator
. install/setup.bash
ros2 launch pattern_calibrator launch.py
```
Then, GUi window will pop up.

>[!note]
>To operate properly, the robot and the computer must be connected to the same network.

## Calibrate
1. Select color you want to calibrate
2. **Right click** selected color on reference marker.
3. **Left click** selected color on active marker.
4. Keep left clicking until finishing calibrate.
5. Do it with all color.

## Key map

|Key|Description|
|----|----|
|b|Blue|
|y|Yellow|
|p|Pink|
|g|Green|
|ctrl + Drag|zoom|
|z|reset Zooming|
|r|Reset calibration<br>reload parameeter from config file|
|s|Save current calibrated parameter|
|ESC|QUIT program|


## Contact

unitybighead@gmail.com
