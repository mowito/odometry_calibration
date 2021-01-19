# odometry_calibration

Tool for calibrating wheel odometry. Determines the correction factors required to minimize the pose error of a two-wheeled differential drive robot.

## Build instructions
```
mkdir -p mowito_ws/src && cd mowito_ws/src
git clone git@github.com:mowito/odometry_calibration.git
cd ..
source /opt/ros/noetic/setup.bash
catkin build
source ./devel/setup.bash
```

## How to run

To run the odometry calibration node, run \
`roslaunch odometry_calibration calibrate_odometry.launch`

To publish robot odometry in a new terminal, run \
`rostopic pub /calibrate_odom geometry_msgs/Vector3 <x> <y> <theta>` \
Two plots will be saved in the package directory.

Default configuration is stored in `mw_tools/config/config.yaml`

#### Parameters
- `linear_odometry_topic` \
  Odometry topic name for linear calibration. Defaults to `/odometry/linear`.
- `angular_odometry_topic` \
  Odometry topic name for angular calibration. Defaults to `/odometry/angular`
- `calibration_topic` \
  Robot pose topic name. Publish your ground truth pose to this topic. Defaults to `/calibrate_odom`
- `wheel_radius` \
  Wheel radius in meters. Defaults to 0.08
- `wheel_dist` \
  Wheel base in meters. Defaults to 0.43
- `fig_name` \
  Plot filename prefix. Defaults to "odom"
