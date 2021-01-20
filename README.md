# odometry_calibration

Tool for calibrating wheel odometry. Determines the correction factors required to minimize the pose error of a two-wheeled differential drive robot.

The linear and angular veocities of a 2-wheel differential drive robot are calculated as:

<img src="https://latex.codecogs.com/gif.latex?V_%7Blin%7D%20%3D%20%5Cfrac%7Br%7D%7B2%7D%20*%20(%7B%5Comega%7D_r%20%2B%20%7B%5Comega%7D_l)%20*%20C_%7Bf%2Clin%7D" /> \
<img src="https://latex.codecogs.com/gif.latex?%5Comega%20%3D%20%5Cfrac%7Br%7D%7Bd%7D%20*%20(%7B%5Comega%7D_r%20-%20%7B%5Comega%7D_l)%20*%20C_%7Bf%2Cang%7D" />

<img src="https://latex.codecogs.com/gif.latex?\text{Where,}" /> \
<img src="https://latex.codecogs.com/gif.latex?V_%7Blin%7D%20%3D%20%5Ctext%7BLinear%20velocity%7D" /> \
<img src="https://latex.codecogs.com/gif.latex?%5Comega%20%3D%20%5Ctext%7BAngular%20velocity%7D" /> \
<img src="https://latex.codecogs.com/gif.latex?r%20%3D%20%5Ctext%7BWheel%20radius%7D" /> \
<img src="https://latex.codecogs.com/gif.latex?d%20%3D%20%5Ctext%7Bwheel%20base%7D" /> \
<img src="https://latex.codecogs.com/gif.latex?%7B%5Comega%7D_%7Br%2Cf%7D%20%3D%20%5Ctext%7BWheel%20angular%20velocities%7D" /> \
<img src="https://latex.codecogs.com/gif.latex?C_%7Bf%2Clin%2C%20ang%7D%20%3D%20%5Ctext%7BConstant%20factors%20that%20minimize%20the%20calculated%20pose%20error%7D" /> 

The purpose of this tool is to calculate the above mentioned correction factors, to minimize the pose error from the ground truth.

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

The tool expects odometry data of the type `nav_msg/Odometry` either from a live feed or a recorded bag file. The data must be published to the `odometry_topic` topic for calibration.

To publish robot ground truth pose in a new terminal, run \
`rostopic pub /calibrate_odom geometry_msgs/Vector3 <x> <y> <theta>` \
The linear and angular correction factors are printed on the terminal window and two plots are saved in the package directory.

Default configuration is stored in `mw_tools/config/config.yaml`

#### Parameters
- `odometry_topic` \
  Odometry topic name for linear and angular calibration. Defaults to `/odometry`.
- `calibration_topic` \
  Robot pose topic name. Publish your ground truth pose to this topic. Defaults to `/calibrate_odom`
- `wheel_radius` \
  Wheel radius in meters. Defaults to 0.08
- `wheel_dist` \
  Wheel base in meters. Defaults to 0.43
- `fig_name` \
  Plot filename prefix. Defaults to "odom"
